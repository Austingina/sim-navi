# Isaac Sim 6 / Kit 110：app ready 后打开 USD，可选 setup + Play。
# run_isaacsim.sh: --exec "本脚本 --usd <path> [--setup <py>] [--play]"
#
# 关键：不要在 async 协程里同步跑 setup（og.Controller / RemovePrim）。
# 那样会占住 asyncio 事件循环，触发大量
#   RuntimeError: Cannot enter into task ... while another task ... is being executed
# 随后 Play 容易直接 segfault。
# 正确做法：async 只负责 open + 等扩展；setup/play 放到下一帧的 sync update 回调里。
from __future__ import annotations

import argparse
import asyncio
import importlib
import os
import runpy


def _log(msg: str) -> None:
    print(f"[isaac_open_stage] {msg}", flush=True)
    try:
        import carb

        carb.log_info(f"[isaac_open_stage] {msg}")
    except Exception:
        pass


def _parse() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--usd", default=os.environ.get("ISAAC_OPEN_USD", ""))
    p.add_argument("--setup", default=os.environ.get("ISAAC_SETUP_SENSORS", ""))
    p.add_argument("--play", action="store_true", default=False)
    args, _unknown = p.parse_known_args()
    if not args.play:
        args.play = os.environ.get("ISAAC_AUTO_PLAY", "").strip() in (
            "1",
            "true",
            "True",
            "yes",
        )
    return args


# 状态机：由 sync update 驱动，避免在 async 任务内做 USD 重写
_STATE: dict = {
    "phase": "idle",  # idle|wait_ready|opening|open_wait|setup|settle|play|done|error
    "usd": "",
    "setup": "",
    "do_play": False,
    "wait": 0,
    "sub": None,
}


async def _open_and_warm(usd: str) -> None:
    import omni.kit.app
    import omni.usd

    app = omni.kit.app.get_app()
    _log(f"opening: {usd}")
    ctx = omni.usd.get_context()
    ok, err = await ctx.open_stage_async(usd)
    if not ok:
        _log(f"open_stage FAILED: {err}")
        _STATE["phase"] = "error"
        return
    _log(f"open_stage OK: {usd}")

    for _ in range(40):
        await app.next_update_async()

    try:
        from isaacsim.core.utils.extensions import enable_extension

        for ext in (
            "isaacsim.ros2.bridge",
            "isaacsim.sensors.physics",
            "isaacsim.sensors.physx",
            "isaacsim.robot.schema",
        ):
            enable_extension(ext)
            _log(f"enable_extension {ext}")
    except Exception as e:
        _log(f"enable_extension warn: {e}")

    for _ in range(40):
        await app.next_update_async()

    # 再让出几帧，确保本协程完全卸下后再跑 setup
    for _ in range(10):
        await app.next_update_async()
    _STATE["phase"] = "setup"
    _log("ready for setup (next update)")


def _on_update(_e=None) -> None:
    phase = _STATE["phase"]
    if phase in ("idle", "done", "error", "opening"):
        return

    import omni.kit.app

    app = omni.kit.app.get_app()

    if phase == "wait_ready":
        if not (app.is_running() and app.is_app_ready()):
            _STATE["wait"] += 1
            if _STATE["wait"] > 6000:
                _log("TIMEOUT waiting for app ready")
                _STATE["phase"] = "error"
                _unsubscribe()
            return
        _STATE["phase"] = "opening"
        _STATE["wait"] = 0
        usd = _STATE["usd"]
        if not usd or not os.path.isfile(usd):
            _log(f"USD missing or not a file: {usd!r}")
            _STATE["phase"] = "error"
            _unsubscribe()
            return
        asyncio.ensure_future(_open_and_warm(usd))
        return

    if phase == "setup":
        setup = (_STATE["setup"] or "").strip()
        _STATE["phase"] = "settle"
        _STATE["wait"] = 120  # setup 后多等几帧再 Play，让 OG/PhysX 消化
        if not setup:
            _log("no setup script; skip")
            return
        if not os.path.isfile(setup):
            _log(f"setup script missing: {setup}")
            _STATE["phase"] = "error"
            _unsubscribe()
            return
        _log(f"running setup (sync update): {setup}")
        try:
            # 此处不在 async task 内 → Kit 其它协程可正常调度
            runpy.run_path(setup, run_name="__main__")
            _log("setup done")
        except Exception as e:
            _log(f"setup FAILED: {e}")
            import traceback

            traceback.print_exc()
            _STATE["phase"] = "error"
            _unsubscribe()
        return

    if phase == "settle":
        _STATE["wait"] -= 1
        if _STATE["wait"] > 0:
            return
        if _STATE["do_play"]:
            _STATE["phase"] = "play"
        else:
            _STATE["phase"] = "done"
            _log("all done (no auto play)")
            _unsubscribe()
        return

    if phase == "play":
        _STATE["phase"] = "done"
        try:
            timeline = importlib.import_module("omni.timeline")
            timeline.get_timeline_interface().play()
            _log("timeline.play()")
        except Exception as e:
            _log(f"auto play FAILED: {e}")
        _unsubscribe()
        return


def _unsubscribe() -> None:
    sub = _STATE.get("sub")
    if sub is not None:
        try:
            sub.unsubscribe()
        except Exception:
            pass
        _STATE["sub"] = None


def main() -> None:
    args = _parse()
    usd = (args.usd or "").strip()
    setup = (args.setup or "").strip()
    _log(f"argv usd={usd!r} setup={setup!r} play={args.play}")
    if not usd:
        _log("no --usd / ISAAC_OPEN_USD; skip")
        return

    _STATE["usd"] = usd
    _STATE["setup"] = setup
    _STATE["do_play"] = bool(args.play)
    _STATE["phase"] = "wait_ready"
    _STATE["wait"] = 0

    import omni.kit.app

    _STATE["sub"] = (
        omni.kit.app.get_app()
        .get_update_event_stream()
        .create_subscription_to_pop(_on_update, name="isaac_open_stage")
    )


main()
