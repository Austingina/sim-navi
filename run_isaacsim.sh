#!/usr/bin/env bash
# 在“干净”环境里启动 Isaac Sim 5.1，避免 conda / 系统 CUDA / ROS 库污染。
# 解决两类崩溃:
#   - libcusparse.so.12: undefined symbol __nvJitLinkCreate_12_8 (系统 CUDA 12.2 顶掉了自带 12.8)
#   - ld.so: _dl_allocate_tls_init assertion (LD_LIBRARY_PATH / conda 库混入导致 TLS 崩溃)
#
# 用法:
#   run_isaacsim.sh                      # GUI 打开空白 Isaac Sim
#   run_isaacsim.sh scene.usd            # GUI 打开指定场景
#   run_isaacsim.sh --stream scene.usd   # 无头 + WebRTC 串流(远程 Streaming Client 连, 手动 Play)
#   run_isaacsim.sh --headless scene.usd # 纯无头(无渲染窗口)+ 自动 Play, 只走 ROS(最省 GPU)
#
# 两种无头的区别:
#   --stream   : 仍渲染并编码画面给 WebRTC, 能远程看; 需在 Streaming Client 里点 Play。
#   --headless : 完全不开窗口/不渲染视口, 脚本自动 Play, 话题立刻开始发; 看不到画面,
#                靠 rviz/ROS 观察(适合"太卡"的场景)。

set -e

ISAAC_DIR="$HOME/isim"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# 解析可选的模式开关(必须放在第一个参数): --gui(默认) / --stream / --headless
MODE="gui"
case "${1:-}" in
    --gui)                  MODE="gui";      shift ;;
    --stream|--streaming)   MODE="stream";   shift ;;
    --headless|--no-window) MODE="headless"; shift ;;
    --probe)                MODE="probe";    shift ;;   # 用和 --headless 相同的干净环境跑 perf_probe.py
esac

# 1) 退出 conda（base 会污染 PATH / 库）
if [[ -n "$CONDA_PREFIX" ]]; then
    # conda deactivate 是 shell 函数，这里直接把 conda 的路径从环境剥离
    source "$(conda info --base)/etc/profile.d/conda.sh" 2>/dev/null || true
    conda deactivate 2>/dev/null || true
    conda deactivate 2>/dev/null || true
fi

# 2) 清掉会顶掉 Isaac 自带库的系统库路径（cuda-12.2 / ROS 等）
unset LD_LIBRARY_PATH
unset PYTHONPATH

# 2.1) 让 isaacsim.ros2.bridge 找到自带的 RMW(DDS) 库。
#      上面清空了 LD_LIBRARY_PATH，bridge 就 dlopen 不到 librmw_fastrtps_cpp.so，
#      会启动失败 -> /clock /odom /tf 等 ROS2 OmniGraph 节点全部缺失。
#      这里只加回 bridge 扩展自带的 humble 库目录（干净，不引入系统 ROS，避免 TLS 崩溃）。
#      注意: 不要在这个脚本里 source /opt/ros/humble/setup.bash，那会重新污染库路径。
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp   # Isaac 自带 FastDDS；cyclonedds 需另装 libcyclonedds.so，否则桥启动失败
export LD_LIBRARY_PATH="$ISAAC_DIR/exts/isaacsim.ros2.bridge/humble/lib"

# 3) 若当前终端没有 DISPLAY(典型: 从 Windows SSH 进来), 自动指向本机物理 X 桌面。
#    窗口会出现在 4090 接的物理显示器上, 需要人在那块屏前才能看到/操作。
#    仅 GUI 模式需要 DISPLAY; 无头(--stream/--headless)不需要, 跳过。
if [[ "$MODE" == "gui" && -z "$DISPLAY" ]]; then
    sock=$(ls /tmp/.X11-unix/ 2>/dev/null | grep -E '^X[0-9]+$' | head -1)
    if [[ -n "$sock" ]]; then
        export DISPLAY=":${sock#X}"
        # 从正在运行的 Xorg 命令行里抓 -auth 路径, 比写死更稳
        xauth_path=$(ps -eo cmd 2>/dev/null | grep -oE '\-auth +[^ ]+' | head -1 | awk '{print $2}')
        [[ -n "$xauth_path" && -e "$xauth_path" ]] && export XAUTHORITY="$xauth_path"
        echo "[info] 终端无 DISPLAY, 已自动指向物理桌面: DISPLAY=$DISPLAY XAUTHORITY=${XAUTHORITY:-未设置}"
        echo "[info] 窗口将出现在 4090 的物理显示器上。若你人不在屏前, 请改用串流: run_isaacsim_stream.sh"
    else
        echo "[warn] 找不到本机 X 显示, 且终端无 DISPLAY。GUI 无法显示, 请改用串流: run_isaacsim_stream.sh"
    fi
fi

# 4) 跳过 RTX 驱动版本检查
#    驱动 535.288.01 实际没问题, 但 NVIDIA Vulkan driverVersion 次版本号只有 8 位,
#    288 & 0xFF = 32, Omniverse 把它误读成 535.32 落入"不支持区间", 故关闭该校验。
EXTRA_ARGS=(--/rtx/verifyDriverVersion/enabled=false)

# 5) 启动
#    注意: 必须把文件转成"绝对路径"。Isaac 的工作目录不是你的当前目录,
#    传相对路径它会找不到, 然后静默打开一个空的默认场景。
TARGET="${1:-}"
if [[ -n "$TARGET" ]]; then
    if [[ "$TARGET" != /* ]]; then
        if [[ -f "$PWD/$TARGET" ]]; then
            TARGET="$PWD/$TARGET"                                   # 相对当前目录
        else
            TARGET="$SCRIPT_DIR/$TARGET"                            # 退回脚本所在目录
        fi
    fi
    if [[ ! -f "$TARGET" ]]; then
        echo "[warn] 找不到文件: $TARGET  (将打开空场景)"
    else
        echo "[info] 打开: $TARGET"
    fi
fi

# scene_seg.usd：NuRec 相机场景默认配置(可被环境变量覆盖)。
#   VIEWPORT=1       开渲染(相机 render product 需要)
#   HZ=0             不主动限速；isaac_headless.py 内封顶 60Hz
#   RENDER_EVERY=6   每 6 物理步渲 1 次 -> 相机 ~10Hz，物理/雷达仍满频
if [[ "$MODE" == "headless" && "$(basename "${TARGET:-}")" == "scene_seg.usd" ]]; then
    export ISAAC_VIEWPORT="${ISAAC_VIEWPORT:-0}"
    export ISAAC_HZ="${ISAAC_HZ:-0}"
    export ISAAC_RENDER_EVERY="${ISAAC_RENDER_EVERY:-6}"
    echo "[info] scene_seg 相机模式: ISAAC_VIEWPORT=$ISAAC_VIEWPORT"
    echo "[info]   ISAAC_HZ=$ISAAC_HZ(封顶60)  ISAAC_RENDER_EVERY=$ISAAC_RENDER_EVERY"
fi

case "$MODE" in
  headless)
    # 纯无头 + 自动 Play: 走 standalone python(SimulationApp headless=True), 只发 ROS。
    echo "[info] 纯无头模式(无渲染窗口, 自动 Play, 只走 ROS)。Ctrl+C 退出。"
    exec "$ISAAC_DIR/python.sh" "$SCRIPT_DIR/isaac_headless.py" "$TARGET" "${EXTRA_ARGS[@]}"
    ;;
  probe)
    # 性能探针: 与 --headless 完全相同的干净环境(conda 已退、PYTHONPATH 已清、RMW 已配),
    # 用来做真·同条件对比。跑 A/B/C 三段后自动退出。
    echo "[info] 探针模式(干净环境, 同 --headless)。跑完自动退出。"
    exec "$ISAAC_DIR/python.sh" "$SCRIPT_DIR/perf_probe.py" "$TARGET" "${EXTRA_ARGS[@]}"
    ;;
  stream)
    # 无头 + WebRTC 串流: 官方 streaming kit 自带 --no-window。需在 Streaming Client 里 Play。
    echo "[info] 串流模式(WebRTC)。用 Isaac Sim Streaming Client 连本机 IP, 在客户端点 Play。"
    if [[ -n "$TARGET" && -f "$TARGET" ]]; then
        exec "$ISAAC_DIR/isaac-sim.streaming.sh" "${EXTRA_ARGS[@]}" --/app/file/openPath="$TARGET"
    else
        exec "$ISAAC_DIR/isaac-sim.streaming.sh" "${EXTRA_ARGS[@]}"
    fi
    ;;
  *)
    if [[ -n "$TARGET" && -f "$TARGET" ]]; then
        exec "$ISAAC_DIR/isaac-sim.sh" "${EXTRA_ARGS[@]}" --/app/file/openPath="$TARGET"
    else
        exec "$ISAAC_DIR/isaac-sim.sh" "${EXTRA_ARGS[@]}"
    fi
    ;;
esac
