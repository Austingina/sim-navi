#!/usr/bin/env bash
# 大学城 Go2：一个窗口拉起终端 1–4。
#   ./scripts/run_go2_rviz_stack.sh
#   ./scripts/run_go2_rviz_stack.sh --route [路线名]     拉起后交互式定路线
#   ./scripts/run_go2_rviz_stack.sh --nav               只重启室外导航栈
#   ./scripts/run_go2_rviz_stack.sh --heading           查询 heading 是否 locked
#   ./scripts/run_go2_rviz_stack.sh --keyboard          键盘控制移动
#   ./scripts/run_go2_rviz_stack.sh --standup           原地扶正（保留 xy）
#   ./scripts/run_go2_rviz_stack.sh --origin            回到出生点站好
# 已在跑的策略 / bringup / outdoor / RViz 会跳过，避免再开一份 Isaac 占住 GPU。
set -u

REPO="$(cd "$(dirname "$0")/.." && pwd)"
FAST_LIO="${FAST_LIO_ROOT:-$HOME/Desktop/fast_lio2}"
NAV_PARAMS="$FAST_LIO/ros2_ws/src/FAST_LIO/config/param_outdoor_sim_go2.yaml"
RVIZ_CFG="$FAST_LIO/ros2_ws/src/FAST_LIO/rviz/outdoor.rviz"
ROUTE_NAME="${SEMANTIC_ROUTE_NAME:-route_01}"
INTERACTIVE_ROUTE=0
ACTION=launch

usage() {
  cat <<EOF
用法: $(basename "$0") [--route [路线名] | --nav | --heading | --keyboard | --standup | --origin]

  （无参数）           拉起终端 1–4。已在跑的会跳过
  --route              拉起后进入交互式定路线，默认路线名 route_01
  --route <路线名>     同上，并指定默认路线名
  --nav                只停掉并重新拉起室外导航栈（终端 3），策略和 RViz 不动
  --heading            查询 /heading_align/status 是不是 locked
  --keyboard           键盘控制。导航栈在跑时发到 /cmd_vel_smoothed
  --standup            原地扶正。不重启 Isaac、不重启导航，仿真时间连续，RViz 定位保持
  --origin             回到场景出生点并站好。不重启 Isaac
  -h, --help           本说明
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --route|-r)
      if [[ "$ACTION" != "launch" ]]; then
        echo "--route 不能和其它单独参数一起用。" >&2
        exit 1
      fi
      INTERACTIVE_ROUTE=1
      if [[ $# -ge 2 && "$2" != -* ]]; then
        ROUTE_NAME="$2"
        shift
      fi
      ;;
    --nav)
      [[ "$ACTION" != "launch" || "$INTERACTIVE_ROUTE" -eq 1 ]] && { echo "参数冲突。" >&2; exit 1; }
      ACTION=nav
      ;;
    --heading)
      [[ "$ACTION" != "launch" || "$INTERACTIVE_ROUTE" -eq 1 ]] && { echo "参数冲突。" >&2; exit 1; }
      ACTION=heading
      ;;
    --keyboard)
      [[ "$ACTION" != "launch" || "$INTERACTIVE_ROUTE" -eq 1 ]] && { echo "参数冲突。" >&2; exit 1; }
      ACTION=keyboard
      ;;
    --standup)
      [[ "$ACTION" != "launch" || "$INTERACTIVE_ROUTE" -eq 1 ]] && { echo "参数冲突。" >&2; exit 1; }
      ACTION=standup
      ;;
    --origin)
      [[ "$ACTION" != "launch" || "$INTERACTIVE_ROUTE" -eq 1 ]] && { echo "参数冲突。" >&2; exit 1; }
      ACTION=origin
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "未知参数: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
  shift
done

proc_running() {
  # 只认真正的进程。pgrep -f 会命中 Cursor 沙箱里“提到过这个文件名”的命令，从而误判已在跑。
  local pat="$1"
  ps -eo args | awk -v pat="$pat" '
    /cursorsandbox/ || /py_compile/ || /run_go2_rviz_stack\.sh/ {next}
    $0 ~ pat {found=1}
    END {exit !found}
  '
}

pids_matching() {
  local pat="$1"
  ps -eo pid,args | awk -v pat="$pat" '
    /cursorsandbox/ || /py_compile/ || /run_go2_rviz_stack\.sh/ {next}
    $0 ~ pat {print $1}
  '
}

source_ros() {
  # Humble 的 setup.bash 会读未赋值的 AMENT_TRACE_SETUP_FILES，和 set -u 冲突。
  set +u
  # shellcheck disable=SC1091
  source "$REPO/setup_ros_local.sh"
  # shellcheck disable=SC1091
  source "$FAST_LIO/ros2_ws/install/setup.bash"
  set -u
}

outdoor_cmd() {
  echo "cd ${FAST_LIO@Q}/scripts && source ${REPO@Q}/setup_ros_local.sh && source ${FAST_LIO@Q}/ros2_ws/install/setup.bash && ros2 launch fast_lio outdoor_bringup_heading_align.launch.py use_sim_time:=true sim:=true enable_elevation:=false nav_params:=${NAV_PARAMS}${hold_shell}"
}

hold_shell='; echo; echo "[窗口保留] 进程已退出，可看上面的日志。"; exec bash'

open_tabs() {
  local opened=0
  add() {
    local title="$1" body="$2"
    # 每个窗口单独起。一次 gnome-terminal 里用 `-- bash -c` 会把后面的 --tab 吞掉，RViz 起不来。
    gnome-terminal --window "--title=$title" -- bash -c "$body" &
    opened=1
  }

  if ! command -v gnome-terminal >/dev/null 2>&1; then
    echo "[error] 没有 gnome-terminal，无法打开窗口。" >&2
    exit 1
  fi

  if proc_running 'isaac_go2_policy_cmd_vel.py'; then
    echo "[跳过] 终端1 策略已在跑"
  else
    echo "[打开] 终端1 Go2 策略"
    add "1 Go2策略" "cd ${REPO@Q} && ./scripts/run_isaac_go2_policy.sh scene_daxuecheng_go2.usd${hold_shell}"
  fi

  if proc_running 'bringup_go2.launch.py'; then
    echo "[跳过] 终端2 bringup 已在跑"
  else
    echo "[打开] 终端2 bringup"
    add "2 bringup" "cd ${REPO@Q} && source ${REPO@Q}/setup_ros_local.sh && source ${FAST_LIO@Q}/ros2_ws/install/setup.bash && ros2 launch ros2_sensors/bringup_go2.launch.py${hold_shell}"
  fi

  if proc_running 'outdoor_bringup_heading_align.launch.py'; then
    echo "[跳过] 终端3 室外栈已在跑（若是改语义路线之前拉起的，请在那个窗口 Ctrl+C 后重跑本脚本）"
  else
    echo "[打开] 终端3 室外栈"
    add "3 outdoor" "$(outdoor_cmd)"
  fi

  if proc_running 'rviz2 .*outdoor.rviz'; then
    echo "[跳过] 终端4 RViz 已在跑（语义点黄球要重开 RViz 才看得到）"
  else
    echo "[打开] 终端4 RViz"
    add "4 RViz" "source ${REPO@Q}/setup_ros_local.sh && source ${FAST_LIO@Q}/ros2_ws/install/setup.bash && rviz2 -d ${RVIZ_CFG@Q} --ros-args -p use_sim_time:=true${hold_shell}"
  fi

  if [[ "$opened" -eq 0 ]]; then
    echo "[信息] 四个终端都已在跑。"
    return 0
  fi
  wait
}

pub_route() {
  local payload="$1"
  ros2 topic pub --once /semantic_route_cmd std_msgs/msg/String \
    "$(/usr/bin/python3 -c 'import json,sys; print(json.dumps({"data": sys.argv[1]}))' "$payload")"
}

show_help() {
  cat <<EOF

路线名：$ROUTE_NAME
先在 RViz 用「2D Goal Pose」把狗走到要记的位置，终端 3 出现 Campus route complete 后，再在这里确认。

  <名字>          确认狗脚下的位置为语义点（例如：北门口）
  mark <名字>     同上
  list            打印当前点序
  undo            去掉最后一点
  clear           清空
  save [名字]     保存。省略则用当前路线名
  run [名字]      按顺序发出。省略则发当前路线
  stop            停下，点还留着
  name <名字>     改默认路线名
  status          看 heading 和路线状态
  help            本说明
  quit            退出菜单（四个终端继续跑）

EOF
}

cmd_status() {
  echo "--- heading ---"
  timeout 5 ros2 topic echo /heading_align/status --once || echo "(还没有 heading，看终端 3 是否 LOCKED)"
  echo "--- semantic_route_status ---"
  timeout 3 ros2 topic echo /semantic_route_status --once || echo "(还没有状态：先 mark 一个点，或终端 3 还是旧进程)"
}

dispatch() {
  local line="$1" verb arg
  # 去掉首尾空白
  line="${line#"${line%%[![:space:]]*}"}"
  line="${line%"${line##*[![:space:]]}"}"
  [[ -z "$line" ]] && return 0
  verb="${line%% *}"
  if [[ "$line" == *" "* ]]; then
    arg="${line#* }"
  else
    arg=""
  fi
  case "$verb" in
    quit|q|exit)
      echo "菜单退出。终端窗口继续跑。"
      exit 0
      ;;
    help|h|\?)
      show_help
      ;;
    name)
      if [[ -z "$arg" ]]; then
        echo "用法: name 路线名"
      else
        ROUTE_NAME="$arg"
        echo "默认路线名改为：$ROUTE_NAME"
      fi
      ;;
    status)
      cmd_status
      ;;
    list|undo|clear|stop)
      pub_route "$verb"
      ;;
    save)
      pub_route "save ${arg:-$ROUTE_NAME}"
      ;;
    run|send)
      pub_route "run ${arg:-$ROUTE_NAME}"
      ;;
    mark|confirm|确认)
      if [[ -z "$arg" ]]; then
        read -r -p "语义点名称: " arg
      fi
      if [[ -z "${arg:-}" ]]; then
        echo "需要一个名字。"
      else
        pub_route "mark $arg"
      fi
      ;;
    *)
      pub_route "mark $line"
      ;;
  esac
}

restart_nav() {
  local pat='outdoor_bringup_heading_align\.launch\.py|nav2_container|component_container_isolated|campus_route_planner|gps_heading_align\.py|map_odom_from_gps\.py|map_to_campus_map_tf'
  mapfile -t nav_pids < <(pids_matching "$pat")
  if [[ ${#nav_pids[@]} -gt 0 ]]; then
    echo "停掉导航栈: ${nav_pids[*]}"
    kill "${nav_pids[@]}" 2>/dev/null || true
    sleep 2
    mapfile -t nav_pids < <(pids_matching "$pat")
    if [[ ${#nav_pids[@]} -gt 0 ]]; then
      kill -9 "${nav_pids[@]}" 2>/dev/null || true
      sleep 1
    fi
  else
    echo "导航栈当前没在跑，直接拉起。"
  fi
  if ! command -v gnome-terminal >/dev/null 2>&1; then
    echo "[error] 没有 gnome-terminal，无法打开窗口。" >&2
    exit 1
  fi
  echo "[打开] 终端3 室外栈"
  gnome-terminal --window "--title=3 outdoor" -- bash -c "$(outdoor_cmd)" &
  wait
  echo "导航栈已重新拉起。等这个窗口出现 heading LOCKED。"
}

query_heading() {
  source_ros
  local status
  status="$(timeout 8 ros2 topic echo /heading_align/status --once --field data 2>/dev/null || true)"
  status="${status//$'\r'/}"
  # echo 会在字段后面再打一行 ---，不能算进状态。
  status="${status%%$'\n'*}"
  status="${status#"${status%%[![:space:]]*}"}"
  status="${status%"${status##*[![:space:]]}"}"
  if [[ -z "$status" ]]; then
    echo "heading: 没有读到 /heading_align/status（导航栈可能没起来）"
    exit 1
  fi
  case "$status" in
    locked)
      echo "heading: locked"
      exit 0
      ;;
    frozen_bad_gps)
      echo "heading: frozen_bad_gps（航向已冻结，规划器按已对齐用，但不是 locked）"
      exit 1
      ;;
    *)
      echo "heading: ${status}（未 locked）"
      exit 1
      ;;
  esac
}

keyboard_drive() {
  if [[ ! -t 0 ]]; then
    echo "键盘控制要在终端里运行，当前没有交互式输入。" >&2
    exit 1
  fi
  source_ros
  local topic="/cmd_vel"
  if proc_running 'outdoor_bringup_heading_align\.launch\.py|nav2_container'; then
    topic="/cmd_vel_smoothed"
    echo "导航栈在跑，键盘发到 ${topic}（直接发 /cmd_vel 会被碰撞监视器盖掉）。"
  else
    echo "导航栈没在跑，键盘直接发到 ${topic}。"
  fi
  echo "i 前进  , 后退  j/l 转向  k 或空格停下。q/z 加减速度，起步 0.4 m/s。Ctrl+C 退出。"
  exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
    -r cmd_vel:="${topic}" \
    -p speed:=0.4 \
    -p turn:=0.4 \
    -p stamped:=false
}

signal_policy() {
  local sig="$1" label="$2"
  local pidfile="/tmp/go2_policy_pose_ctl.pid" pid
  if [[ ! -f "$pidfile" ]]; then
    echo "当前策略还没有扶正/回原点。请先在终端 1 里 Ctrl+C，再重新执行：" >&2
    echo "  ./scripts/run_isaac_go2_policy.sh scene_daxuecheng_go2.usd" >&2
    echo "日志出现「扶正=SIGUSR2」之后，再跑本参数。旧进程收到信号会被直接杀掉。" >&2
    exit 1
  fi
  pid="$(tr -d '[:space:]' < "$pidfile")"
  if [[ -z "$pid" ]] || ! kill -0 "$pid" 2>/dev/null; then
    echo "策略进程不在了。请先重新拉起终端 1。" >&2
    exit 1
  fi
  kill "-${sig}" "$pid"
  echo "已让策略${label}（pid ${pid}）。看终端 1 的 [${label}] 日志。"
}

read_clock_sec() {
  local raw
  raw="$(timeout 4 ros2 topic echo /clock --once --field clock.sec 2>/dev/null || true)"
  raw="${raw%%$'\n'*}"
  raw="${raw//[^0-9]/}"
  printf '%s' "$raw"
}

map_base_xy() {
  /usr/bin/python3 - <<'PY'
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
rclpy.init()
n = Node("standup_tf_check")
buf = Buffer()
TransformListener(buf, n)
import time
t0 = time.time()
pos = None
while time.time() - t0 < 2.0:
    rclpy.spin_once(n, timeout_sec=0.05)
    try:
        t = buf.lookup_transform("map", "base_link", rclpy.time.Time())
        tr = t.transform.translation
        pos = (tr.x, tr.y)
        break
    except Exception:
        pass
n.destroy_node()
rclpy.shutdown()
if pos is None:
    print("FAIL")
else:
    print(f"{pos[0]:.1f} {pos[1]:.1f}")
PY
}

cancel_motion() {
  ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}" >/dev/null 2>&1 || true
  ros2 service call /navigate_through_poses/_action/cancel_goal action_msgs/srv/CancelGoal "{}" >/dev/null 2>&1 || true
  ros2 topic pub --once /semantic_route_cmd std_msgs/msg/String "{data: stop}" >/dev/null 2>&1 || true
  ros2 topic pub --once /cmd_vel_smoothed geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true
}

standup_keep_loc() {
  source_ros
  local c0 c1 p0 p1
  c0="$(read_clock_sec)"
  p0="$(map_base_xy)"
  cancel_motion
  signal_policy USR2 standup
  sleep 1.5
  c1="$(read_clock_sec)"
  p1="$(map_base_xy)"
  if [[ -n "$c0" && -n "$c1" && "$c1" -lt "$c0" ]]; then
    echo "仿真时间从 ${c0}s 倒退到 ${c1}s。这不是扶正造成的，是终端 1 被重启了。不要重启策略，只保留这次仿真。" >&2
    exit 1
  fi
  echo "仿真时间连续：${c0:-?}s -> ${c1:-?}s（没有回零）。"
  if [[ "$p0" == "FAIL" || "$p1" == "FAIL" ]]; then
    echo "map 到 base_link 这次没有对上。导航栈还在用旧时间时会出现这种情况。策略进程不要关，另开终端执行 --nav 把定位接到当前时钟上。" >&2
    exit 1
  fi
  echo "扶正前地图位置 ${p0}，扶正后 ${p1}。狗还在原地附近，RViz 不用重开。"
}

case "$ACTION" in
  nav) restart_nav; exit 0 ;;
  heading) query_heading ;;
  keyboard) keyboard_drive ;;
  standup) standup_keep_loc; exit 0 ;;
  origin) signal_policy USR1 origin; exit 0 ;;
esac

open_tabs

if [[ "$INTERACTIVE_ROUTE" -ne 1 ]]; then
  echo "四个终端已处理。要交互式定路线：$(basename "$0") --route [路线名]"
  exit 0
fi

source_ros

echo
read -r -p "默认路线名 [$ROUTE_NAME]: " typed_name
if [[ -n "${typed_name:-}" ]]; then
  ROUTE_NAME="$typed_name"
fi
echo "等终端 3 出现 heading LOCKED 后再确认语义点。点「2D Goal Pose」走到位置，再在这里输入点名。"
show_help

while true; do
  read -r -p "路线[$ROUTE_NAME]> " line || { echo; exit 0; }
  dispatch "$line"
done
