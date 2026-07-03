#!/bin/bash
# 启动 Isaac Sim 5.1 并直接打开一个 scene.usd
# 用法:
#   ./open_isaac_scene.sh                          # 有窗口,打开默认 robot_data/scene.usd
#   ./open_isaac_scene.sh /path/to/x.usd           # 有窗口,打开指定 usd
#   ./open_isaac_scene.sh --headless               # 无头(WebRTC 串流),打开默认 scene.usd
#   ./open_isaac_scene.sh /path/to/x.usd --headless
set -e

ISAAC_DIR="/home/ai-sz-a26317-u1/isaac_sim/isaac-sim-standalone-5.1.0-linux-x86_64"
SCENE="/home/ai-sz-a26317-u1/robot_data/scene.usd"
HEADLESS=false

for arg in "$@"; do
    case "$arg" in
        --headless) HEADLESS=true ;;
        *) SCENE="$arg" ;;
    esac
done

case "$SCENE" in
    /*) : ;;
    *) SCENE="$(cd "$(dirname "$SCENE")" && pwd)/$(basename "$SCENE")" ;;
esac
if [ ! -f "$SCENE" ]; then
    echo "找不到 usd 文件: $SCENE" >&2
    exit 1
fi

# 高斯场景 usdz 很大(~1.7GB)，缺了会 Stage 里只有机器人、视口像“空的/黑的”
SCENE_DIR="$(dirname "$SCENE")"
if grep -q 'zhicheng-square-collision.usdz' "$SCENE" 2>/dev/null; then
    USDZ="$SCENE_DIR/assets/zhicheng-square/zhicheng-square-collision.usdz"
else
    USDZ="$SCENE_DIR/assets/zhicheng/zhicheng-usd-collision.usdz"
fi
if [ ! -f "$USDZ" ]; then
    echo "WARN: 场景依赖的碰撞 usdz 不存在: $USDZ" >&2
    echo "      视口可能只有机器人或全黑。见 ros2_sensors/README.md 第 8 节生成。" >&2
fi

EXTRA_ARGS=(
    --/rtx/verifyDriverVersion/enabled=false
    --/app/content/emptyStageOnStart=false
    --/app/file/openPath="$SCENE"
)

if [ "$HEADLESS" = true ]; then
    echo "无头模式(WebRTC): 窗口会是黑的/没窗口，必须用 Isaac Sim Streaming Client 连本机 IP 看画面"
    echo "等终端出现 'Streaming App is loaded' 再连"
    cd "$ISAAC_DIR"
    exec ./isaac-sim.streaming.sh "${EXTRA_ARGS[@]}"
fi

if [ -z "${DISPLAY:-}" ]; then
    echo "ERROR: DISPLAY 未设置，GUI 起不来（SSH 无 X11 转发？）。" >&2
    echo "  本机桌面直接跑，或 ssh -X，或用 --headless + Streaming Client。" >&2
    exit 1
fi

echo "打开场景: $SCENE"
echo "提示: Isaac 启动约 20~30s 才开始加载 usdz；高斯体积再需 30~60s 才出画面。"
echo "      若仍黑: Stage 选中 World 按 F 聚焦；Viewport 菜单 Rendering 选 RTX Real-Time。"

# 与 Isaac 同 ROS 网络（可选，消除 rclpy 警告；不影响视口）
if [ -f /opt/ros/jazzy/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
fi

cd "$ISAAC_DIR"
exec ./isaac-sim.sh "${EXTRA_ARGS[@]}"
