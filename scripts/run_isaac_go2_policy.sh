#!/usr/bin/env bash
# 阶段 C：干净环境启动 Isaac Go2 策略 + Mid360（同进程 /cmd_vel）。
# 环境对齐 run_isaacsim.sh，避免 ROS2 bridge / SRE 崩溃。
set -euo pipefail

ISAAC_DIR="${ISAAC_DIR:-$HOME/isaacsim/current}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# 退出 conda
if [[ -n "${CONDA_PREFIX:-}" ]]; then
  source "$(conda info --base)/etc/profile.d/conda.sh" 2>/dev/null || true
  conda deactivate 2>/dev/null || true
  conda deactivate 2>/dev/null || true
fi

unset LD_LIBRARY_PATH
unset PYTHONPATH

export ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_BRIDGE_LIB="$ISAAC_DIR/exts/isaacsim.ros2.core/$ROS_DISTRO/lib"
if [[ ! -d "$ROS_BRIDGE_LIB" ]]; then
  echo "[error] missing $ROS_BRIDGE_LIB" >&2
  exit 1
fi
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export LD_LIBRARY_PATH="$ROS_BRIDGE_LIB"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$REPO_DIR/cyclonedds_localhost.xml}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
# 默认开渲染以便 /zed/rgb/image_raw；只要雷达可 ISAAC_VIEWPORT=0 省 GPU
export ISAAC_VIEWPORT="${ISAAC_VIEWPORT:-1}"

USD="${1:-$REPO_DIR/scene_daxuecheng_go2.usd}"
if [[ "$USD" != /* ]]; then USD="$REPO_DIR/$USD"; fi
POLICY="${ISAAC_GO2_POLICY:-$REPO_DIR/policies/go2_velocity/2026-09-22_11-04-32/exported/policy.pt}"
DEPLOY="${ISAAC_GO2_DEPLOY_YAML:-$REPO_DIR/policies/go2_velocity/2026-09-22_11-04-32/params/deploy.yaml}"

echo "[info] ROS bridge lib=$ROS_BRIDGE_LIB rmw=$RMW_IMPLEMENTATION domain=$ROS_DOMAIN_ID"
echo "[info] ISAAC_VIEWPORT=$ISAAC_VIEWPORT (1=相机渲染 /zed/*)"
echo "[info] usd=$USD"
echo "[info] policy=$POLICY"

exec "$ISAAC_DIR/python.sh" "$SCRIPT_DIR/isaac_go2_policy_cmd_vel.py" \
  --usd "$USD" \
  --policy "$POLICY" \
  --deploy-yaml "$DEPLOY" \
  --/rtx/verifyDriverVersion/enabled=false
