#!/usr/bin/env bash
# 本机单机仿真 ROS 2 环境（Isaac + bringup + rviz 各终端 source 此文件）。
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 必须用系统 Python 3.10（Humble）；conda base/env_isaaclab 的 3.12/3.14 会导致
# rclpy C 扩展 / numpy 全挂。先尽量退出 conda，再把 /usr/bin 提到 PATH 最前。
if [[ -n "${CONDA_DEFAULT_ENV:-}" || -n "${CONDA_PREFIX:-}" ]]; then
  if command -v conda >/dev/null 2>&1; then
    # shellcheck disable=SC1091
    source "$(conda info --base 2>/dev/null)/etc/profile.d/conda.sh" 2>/dev/null || true
    conda deactivate 2>/dev/null || true
    conda deactivate 2>/dev/null || true
  fi
  echo "[env] 已尝试 conda deactivate（Humble 需要 /usr/bin/python3=3.10）"
fi
export PATH="/usr/bin:/bin:${PATH}"
# 清掉 Isaac / conda 残留，避免 import 到错误的 numpy/rclpy
unset PYTHONPATH PYTHONHOME
# 若仍指向 conda python，强制提示
if ! /usr/bin/python3 -c 'import sys; assert sys.version_info[:2]==(3,10)' 2>/dev/null; then
  echo "[error] /usr/bin/python3 不是 3.10，无法搭配 ROS Humble" >&2
fi

source /opt/ros/humble/setup.bash
[[ -f "$SCRIPT_DIR/install/setup.bash" ]] && source "$SCRIPT_DIR/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=1
# 室外栈一次 launch 会起 15+ 进程，默认 participant 上限(~10)不够
export CYCLONEDDS_URI="file://$SCRIPT_DIR/cyclonedds_localhost.xml"
echo "[env] ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY CYCLONEDDS_URI=$CYCLONEDDS_URI"
echo "[env] python3=$(command -v python3) ($(/usr/bin/python3 -c 'import sys; print(sys.version.split()[0])'))"
