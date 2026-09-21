#!/usr/bin/env bash
# 本机单机仿真 ROS 2 环境（Isaac + bringup + rviz 各终端 source 此文件）。
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
[[ -f "$SCRIPT_DIR/install/setup.bash" ]] && source "$SCRIPT_DIR/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=7
export ROS_LOCALHOST_ONLY=1
# 室外栈一次 launch 会起 15+ 进程，默认 participant 上限(~10)不够
export CYCLONEDDS_URI="file://$SCRIPT_DIR/cyclonedds_localhost.xml"
echo "[env] ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY CYCLONEDDS_URI=$CYCLONEDDS_URI"
