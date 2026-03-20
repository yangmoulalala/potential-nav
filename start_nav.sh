#!/bin/bash

cd /root/ros_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build --symlink-install

USE_RVIZ=False

# 核心判断：DISPLAY存在 + 能真正访问X服务
if [ -n "$DISPLAY" ]; then
    if timeout 0.5 xdpyinfo >/dev/null 2>&1; then
        USE_RVIZ=True
    else
        echo "[WARN] DISPLAY=$DISPLAY but X server unreachable"
    fi
else
    echo "[INFO] DISPLAY not set"
fi

echo "[INFO] use_rviz = $USE_RVIZ"

exec ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
    slam:=True \
    use_robot_state_pub:=True \
    use_rviz:=$USE_RVIZ \
    use_respawn:=True \
    "$@"