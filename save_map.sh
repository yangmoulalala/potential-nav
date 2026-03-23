#!/bin/bash
set -e

if [ $# -ne 1 ]; then
    echo "Usage: $0 <WORLD_NAME>"
    exit 1
fi

WORLD_NAME=$1
CONTAINER="potential-nav"

# 读取宿主机的 ROS_DOMAIN_ID
if [ -z "$ROS_DOMAIN_ID" ]; then
    echo "Warning: ROS_DOMAIN_ID is not set. Using 0."
    ROS_DOMAIN_ID=0
fi

echo ">>> Phase 1: saving map while container is running..."

docker exec \
    -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
    $CONTAINER /bin/bash -c "

source /opt/ros/humble/setup.bash
source /root/ros_ws/install/setup.bash
cd /root/ros_ws

echo '>>> Running map_saver_cli...'
ros2 run nav2_map_server map_saver_cli -f $WORLD_NAME --ros-args -r __ns:=/nav

MAP_PATH='/root/ros_ws/src/potential_nav/pb2025_nav_bringup/map/reality'
mkdir -p \$MAP_PATH

mv '${WORLD_NAME}.pgm' \"\$MAP_PATH/${WORLD_NAME}.pgm\"
mv '${WORLD_NAME}.yaml' \"\$MAP_PATH/${WORLD_NAME}.yaml\"
sed -i \"s|image: .*|image: ${WORLD_NAME}.pgm|\" \"\$MAP_PATH/${WORLD_NAME}.yaml\"

echo '>>> Map saved.'
"

echo ""
echo "==================================================================="
echo "📌 请停止容器内的建图程序（point_lio / SLAM）！"
echo "📌 当你停止后容器会自动退出。然后按 Enter 继续保存 scans.pcd ..."
echo "==================================================================="
read -p "[按 Enter 键继续]"

echo ">>> Phase 2: Saving PCD from host (container already stopped)"

HOST_PCD="./src/potential_nav/point_lio/PCD/scans.pcd"
HOST_TARGET="./src/potential_nav/pb2025_nav_bringup/pcd/reality/${WORLD_NAME}.pcd"

mkdir -p ./src/potential_nav/pb2025_nav_bringup/pcd/reality

if [ ! -f "$HOST_PCD" ]; then
    echo "❌ Error: scans.pcd not found at $HOST_PCD"
    echo "请确认建图程序已经完全停止（point_lio 已写出 scans.pcd）"
    exit 1
fi

cp "$HOST_PCD" "$HOST_TARGET"

echo ">>> PCD saved to: $HOST_TARGET"
echo ">>> All done!"