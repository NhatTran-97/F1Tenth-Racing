#!/bin/bash

echo "Starting f1tenth_stack"

# Thiết lập môi trường ROS2
source /opt/ros/foxy/setup.bash
source ~/f1_ws/install/setup.bash

# Chạy ROS2 launch trong nền
ros2 launch f1tenth_stack bringup_system.launch.py &
LAUNCH_PID=$!  # Lưu lại PID của tiến trình launch

# Danh sách các node cần kiểm tra
NODES=(
    "/ackermann_mux"
    "/ackermann_to_vesc_node"
    "/bno055"
    "/joy"
    "/vesc_driver_node"
    "/vesc_to_odom_node"
)

echo "Waiting for all required ROS2 nodes to start..."

# Kiểm tra khi tất cả các node xuất hiện
while true; do
    RUNNING_NODES=$(ros2 node list)
    ALL_NODES_UP=true

    for NODE in "${NODES[@]}"; do
        if ! echo "$RUNNING_NODES" | grep -q "$NODE"; then
            ALL_NODES_UP=false
            break
        fi
    done

    if [ "$ALL_NODES_UP" = true ]; then
        break
    fi

    sleep 1
done

echo "All required nodes are up. Starting RViz..."
RVIZ_CONFIG_PATH=~/f1_ws/src/scripts/visualize_f1tenth.rviz
ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG_PATH" &
RVIZ_PID=$!

# Chờ RViz khởi động (bằng cách kiểm tra node /rviz2 xuất hiện)
echo "Waiting for RViz to start..."
while true; do
    if ros2 node list | grep -q "/rviz"; then
        break
    fi
    sleep 1
done

echo "RViz started successfully. Launching provide_map.launch.py..."

# Chạy ROS2 launch provide_map.launch.py
ros2 launch f1tenth_stack provide_map.launch.py &

# # Giữ tiến trình chính để không bị thoát ngay
wait $LAUNCH_PID
