#!/bin/bash
# Single script to launch the OpenAMR with Gazebo, Nav2 and ROS 2 Controllers

# Stop only what this script started (the launch process group), instead of
# pattern-killing every process whose command line contains "ros2" or "gz".
cleanup() {
    echo "Cleaning up..."
    if [ -n "$LAUNCH_PID" ]; then
        kill -INT -- "-$LAUNCH_PID" 2>/dev/null
        sleep 5.0
        kill -KILL -- "-$LAUNCH_PID" 2>/dev/null
    fi
}

# Set up cleanup trap
trap 'cleanup' SIGINT SIGTERM

# Check if SLAM argument is provided
if [ "$1" = "slam" ]; then
    SLAM_ARG="slam:=True"
else
    SLAM_ARG="slam:=False"
fi

# For cafe.world -> z:=0.20
# For house.world -> z:=0.05
# To change Gazebo camera pose: gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 2000 --req "pose: {position: {x: 0.0, y: -2.0, z: 2.0} orientation: {x: -0.2706, y: 0.2706, z: 0.6533, w: 0.6533}}"

MAP_FILE="$(ros2 pkg prefix --share open_amr_navigation)/maps/cafe_world_map.yaml"

echo "Launching Gazebo simulation with Nav2..."
# setsid -> own process group, so cleanup can signal the whole launch tree
setsid ros2 launch open_amr_bringup open_amr_navigation.launch.py \
    enable_odom_tf:=false \
    headless:=False \
    load_controllers:=true \
    world_file:=cafe.world \
    use_rviz:=true \
    use_robot_state_pub:=true \
    use_sim_time:=true \
    x:=0.0 \
    y:=0.0 \
    z:=0.20 \
    roll:=0.0 \
    pitch:=0.0 \
    yaw:=0.0 \
    "$SLAM_ARG" \
    map:="$MAP_FILE" &
LAUNCH_PID=$!

echo "Waiting 25 seconds for simulation to initialize..."
sleep 25

echo "Adjusting camera position..."
gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 2000 --req "pose: {position: {x: 0.0, y: -2.0, z: 2.0} orientation: {x: -0.2706, y: 0.2706, z: 0.6533, w: 0.6533}}"

# Keep the script running until Ctrl+C
wait
