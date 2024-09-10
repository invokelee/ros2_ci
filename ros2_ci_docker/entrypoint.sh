#! /bin/bash
echo "[$(date +'%F %T')] Starting ROS2 Simulation of TortoiseBot..."
source /tortoise_ws/install/setup.bash && 
ros2 launch tortoisebot_bringup bringup.launch.py use_sim_time:=True &
sleep 20
# update test
source /tortoise_ws/install/setup.bash &&
colcon test --packages-select tortoisebot_waypoints --event-handler=console_direct+ && 
colcon test-result --all
sleep 10
echo "ros2- Tortoisebot Waypoints Test complete !"
