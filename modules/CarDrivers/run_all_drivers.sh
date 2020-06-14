#!/bin/bash
#set -e -x
export ROS_IPV6=off

echo "using ROS_IPV6 ? $ROS_IPV6"
source /home/cardrivers/docker_setup.sh
source /home/edgecar_msgs/catkin_ws/devel/setup.bash
source /home/cardrivers/catkin_ws/devel/setup.bash --extend
echo "starting roscore "
roscore &
echo "Starting all drivers"
roslaunch cardrivers all_drivers.launch veh:=$VEHICLE_NAME --wait
echo "Started All Drivers"