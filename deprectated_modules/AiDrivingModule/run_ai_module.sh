#!/bin/bash
# set -e -x

export ROS_IPV6=off

echo "using ros ipv6 ? $ROS_IPV6"
source /home/aidrivingmodule/docker_setup.sh
source /home/edgecar_msgs/catkin_ws/devel/setup.bash
source /home/aidrivingmodule/catkin_ws/devel/setup.bash --extend

roslaunch driving_module driving_module.launch veh:=$VEHICLE_NAME --wait