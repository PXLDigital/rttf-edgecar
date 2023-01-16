#!/bin/bash
# set -e -x

export ROS_IPV6=off

cd /home/edgecar_msgs/catkin_ws \
&& catkin_make \
&& source /home/edgecar_msgs/catkin_ws/devel/setup.bash \
&& cd /home/aidrivingmodule/catkin_ws \
&& catkin_make

echo "using ros ipv6 ? $ROS_IPV6"
source /home/aidrivingmodule/docker_setup.sh
source /home/edgecar_msgs/catkin_ws/devel/setup.bash
source /home/aidrivingmodule/catkin_ws/devel/setup.bash --extend

#roslaunch detector_node detector_node.launch veh:=$VEHICLE_NAME --wait
#roslaunch driving_module driving_module.launch veh:=$VEHICLE_NAME --wait
roslaunch autopilot autopilot.launch veh:=$VEHICLE_NAME --wait