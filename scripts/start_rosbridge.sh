#!/bin/bash

export ROS_IPV6=off

cd /home/edgecar_msgs/catkin_ws && catkin_make

source /home/cardrivers/docker_setup.sh
source /home/edgecar_msgs/catkin_ws/devel/setup.bash

cd /home/cardrivers/catkin_ws && catkin_make

source /home/edgecar_msgs/catkin_ws/devel/setup.bash
source /home/cardrivers/catkin_ws/devel/setup.bash --extend

/home/cardrivers/run_all_drivers.sh &

export ROS_IPV6=off
source /home/cardrivers/docker_setup.sh
source /home/edgecar_msgs/catkin_ws/devel/setup.bash
source /home/cardrivers/catkin_ws/devel/setup.bash --extend

roslaunch rosbridge_server rosbridge_websocket.launch --wait