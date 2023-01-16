#!/bin/bash

export ROS_IPV6=off
source /home/cardrivers/docker_setup.sh
source /home/edgecar_msgs/catkin_ws/devel/setup.bash
source /home/cardrivers/catkin_ws/devel/setup.bash --extend

rostopic echo "/rosout"
