#!/bin/bash

export ROS_IPV6=off
source /home/cardrivers/docker_setup.sh
source /home/edgecar_msgs/catkin_ws/devel/setup.bash
source /home/cardrivers/catkin_ws/devel/setup.bash --extend

rosbag play /home/cardrivers/Data/bags/trim-2020-09-18-11-43-30.bag -l
# rosbag info /home/cardrivers/Data/bags/2020-09-18-11-43-30.bag
# rosbag filter /home/cardrivers/Data/bags/2020-09-18-11-43-30.bag /home/cardrivers/Data/bags/trim-2020-09-18-11-43-30.bag "topic == '/master/camera_node/image/compressed'"