cmake_minimum_required(VERSION 2.8.3)
project(camera_driver)

find_package(catkin REQUIRED
    rospy
    std_msgs
    edgecar_msgs)

catkin_python_setup()


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES camera_driver
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
