cmake_minimum_required(VERSION 2.8.3)
project(wheels_driver)

find_package(catkin REQUIRED COMPONENTS
  adafruit_drivers
  edgecar_msgs
  rospy
)
catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
