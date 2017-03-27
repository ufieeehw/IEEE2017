cmake_minimum_required(VERSION 2.8.3)
project(ieee2017_command)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
