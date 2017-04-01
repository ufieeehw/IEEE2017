cmake_minimum_required(VERSION 2.8.3)
project(ieee2017_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  message_runtime
  std_msgs
  geometry_msgs
  sensor_msgs
)

add_message_files(
  FILES
  Mecanum.msg
  StartNavigation.msg
  ShooterControl.msg
)

add_service_files(
   FILES
   NavWaypoint.srv
   StopMecanum.srv
   StopController.srv
   LidarSelector.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  LIBRARIES ieee2017_msgs
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
