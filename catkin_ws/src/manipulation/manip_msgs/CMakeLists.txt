cmake_minimum_required(VERSION 2.8.3)
project(manip_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  InverseKinematicsFloatArray.srv
  InverseKinematicsPath.srv
  InverseKinematicsPose.srv
  DirectKinematics.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs   nav_msgs   std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

