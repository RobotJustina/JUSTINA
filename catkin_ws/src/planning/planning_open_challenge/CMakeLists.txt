cmake_minimum_required(VERSION 2.8.3)
project(planning_open_challenge)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  knowledge_msgs
)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

catkin_package(
	CATKIN_DEPENDS roscpp knowledge_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
