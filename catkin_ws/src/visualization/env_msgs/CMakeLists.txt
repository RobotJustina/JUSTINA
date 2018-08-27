cmake_minimum_required(VERSION 2.8.3)
project(env_msgs)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	geometry_msgs
	message_generation
)

add_message_files(
   FILES
   ObjectViz.msg
)

add_service_files(
  FILES
  AddUpdateObjectViz.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs geometry_msgs env_msgs
)

catkin_package(
)

include_directories(
	${catkin_INCLUDE_DIRS}
)
