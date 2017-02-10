cmake_minimum_required(VERSION 2.8.3)
project(knowledge)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	geometry_msgs
	message_generation
 	visualization_msgs
	interactive_markers
	tf
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  KnownLocations.srv
)

add_message_files(
	FILES
	MapKnownLocation.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  	DEPENDENCIES
  	std_msgs
	geometry_msgs
	knowledge
)

catkin_package(
)
 
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ltm_node 
  src/ltm_node.cpp  
)

add_dependencies(ltm_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ltm_node
  ${catkin_LIBRARIES}
)
