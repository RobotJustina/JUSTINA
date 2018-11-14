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
	nav_msgs
 	visualization_msgs
	interactive_markers
	tf
        knowledge_msgs
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  	DEPENDENCIES
  	std_msgs
	geometry_msgs
	nav_msgs
        knowledge_msgs
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

add_executable(pred_ques_node
  src/pred_ques_node.cpp
)

add_dependencies(pred_ques_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(pred_ques_node
  ${catkin_LIBRARIES}
)


add_executable(pred_poses_mani_node
  src/pred_poses_mani.cpp
)

add_dependencies(pred_poses_mani_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(pred_poses_mani_node
  ${catkin_LIBRARIES}
)
