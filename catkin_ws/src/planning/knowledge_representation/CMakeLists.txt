cmake_minimum_required(VERSION 2.8.3)
project(knowledge_representation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  vision_msgs
  sensor_msgs
  knowledge_msgs
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
	CATKIN_DEPENDS roscpp std_msgs geometry_msgs vision_msgs sensor_msgs knowledge_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(kdb_node
	src/kdb_node.cpp
)

add_dependencies(kdb_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(kdb_node
  ${catkin_LIBRARIES}
)
