cmake_minimum_required(VERSION 2.8.3)
project(supervisor)

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
  justina_tools
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

add_executable(supervisor_node
	src/supervisor_node.cpp
)

add_dependencies(supervisor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(supervisor_node
  ${catkin_LIBRARIES}
)

add_executable(sensor_report_node
	src/report_sensors.cpp
)

add_dependencies(sensor_report_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(sensor_report_node
  ${catkin_LIBRARIES}
)
