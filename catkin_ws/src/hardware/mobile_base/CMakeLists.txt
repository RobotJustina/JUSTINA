cmake_minimum_required(VERSION 2.8.3)
project(mobile_base)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  hardware_tools #In this package is located the Roboclaw.py
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
# add_executable(mobile_base_node src/mobile_base_node.cpp)

#add_dependencies(mobile_base_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(mobile_base_node
#   ${catkin_LIBRARIES}
# )
