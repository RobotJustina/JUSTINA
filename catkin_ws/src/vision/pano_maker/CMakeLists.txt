cmake_minimum_required(VERSION 2.8.3)
project(pano_maker)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy 
  sensor_msgs
  std_msgs
  std_srvs
  vision_msgs
  justina_tools
  roslib
  cv_bridge
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
set(CMAKE_PREFIX_PATH "/usr/local/")
set(OpenCV_INCLUDE_DIRS "/usr/local/include")

generate_messages(
  DEPENDENCIES
  geometry_msgs sensor_msgs std_msgs vision_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(
  pano_maker_node 
  src/pano_maker_node.cpp
)

add_dependencies(
  pano_maker_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  pano_maker_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
