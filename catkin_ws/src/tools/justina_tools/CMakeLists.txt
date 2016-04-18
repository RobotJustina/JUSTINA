cmake_minimum_required(VERSION 2.8.3)
project(justina_tools)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  hri_msgs
  manip_msgs
  navig_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  vision_msgs
  point_cloud_manager
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES justina_tools
  CATKIN_DEPENDS geometry_msgs hri_msgs manip_msgs navig_msgs roscpp rospy sensor_msgs std_msgs vision_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(justina_tools
  src/JustinaHardware.cpp
  src/JustinaNavigation.cpp
)

target_link_libraries(justina_tools
  ${catkin_LIBRARIES}
)
