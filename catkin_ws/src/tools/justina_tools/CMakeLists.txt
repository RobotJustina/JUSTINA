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
  roslib
  sensor_msgs
  std_msgs
  vision_msgs
  point_cloud_manager
  nav_msgs
  tf
  tf_conversions
  pcl_conversions
  sound_play
  knowledge_msgs
  std_srvs
)

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES justina_tools
  CATKIN_DEPENDS geometry_msgs hri_msgs manip_msgs navig_msgs roscpp rospy sensor_msgs std_msgs vision_msgs pcl_conversions knowledge_msgs std_srvs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(justina_tools
  src/JustinaHardware.cpp
  src/JustinaNavigation.cpp
  src/JustinaHRI.cpp
  src/JustinaTools.cpp
  src/JustinaManip.cpp
  src/JustinaVision.cpp
  src/JustinaTasks.cpp
  src/JustinaAudio.cpp
  src/JustinaKnowledge.cpp
  src/JustinaRepresentation.cpp
)

add_dependencies(justina_tools ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(justina_tools
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)
