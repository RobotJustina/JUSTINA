cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_manager)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  message_generation
)

find_package(OpenCV REQUIRED)

add_service_files(
   FILES
   get_rgbd.srv
)

generate_messages(
   DEPENDENCIES
   geometry_msgs   sensor_msgs   std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES point_cloud_manager
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(point_cloud_man_node 
  src/point_cloud_man_node.cpp
  src/PcManNode.cpp
)

target_link_libraries(point_cloud_man_node
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
