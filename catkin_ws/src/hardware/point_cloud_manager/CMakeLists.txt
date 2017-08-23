cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_manager)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  message_generation
  tf_conversions
  pcl_ros
)

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)
set(CMAKE_PREFIX_PATH "/usr/local/")
set(OpenCV_INCLUDE_DIRS "/usr/local/include")

add_service_files(
   FILES
   GetRgbd.srv
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
  ${PCL_INCLUDE_DIRS}
)

add_executable(point_cloud_man_node 
  src/point_cloud_man_node.cpp
  src/PcManNode.cpp
)

add_dependencies(point_cloud_man_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(point_cloud_man_node
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)
