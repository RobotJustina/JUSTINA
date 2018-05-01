cmake_minimum_required(VERSION 2.8.3)
project(roi_tracker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  vision_msgs
  justina_tools
  roslib
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(dlib REQUIRED)
set(CMAKE_PREFIX_PATH "/usr/local/")
set(OpenCV_INCLUDE_DIRS "/usr/local/include")
include_directories(/opt/OpenBLAS/include)
link_directories(/opt/OpenBLAS/lib)

add_definitions(-std=c++11)

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
  roi_tracker_node
  src/roi_tracker_node.cpp
  src/RoiTracker.cpp
  src/LMS.cpp
)

add_dependencies(
  roi_tracker_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  roi_tracker_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  dlib::dlib
  /usr/local/lib/libopencv_tracking.so.3.3.1
)
