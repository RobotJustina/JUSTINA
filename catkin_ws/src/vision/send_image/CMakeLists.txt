cmake_minimum_required(VERSION 2.8.3)
project(send_image)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  image_transport
  cv_bridge
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)
set(CMAKE_PREFIX_PATH "/usr/local/")
set(OpenCV_INCLUDE_DIRS "/usr/local/include")


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(
send_image_node
  src/send_image_node.cpp
)

add_dependencies(
  send_image_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  send_image_node
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
