cmake_minimum_required(VERSION 2.8.3)
project(manip_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  message_generation
)

## Generate services in the 'srv' folder
file(GLOB SERVICE_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/srv ${CMAKE_CURRENT_SOURCE_DIR}/srv/*.srv)
add_service_files(FILES ${SERVICE_FILES})

generate_messages(
  DEPENDENCIES
  geometry_msgs   nav_msgs   std_msgs
)

catkin_package(
)
