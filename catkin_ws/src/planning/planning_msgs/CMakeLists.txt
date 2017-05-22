cmake_minimum_required(VERSION 2.8.3)
project(planning_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  vision_msgs
  sensor_msgs
  geometry_msgs
)

## Generate messages in the 'msg' folder
file(GLOB MESSAGE_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/msg ${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg)
add_message_files(FILES ${MESSAGE_FILES})

## Generate services in the 'srv' folder
file(GLOB SERVICE_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/srv ${CMAKE_CURRENT_SOURCE_DIR}/srv/*.srv)
add_service_files(FILES ${SERVICE_FILES})

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   vision_msgs
   geometry_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES planning_msgs
#  CATKIN_DEPENDS roscpp rospy std_msgs
#CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)
