cmake_minimum_required(VERSION 2.8.3)
project(tutorials)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  manip_msgs
  nav_msgs
  hri_msgs
  navig_msgs
  point_cloud_manager
  roscpp
  rospy
  std_msgs
  tf
  vision_msgs
  bbros_bridge
  justina_tools
  std_srvs
  pcl_ros
  sensor_msgs
  cv_bridge
  image_transport
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES act_pln
#  CATKIN_DEPENDS geometry_msgs manip_msgs nav_msgs navig_msgs point_cloud_manager roscpp rospy std_msgs tf vision_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(act_pln
#   src/${PROJECT_NAME}/act_pln.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(act_pln ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
#add_executable(act_pln_node
#  src/act_pln_node.cpp
#)
#add_executable(navigation_test
#  src/navigation_test.cpp
#)

add_executable(nav_test
  src/nav_test.cpp
)

add_dependencies(nav_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(nav_test
  ${catkin_LIBRARIES}
)

