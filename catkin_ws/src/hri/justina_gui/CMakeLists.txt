cmake_minimum_required(VERSION 2.8.3)
project(justina_gui)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(justina_gui_node src/justina_gui_node.cpp)

target_link_libraries(justina_gui_node
  ${catkin_LIBRARIES}
)
