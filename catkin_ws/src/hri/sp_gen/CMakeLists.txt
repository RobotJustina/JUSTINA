cmake_minimum_required(VERSION 2.8.3)
project(sp_gen)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sound_play
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(sp_gen_node src/sp_gen_node.cpp)

target_link_libraries(sp_gen_node
   ${catkin_LIBRARIES}
)
