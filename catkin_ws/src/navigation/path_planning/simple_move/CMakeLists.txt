cmake_minimum_required(VERSION 2.8.3)
project(simple_move)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  navig_msgs
  roscpp
  rospy
  std_msgs
  tf
)

# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(simple_move_node src/simple_move_node.cpp)

add_dependencies(simple_move_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(simple_move_node
  ${catkin_LIBRARIES}
)
