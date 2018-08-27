cmake_minimum_required(VERSION 2.8.3)
project(env_des)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
  env_msgs
)

SET(CMAKE_CXX_FLAGS "-frounding-math -std=c++11")

find_package(yaml-cpp REQUIRED)
#find_package(haru REQUIRED)

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES environment_description
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)


add_executable(environment_description_node 
  src/environment_description_node.cpp
)

add_dependencies(environment_description_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(environment_description_node
  ${catkin_LIBRARIES}
  yaml-cpp
  ParserEnvironment
)

add_library(ParserEnvironment src/ParserEnvironment.cpp)
target_link_libraries(ParserEnvironment
  	yaml-cpp
	${catkin_LIBRARIES}
	)
add_dependencies(ParserEnvironment environment_description_gencpp)
