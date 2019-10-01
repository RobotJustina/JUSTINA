cmake_minimum_required(VERSION 2.8.3)
project(color_reco)

find_package(catkin REQUIRED COMPONENTS
  	roscpp
	rospy
	geometry_msgs
	justina_tools
	roslib
	sensor_msgs
	std_msgs
	tf
	vision_msgs
	cv_bridge
	visualization_msgs
)

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)
set(CMAKE_PREFIX_PATH "/usr/local/")
set(OpenCV_INCLUDE_DIRS "/usr/local/include")

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(color_reco_node src/color_reco_node.cpp)

add_dependencies(color_reco_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(color_reco_node
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)
