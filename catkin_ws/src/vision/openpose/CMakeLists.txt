cmake_minimum_required(VERSION 2.8.3)
project(openpose)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/Modules/")
message(STATUS "CMAKE Module Path at ${CMAKE_MODULE_PATH}")

add_definitions(-std=c++11)
add_definitions(-DUSE_CAFFE)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	sensor_msgs
	geometry_msgs
	visualization_msgs
	justina_tools
	cmake_modules
	tf
	tf_conversions
	vision_msgs
	)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gflags REQUIRED)
catkin_package(
	#  INCLUDE_DIRS include
	#  LIBRARIES openpose
	#  CATKIN_DEPENDS roscpp rospy std_msgs
	#  DEPENDS system_lib
	# DEPENDS Eigen3
	)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	)

add_executable(open_pose_node src/open_pose_node.cpp)
target_link_libraries(open_pose_node ${catkin_LIBRARIES} gflags)

