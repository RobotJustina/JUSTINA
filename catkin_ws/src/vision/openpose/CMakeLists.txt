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

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

set(OPENPOSE_DIR "$ENV{OPENPOSE_HOME}")

if(OPENPOSE_DIR)               # <--- Use 'BOOST_DIR', not 'DEFINED ${BOOST_DIR}'
	find_package(Eigen3 REQUIRED)
	find_package(CUDA REQUIRED)
	#find_package(GFlags REQUIRED)
	#find_package(GLog REQUIRED)
	find_package(OpenPose REQUIRED)
	find_package(OpenCV REQUIRED)
	find_package(gflags REQUIRED)
	message(STATUS "OPENPOSE_HOME at ${OPENPOSE_DIR}")

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
		${CUDA_INCLUDE_DIRS}
		${Eigen_INCLUDE_DIRS}
		${OpenPose_INCLUDE_DIRS}
		${GFLAGS_INCLUDE_DIR}
		${GLOG_INCLUDE_DIR}
		${OpenCV_INCLUDE_DIRS}
		)

	add_library(OpenPose src/OpenPose.cpp)
	target_link_libraries(OpenPose 
		${catkin_LIBRARIES}
		${Eigen_LIBRARIES}
		${OpenPose_LIBS}
		${GFLAGS_LIBRARY}
		${GLOG_LIBRARY}
		${OpenCV_LIBS}
		gflags
		)

	add_executable(open_pose_node src/open_pose_node.cpp)
	target_link_libraries(open_pose_node ${catkin_LIBRARIES} OpenPose)
else()
	message(WARNING "Undefined OPENPOSE_HOME env var, this package will not be compiled.")
endif()

