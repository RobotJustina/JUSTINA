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

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

set(OPENPOSE_DIR "$ENV{OPENPOSE_HOME}")

if(OPENPOSE_DIR)               # <--- Use 'BOOST_DIR', not 'DEFINED ${BOOST_DIR}'
	find_package(CUDA REQUIRED)
	find_package(gflags REQUIRED)
	message(STATUS "OPENPOSE_HOME at ${OPENPOSE_DIR}")
	## Generate messages in the 'msg' folder
	# add_message_files(
	#   FILES
	#   Message1.msg
	#   Message2.msg
	# )

	## Generate services in the 'srv' folder
	# add_service_files(
	#   FILES
	#   Service1.srv
	#   Service2.srv
	# )

	## Generate added messages and services with any dependencies listed here
	# generate_messages(
	#   DEPENDENCIES
	#   std_msgs
	# )

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
		${OPENPOSE_DIR}/include
		${OPENPOSE_DIR}/3rdparty/caffe/include
		${OPENPOSE_DIR}/3rdparty/caffe/.build_release/src
		)

	add_library(OpenPose src/OpenPose.cpp)
	target_link_libraries(OpenPose 
		${catkin_LIBRARIES} 
		${Eigen_LIBRARIES}
		${OPENPOSE_DIR}/build/lib/libopenpose.so
		${OPENPOSE_DIR}/3rdparty/caffe/.build_release/lib/libcaffe.so
		gflags
		)

	add_executable(open_pose_node src/open_pose_node.cpp)
	target_link_libraries(open_pose_node ${catkin_LIBRARIES} OpenPose)
else()
	message(WARNING "Undefined OPENPOSE_HOME env var, this package will not be compiled.")
endif()

