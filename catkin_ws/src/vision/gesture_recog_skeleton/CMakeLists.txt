cmake_minimum_required(VERSION 2.8.3)
project(gesture_recog_skeleton)

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
)

add_definitions(-std=c++11)

find_package(orocos_kdl REQUIRED)
find_package(OpenCV REQUIRED)
set(CMAKE_PREFIX_PATH "/usr/local/")
set(OpenCV_INCLUDE_DIRS "/usr/local/include")

# Find OpenNI
find_package(PkgConfig)
pkg_check_modules(OpenNI REQUIRED libopenni)

catkin_package(
)

include_directories(
	include ${catkin_INCLUDE_DIRS}
	${OpenNI_INCLUDEDIR}
	${orocos_kdl_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

add_executable(gesture_recog_node src/gesture_recog_node.cpp)

##add_dependencies(gesture_recog_node geometry_msgs_gencpp vision_msgs_gencpp)
add_dependencies(gesture_recog_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(gesture_recog_node
	#${OpenCV_LIBS}
	#${PCL_LIBRARIES}
	${OpenNI_LIBRARIES}
	${orocos_kdl_LIBRARIES}
	${catkin_LIBRARIES}
)

install(TARGETS gesture_recog_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

