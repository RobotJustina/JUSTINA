cmake_minimum_required(VERSION 2.8.3)
project(skeleton_finder)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	geometry_msgs
	roslib
	sensor_msgs
	std_msgs
	tf
	vision_msgs
)

find_package(orocos_kdl REQUIRED)

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

add_executable(skeleton_finder_node src/skeleton_finder_node.cpp)

##add_dependencies(skeleton_finder_node geometry_msgs_gencpp vision_msgs_gencpp)
add_dependencies(skeleton_finder_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(skeleton_finder_node 
	#${OpenCV_LIBS}
	#${PCL_LIBRARIES}
	${OpenNI_LIBRARIES}
	${orocos_kdl_LIBRARIES}
	${catkin_LIBRARIES}
)

install(TARGETS skeleton_finder_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES openni_tracker.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
