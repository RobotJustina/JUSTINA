cmake_minimum_required(VERSION 2.8.3)
project(hardware_tools)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	#dynamixel_sdk
	cmake_modules
	tf
	)
catkin_python_setup()
find_package(dynamixel_sdk REQUIRED)
find_package(serial REQUIRED)

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

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

catkin_package(
	INCLUDE_DIRS include/
	LIBRARIES DynamixelManager JrkManager RoboClaw
	CATKIN_DEPENDS roscpp rospy dynamixel_sdk serial
	#  DEPENDS system_lib
	)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	)


add_library(DynamixelManager src/DynamixelManager.cpp)
target_link_libraries(DynamixelManager
	${catkin_LIBRARIES}
	)
add_dependencies(DynamixelManager hardware_tools_gencpp)

add_library(JrkManager src/JrkManager.cpp)
target_link_libraries(JrkManager
	${catkin_LIBRARIES}
	)
add_dependencies(JrkManager hardware_tools_gencpp)

add_library(RoboClaw src/roboclaw.cpp)
target_link_libraries(RoboClaw
	${catkin_LIBRARIES}
	)
add_dependencies(RoboClaw hardware_tools_gencpp)
