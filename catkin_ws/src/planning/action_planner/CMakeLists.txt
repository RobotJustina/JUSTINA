cmake_minimum_required(VERSION 2.8.3)
#include (${CMAKE_CURRENT_SOURCE_DIR}/dummy/DepCheck.cmake OPTIONAL)

project(action_planner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	message_generation

	# Package Dependencies
	arms
	bbros_bridge
	head
	language_understanding
	mvn_pln
	roah_rsbb_comm_ros
    	robot_service_manager
	tf_conversions
        simple_task_planner
    	planning_msgs
)

## System dependencies are found with CMake's conventions

### First, we define the BOOST library
### It will be used in future commits to enhance the project
set(Boost_USE_STATIC_LIBS   ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME ON)
find_package( Boost 1.40 COMPONENTS program_options system filesystem regex thread chrono date_time REQUIRED )

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# file(GLOB MESSAGE_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/msg ${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg)
# add_message_files(FILES ${MESSAGE_FILES})

## Generate services in the 'srv' folder
# file(GLOB SERVICE_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/srv ${CMAKE_CURRENT_SOURCE_DIR}/srv/*.srv)
# add_service_files(FILES ${SERVICE_FILES})

## Generate actions in the 'action' folder
# file(GLOB ACTION_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/action ${CMAKE_CURRENT_SOURCE_DIR}/action/*.action)
# add_action_files(FILES ${ACTION_FILES})

## Generate added messages and services with any dependencies listed here
# generate_messages(
#	DEPENDENCIES
#	std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS include
#  LIBRARIES action_planner
#  CATKIN_DEPENDS roscpp rospy
  DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

file(GLOB_RECURSE headers "include/action_planner/*.h*")
file(GLOB_RECURSE sources "include/action_planner/*.c*" "src/*.c*")
### Note: All cpp files should be HERE
# file(GLOB_RECURSE sources "src/*.c*")

## Declare a C++ library
# add_library(action_planner
#   src/${PROJECT_NAME}/action_planner.cpp
# )

## Declare a C++ executable
add_executable( action_planner_complete ${sources} ${headers} )

## Specify libraries to link a library or executable target against
target_link_libraries(
	action_planner_complete
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(
	action_planner_complete
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
	action_planner_generate_messages_cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
	action_planner_complete
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
)
