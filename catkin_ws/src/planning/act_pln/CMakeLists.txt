cmake_minimum_required(VERSION 2.8.3)
project(act_pln)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  manip_msgs
  nav_msgs
  hri_msgs
  navig_msgs
  point_cloud_manager
  roscpp
  rospy
  std_msgs
  tf
  vision_msgs
  bbros_bridge
  justina_tools
  std_srvs
  pcl_ros
  sensor_msgs
  cv_bridge
  image_transport
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
#   geometry_msgs#   manip_msgs#   nav_msgs#   navig_msgs#   std_msgs#   vision_msgs
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
#  INCLUDE_DIRS include
#  LIBRARIES act_pln
#  CATKIN_DEPENDS geometry_msgs manip_msgs nav_msgs navig_msgs point_cloud_manager roscpp rospy std_msgs tf vision_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(act_pln
#   src/${PROJECT_NAME}/act_pln.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(act_pln ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
#add_executable(act_pln_node
#  src/act_pln_node.cpp
#)
#add_executable(navigation_test
#  src/navigation_test.cpp
#)

add_executable(monitor
  src/monitor.cpp
)

add_executable(follow_me_test
  src/follow_me_test.cpp
)

add_executable(helpMeCarry_test
  src/helpMeCarry_test.cpp
)

#add_executable(speech_recognition_test
#  src/speech_recognition_test.cpp
#)

#add_executable(manipulation_test
#  src/manipulation_test.cpp
#)

#add_executable(vision_states
#  src/vision_states.cpp
#)

#add_executable(manipulation_test_linea
#  src/manipulation_test_linea.cpp
#)

#add_executable(robot_inspection_test
#  src/robot_inspection_test.cpp
#)

add_executable(gpsr_test
  src/gpsr_test.cpp
)
#add_executable(person_recognition_test
#  src/person_recognition.cpp
#)
add_executable(speechandperson_test
  src/speechandperson_test.cpp
)
add_executable(open_challenge_test
  src/open_challenge.cpp
)
#add_executable(the_real_open_challenge_test
#  src/the_real_open_challenge.cpp
#)
#add_executable(just_for_test
#  src/just_for_test.cpp
#)

#add_executable(restaurant_test
#  src/restaurant.cpp
#)

#add_executable(restaurant_nagoya
#  src/restaurant_nagoya.cpp
#)

add_executable(restaurant_canada
  src/restaurant_canada.cpp
)

add_executable(eegpsr_test
  src/eegpsr_test.cpp
)

#add_executable(storing_groseries_test
#  src/storing_groseries_test.cpp
#)

#add_executable(storing_groseries_test_2
#  src/storing_groseries_test_2.cpp
#)

add_executable(storing_groseries_test
  src/storing_groseries_test_2018.cpp
)

#add_executable(tv_azteca_test
#  src/tv_azteca_test.cpp
#)

add_executable(robot_inspection_test_2
	src/robot_inspection_test_2.cpp
)

add_executable(set_a_table_test
	src/set_a_table_and_clean_it_up.cpp
)

add_executable(open_challenge_nagoya
	src/open_challenge_nagoya.cpp
)

add_executable(hybrid_follow_test
	src/hybrid_follow_test.cpp
)

add_executable(set_table
  src/suat.cpp
)

add_executable(async_audio_test
	src/async_audio_test.cpp
)

add_executable(rec_obj_votation_test
	src/rec_obj_votation_test.cpp
)

add_executable(cubes_challenge
	src/cubes_challenge.cpp
)

add_executable(p_and_g_test
	src/p_and_g_test.cpp
)

add_executable(final_cubes_challange
	src/final_cubes_challange.cpp
)

#add_executable(visit_my_home_test
#	src/visit_my_home_test.cpp
#)

#add_executable(welcoming_visitor_test
#	src/welcoming_visitor_test.cpp
#)

#add_executable(catering_comfort_test
#	src/catering_comfort_test.cpp
#)

#add_executable(getting_to_know_my_home
#	src/getting_to_know_my_home.cpp
#)

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(act_pln_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(navigation_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(monitor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(follow_me_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(helpMeCarry_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(speech_recognition_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(robot_inspection_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(gpsr_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(person_recognition_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(open_challenge_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(the_real_open_challenge_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(just_for_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(eegpsr_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(storing_groseries_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(storing_groseries_test_2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(storing_groseries_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(speechandperson_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(tv_azteca_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(robot_inspection_test_2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(set_a_table_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(restaurant_nagoya ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(restaurant_canada ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(open_challenge_nagoya ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(hybrid_follow_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(set_table ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(async_audio_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(rec_obj_votation_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(cubes_challenge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(p_and_g_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(final_cubes_challange ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(visit_my_home_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(welcoming_visitor_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(catering_comfort_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(getting_to_know_my_home ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#target_link_libraries(act_pln_node
#  ${catkin_LIBRARIES}
#)
#target_link_libraries(navigation_test
#  ${catkin_LIBRARIES}
#)

target_link_libraries(monitor
  ${catkin_LIBRARIES}
)

target_link_libraries(follow_me_test
  ${catkin_LIBRARIES}
)

target_link_libraries(helpMeCarry_test
  ${catkin_LIBRARIES}
)

#target_link_libraries(restaurant_test
#  ${catkin_LIBRARIES}
#)

#target_link_libraries(restaurant_nagoya
#  ${catkin_LIBRARIES}
#)

target_link_libraries(restaurant_canada
	${catkin_LIBRARIES}
)

#target_link_libraries(manipulation_test
#  ${catkin_LIBRARIES}
#)

#target_link_libraries(vision_states
#  ${catkin_LIBRARIES}
#)

#target_link_libraries(manipulation_test_linea
#  ${catkin_LIBRARIES}
#)

#target_link_libraries(speech_recognition_test
#  ${catkin_LIBRARIES}
#)

#target_link_libraries(robot_inspection_test
#  ${catkin_LIBRARIES}
#)

target_link_libraries(gpsr_test
  ${catkin_LIBRARIES}
)

#target_link_libraries(person_recognition_test
#  ${catkin_LIBRARIES}
#)

target_link_libraries(open_challenge_test
  ${catkin_LIBRARIES}
)

#target_link_libraries(the_real_open_challenge_test
#  ${catkin_LIBRARIES}
#)

#target_link_libraries(just_for_test
#  ${catkin_LIBRARIES}
#)

target_link_libraries(eegpsr_test
  ${catkin_LIBRARIES}
)

#target_link_libraries(storing_groseries_test
#  ${catkin_LIBRARIES}
#)

target_link_libraries(storing_groseries_test
  ${catkin_LIBRARIES}
)

target_link_libraries(speechandperson_test
  ${catkin_LIBRARIES}
)

#target_link_libraries(tv_azteca_test
#  ${catkin_LIBRARIES}
#)

target_link_libraries(robot_inspection_test_2
  ${catkin_LIBRARIES}
)

target_link_libraries(set_a_table_test
  ${catkin_LIBRARIES}
)

target_link_libraries(open_challenge_nagoya
  ${catkin_LIBRARIES}
)

target_link_libraries(hybrid_follow_test
  ${catkin_LIBRARIES}
)

target_link_libraries(set_table
  ${catkin_LIBRARIES}
)

target_link_libraries(async_audio_test
 ${catkin_LIBRARIES}
)

target_link_libraries(rec_obj_votation_test
 ${catkin_LIBRARIES}
)

target_link_libraries(cubes_challenge
 ${catkin_LIBRARIES}
)

target_link_libraries(p_and_g_test
 ${catkin_LIBRARIES}
)

target_link_libraries(final_cubes_challange
 ${catkin_LIBRARIES}
)

#target_link_libraries(visit_my_home_test
# ${catkin_LIBRARIES}
#)

#target_link_libraries(welcoming_visitor_test
# ${catkin_LIBRARIES}
#)

#target_link_libraries(catering_comfort_test
# ${catkin_LIBRARIES}
#)

#target_link_libraries(getting_to_know_my_home
# ${catkin_LIBRARIES}
#)
