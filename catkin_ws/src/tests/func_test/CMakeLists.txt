cmake_minimum_required(VERSION 2.8.3)
project(func_test)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  justina_tools
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
#   std_msgs
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
#  LIBRARIES func_test
#  CATKIN_DEPENDS roscpp rospy std_msgs
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
# add_library(func_test
#   src/${PROJECT_NAME}/func_test.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(func_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(func_test_node src/func_test_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(func_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(func_test_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS func_test func_test_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_func_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_executable(grasp_object_test
  src/grasp_object_test.cpp
)

target_link_libraries(grasp_object_test
  ${catkin_LIBRARIES}
)

add_dependencies(grasp_object_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#############
add_executable(find_person_test
  src/find_person_test.cpp
)

target_link_libraries(find_person_test
  ${catkin_LIBRARIES}
)

add_dependencies(find_person_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#############
add_executable(grasp_two_objects
  src/grasp_two_objects.cpp
)

target_link_libraries(grasp_two_objects
  ${catkin_LIBRARIES}
)

add_dependencies(grasp_two_objects ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#############
add_executable(drop_object_test 
  src/drop_object_test.cpp
)

target_link_libraries(drop_object_test
  ${catkin_LIBRARIES}
)

add_dependencies(drop_object_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


############
add_executable(find_all_objects_test
  src/find_all_objects_test.cpp
)

target_link_libraries(find_all_objects_test
  ${catkin_LIBRARIES}
)

add_dependencies(find_all_objects_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


############
add_executable(place_object_test
  src/place_object_test.cpp
)

target_link_libraries(place_object_test
  ${catkin_LIBRARIES}
)

add_dependencies(place_object_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


############
add_executable(follow_person_test
  src/follow_person_test.cpp
)

target_link_libraries(follow_person_test
  ${catkin_LIBRARIES}
)

add_dependencies(follow_person_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


############
add_executable(guide_person_test
  src/guide_person_test.cpp
)

target_link_libraries(guide_person_test
  ${catkin_LIBRARIES}
)

add_dependencies(guide_person_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

###########
add_executable(find_gesture_test
  src/find_gesture_test.cpp
)

target_link_libraries(find_gesture_test
  ${catkin_LIBRARIES}
)

add_dependencies(find_gesture_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

###########
add_executable(pano_maker_test
  src/pano_maker_test.cpp
)

target_link_libraries(pano_maker_test
  ${catkin_LIBRARIES}
)

add_dependencies(pano_maker_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

###########
add_executable(find_waving_test
  src/find_waving_test.cpp
)

target_link_libraries(find_waving_test
  ${catkin_LIBRARIES}
)

add_dependencies(find_waving_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

###########
add_executable(segment_cubes
  src/segment_cubes.cpp
)

target_link_libraries(segment_cubes
  ${catkin_LIBRARIES}
)

add_dependencies(segment_cubes ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

###########
add_executable(request_object_test
  src/request_object.cpp
)

target_link_libraries(request_object_test
  ${catkin_LIBRARIES}
)

add_dependencies(request_object_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

###########
add_executable(grasp_cube_test
  src/grasp_cube_test.cpp
)

target_link_libraries(grasp_cube_test
  ${catkin_LIBRARIES}
)

add_dependencies(grasp_cube_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

###########
add_executable(arms_test
  src/arms_sm.cpp
)

target_link_libraries(arms_test
  ${catkin_LIBRARIES}
)

add_dependencies(arms_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

###########
add_executable(place_cube_test
  src/place_cube_test.cpp
)

target_link_libraries(place_cube_test
  ${catkin_LIBRARIES}
)

add_dependencies(place_cube_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
###########
add_executable(take_bag_from_hand
  src/take_bag_from_hand.cpp
)

target_link_libraries(take_bag_from_hand
  ${catkin_LIBRARIES}
)

add_dependencies(take_bag_from_hand ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
###########
add_executable(audio_source_test
  src/audio_source_test.cpp
)

target_link_libraries(audio_source_test
  ${catkin_LIBRARIES}
)

add_dependencies(audio_source_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
###########
add_executable(place_cutlery_test
  src/place_cutlery_test.cpp
)

target_link_libraries(place_cutlery_test
  ${catkin_LIBRARIES}
)

add_dependencies(place_cutlery_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
###########
add_executable(segment_tray_test
  src/segment_tray_test.cpp
)

target_link_libraries(segment_tray_test
  ${catkin_LIBRARIES}
)

add_dependencies(segment_tray_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

###########
add_executable(find_skeleton_test
  src/find_skeleton_test.cpp
)

target_link_libraries(find_skeleton_test
  ${catkin_LIBRARIES}
)

add_dependencies(find_skeleton_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

###########
add_executable(is_open_door_test
  src/is_open_door_test.cpp
)

target_link_libraries(is_open_door_test
  ${catkin_LIBRARIES}
)

add_dependencies(is_open_door_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
