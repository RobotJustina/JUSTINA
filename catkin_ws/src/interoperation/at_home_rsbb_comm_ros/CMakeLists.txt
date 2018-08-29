cmake_minimum_required(VERSION 2.8.3)
project(roah_rsbb_comm_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  message_generation
  std_msgs
  std_srvs
)

add_definitions("-Wall")
add_definitions("-std=c++0x")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED)

####### BEGIN ;EP

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP yaml-cpp>=0.5.0)
if(YAML_CPP_FOUND)
  include_directories(${YAML_CPP_INCLUDE_DIRS})
  link_directories(${YAML_CPP_LIBRARY_DIRS})
  message(STATUS "  using yaml-cpp from ${YAML_CPP_LIBRARIES} (YAML_CPP_LIBRARY_DIRS: ${YAML_CPP_LIBRARY_DIRS}, YAML_CPP_INCLUDE_DIRS: ${YAML_CPP_INCLUDE_DIRS})")
else()
  include_directories("/tmp/yaml-cpp-0.5.1/include")
  set(YAML_CPP_LIBRARIES "/tmp/yaml-cpp-0.5.1/libyaml-cpp.a")
  if(NOT EXISTS ${YAML_CPP_LIBRARIES})
    message(FATAL_ERROR "yaml-cpp>=0.5.0 not found, please run:
    cd /tmp &&
    wget https://yaml-cpp.googlecode.com/files/yaml-cpp-0.5.1.tar.gz &&
    tar xavf yaml-cpp-0.5.1.tar.gz &&
    cd yaml-cpp-0.5.1 &&
    cmake . &&
    make -j8 yaml-cpp")
  endif()
  message(STATUS "  using yaml-cpp from ${YAML_CPP_LIBRARIES}")
endif()
####### END ;EP

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
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

# Generate messages in the 'msg' folder
file(GLOB MESSAGE_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/msg ${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg)
add_message_files(FILES ${MESSAGE_FILES})

## Generate services in the 'srv' folder
file(GLOB SERVICE_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/srv ${CMAKE_CURRENT_SOURCE_DIR}/srv/*.srv)
add_service_files(FILES ${SERVICE_FILES})

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  std_srvs
)


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
#  LIBRARIES roah_rsbb_comm_ros
CATKIN_DEPENDS geometry_msgs message_runtime std_msgs std_srvs # roscpp
#  DEPENDS system_lib
)


###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(comm/gen/roah_rsbb_msgs/)
add_subdirectory(comm/libs/)
add_subdirectory(comm/include/)
include_directories(
  ${INCLUDE_DIRS}
)

add_executable(comm src/comm.cpp)
add_dependencies(comm roah_rsbb_comm_ros_generate_messages_cpp)
target_link_libraries(comm roah_rsbb_msgs protobuf_comm ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

#add_executable(dummy src/dummy.cpp)
#add_dependencies(dummy roah_rsbb_comm_ros_generate_messages_cpp)
#target_link_libraries(dummy roah_rsbb_msgs ${catkin_LIBRARIES})


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
install(TARGETS comm
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
file(GLOB LAUNCH_FILES ${PROJECT_SOURCE_DIR}/launch/*.launch)
install(FILES ${LAUNCH_FILES}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_roah_rsbb_comm_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
