cmake_minimum_required(VERSION 2.8.3)
project(qr_reader)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  image_transport
  justina_tools
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
#  LIBRARIES voice
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

#######################################
#
# Catkin
#
#######################################

# include_directories(include)
include_directories(
 	${catkin_INCLUDE_DIRS}
)

#######################################
#
# OpenCV
#
#######################################

find_package( OpenCV REQUIRED )

#######################################
#
# ZBar & libDecodeQR
#
#######################################

set(CONFIG_DIR "${CMAKE_CURRENT_LIST_DIR}/config/")
# set(DecodeQR_DIR "${CONFIG_DIR}")
# find_package(DecodeQR REQUIRED)
set(ZBar_DIR "${CONFIG_DIR}")
find_package(ZBar REQUIRED)

#######################################
#
# BOOST
#
#######################################

set(Boost_USE_STATIC_LIBS   ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME ON)
FIND_PACKAGE( Boost 1.40 COMPONENTS program_options system filesystem regex thread date_time REQUIRED )

#######################################
##
## BUILD qr_reader
##
#######################################

INCLUDE_DIRECTORIES(include ${catkin_INCLUDE_DIRS}
							${Boost_INCLUDE_DIR}
							${OpenCV_INCLUDE_DIRS}
							# ${DecodeQR_INCLUDE_DIR}
							${ZBar_INCLUDE_DIR}
					)

file(GLOB_RECURSE headers "src/*.h*")
file(GLOB_RECURSE sources "src/*.c*")

add_executable(qr_reader ${sources} ${headers} )

target_link_libraries(qr_reader ${catkin_LIBRARIES}
							${Boost_LIBRARIES}
							${OpenCV_LIBRARIES}
							# ${DecodeQR_LIBRARY}
							${ZBar_LIBRARY}
			)
add_dependencies(qr_reader qr_reader_generate_messages_cpp)