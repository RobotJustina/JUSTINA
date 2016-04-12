cmake_minimum_required(VERSION 2.8.3)
project(justina_gui)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
  map_server
  hri_msgs
)

find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/resource.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/local/*.hpp src/MainWindow.h src/QtRosNode.h)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/justina_gui_node.cpp src/MainWindow.cpp QtRosNode.cpp)

add_executable(
	justina_gui_node
		${QT_SOURCES}
		${QT_RESOURCES_CPP}
		${QT_FORMS_HPP}
		${QT_MOC_HPP}
)

target_link_libraries(justina_gui_node
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
)
