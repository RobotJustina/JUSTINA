cmake_minimum_required(VERSION 2.8.3)
project(justina_simple_gui)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  navig_msgs
  hri_msgs
)

find_package(Qt4 REQUIRED)
SET(QT_USE_QTXML TRUE)
include(${QT_USE_FILE})

catkin_package(
)

include_directories(
  include 
  ${CMAKE_CURRENT_BINARY_DIR} 
  "/usr/include/qt4" 
  "/usr/include/qt4/QtGui" 
  "/usr/include/qt4/QtCore" 
  ${catkin_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}
)

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/resource.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/local/*.hpp src/*.h)


QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

add_executable(
	justina_simple_gui_node
        ${QT_FORMS_HPP}
		${QT_SOURCES}
		${QT_RESOURCES_CPP}
		${QT_MOC_HPP}
)

add_dependencies(justina_simple_gui_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(justina_simple_gui_node
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
)
