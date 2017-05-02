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
  justina_tools
)


#set(CMAKE_AUTOMOC ON)
#set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt5Widgets REQUIRED)

if (Qt5Widgets_FOUND)
    if (Qt5Widgets_VERSION VERSION_LESS 5.0)
        message(FATAL_ERROR "Minimum supported Qt5 version is 5.7.1!")
    endif()
else()
    message(SEND_ERROR "The Qt5Widgets library could not be found!")
endif(Qt5Widgets_FOUND)

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR} 
#  ${Qt5Widgets_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}
  ${PCL_INCLUDE_DIRS}
)

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/resource.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/local/*.hpp src/*.h)

##set(GLOB CPP_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.cpp)
##set(GLOB CPP_HDRS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/local/*.hpp src/*.h)
##set(GLOB QRC_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/resource.qrc)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

catkin_package()

add_definitions(${Qt5Widgets_DEFINITIONS})

##qt5_wrap_ui(UIS_HDRS ${CMAKE_CURRENT_SOURCE_DIR} src/MainWindow.ui)
##qt5_wrap_cpp(MOC_SOURCES ${CPP_SOURCES} ${CPP_HDRS})
##qt5_add_resources(RESOURCES_RCC ${QRC_RESOURCES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS} -std=c++11")

qt5_wrap_ui(QT_FORMS_HPP ${QT_FORMS})
qt5_add_resources(QT_RESOURCES_CPP ${QT_RESOURCES})
qt5_wrap_cpp(QT_MOC_HPP ${QT_MOC})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

add_executable(
        justina_gui_node
        ${QT_FORMS_HPP}
                ${QT_SOURCES}
                ${QT_RESOURCES_CPP}
                ${QT_MOC_HPP}
)

qt5_use_modules(justina_gui_node Widgets)

add_dependencies(justina_gui_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(justina_gui_node
  ${QT_LIBRARIES}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)
