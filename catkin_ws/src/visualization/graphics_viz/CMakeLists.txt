cmake_minimum_required(VERSION 2.8.3)
project(graphics_viz)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  roslib
  sensor_msgs
  std_msgs
  visualization_msgs
  map_server
)

set(CMAKE_BUILD_TYPE Debug)

find_package(Qt5Widgets REQUIRED)

if (Qt5Widgets_FOUND)
    if (Qt5Widgets_VERSION VERSION_LESS 5.0)
        message(FATAL_ERROR "Minimum supported Qt5 version is 5.7.1!")
    endif()
else()
    message(SEND_ERROR "The Qt5Widgets library could not be found!")
endif(Qt5Widgets_FOUND)

message(Qt5Widgets_VERSION ":Version of QT.")

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR} 
  ${catkin_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}
)

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/resource.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/graphics_viz/glwidget.h include/graphics_viz/treemodel.h src/*.h)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

catkin_package()

add_definitions(${Qt5Widgets_DEFINITIONS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS} -std=c++11")
set(CMAKE_AUTOUIC ON)

qt5_wrap_ui(QT_FORMS_HPP ${QT_FORMS})
qt5_add_resources(QT_RESOURCES_CPP ${QT_RESOURCES})
qt5_wrap_cpp(QT_MOC_HPP ${QT_MOC})

#file(GLOB_RECURSE QT_SOURCES RELATIVE
#    ${CMAKE_CURRENT_SOURCE_DIR}
#    FOLLOW_SYMLINKS
#    src/glwidget.cpp
#    src/graphics_viz_node.cpp
#    src/mainwindow.cpp
#    src/QtRosNode.cpp
#    src/treeitem.cpp
#    src/treemodel.cpp
#)

#add_executable(
#        graphics_viz_node
#        src/graphics_viz_node.cpp
#        src/firstpersoncamera.cpp
#        src/orbitcamera.cpp
#        src/thirdpersoncamera.cpp
#        src/Texture.cpp
#        src/Shader.cpp
#        src/box.cpp
#        src/sphere.cpp
#        src/triangle.cpp
#        src/quad.cpp
#        src/lines.cpp
#        src/cylinder.cpp
#        src/grid.cpp
#        src/compositeModel.cpp
#        ${include_directories}
#        ${QT_FORMS_HPP}
#        ${QT_SOURCES}
#        ${QT_RESOURCES_CPP}
#        ${QT_MOC_HPP}
#        resources/resource.qrc
#)

#qt5_use_modules(graphics_viz_node Widgets)

#add_dependencies(graphics_viz_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#target_link_libraries(graphics_viz_node
#  ${QT_LIBRARIES}
#  ${catkin_LIBRARIES} Qt5OpenGL
#  GLEW GL pthread rt Xrandr Xinerama Xi Xcursor xcb-present xcb-sync xshmfence Xxf86vm Xfixes Xext X11 xcb Xau Xdmcp dl assimp fontconfig freetype freeimage
#)
