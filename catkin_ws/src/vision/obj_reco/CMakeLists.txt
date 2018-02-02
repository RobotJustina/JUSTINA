cmake_minimum_required(VERSION 2.8.3)
project(obj_reco)

 find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  vision_msgs
  justina_tools
  roslib
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
set(CMAKE_PREFIX_PATH "/usr/local/")
set(OpenCV_INCLUDE_DIRS "/usr/local/include")

#message("Veersion OpenCV" ${OpenCV_VERSION})
#message("Libs dir" ${OpeCV_LINK_DIR})
#message("Libs: " ${OpenCV_LIBS})
#message("OpenCV_LIB_COMPONENTS:" ${OpenCV_LIB_COMPONENTS})
#message("OpenCV_CONFIG_PATH" ${OpenCV_CONFIG_PATH})


generate_messages(
  DEPENDENCIES
  geometry_msgs sensor_msgs std_msgs vision_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(
  obj_reco_node 
  src/obj_reco_node.cpp
  src/ObjExtractor.cpp
  src/PlanarSegment.cpp
  src/Plane3D.cpp
  src/DetectedObject.cpp
  src/ObjRecognizer.cpp
)

add_dependencies(obj_reco_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(obj_reco_node
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES}
   /usr/local/lib/libopencv_tracking.so.3.3.1  
)


