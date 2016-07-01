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
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

generate_messages(
  DEPENDENCIES
  geometry_msgs sensor_msgs std_msgs vision_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
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
)

add_executable(
  obj_reco_marco 
  src/obj_reco_marco.cpp
  src/ObjExtractor.cpp
  src/PlanarSegment.cpp
  src/Plane3D.cpp
  src/DetectedObject.cpp
  src/ObjRecognizer.cpp
)

target_link_libraries(obj_reco_marco
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES}
)
