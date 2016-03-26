cmake_minimum_required(VERSION 2.8.3)
project(obj_detector_yisus)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  vision_msgs
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(obj_detector_yisus_node 
  src/obj_detector_yisus.cpp
  src/Utils/ColorObjectRecognizer.cpp
  src/Utils/HuObjectRecognizer.cpp
  src/Utils/LineSegment.cpp
  src/Utils/ObjectsExtractor.cpp
  src/Utils/PersonExtractor.cpp
  src/Utils/PlanarHorizontalSegment.cpp
  src/Utils/Plane3D.cpp
  src/Utils/Transform.cpp
)

target_link_libraries(obj_detector_yisus_node
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   ${catkin_LIBRARIES}
)

