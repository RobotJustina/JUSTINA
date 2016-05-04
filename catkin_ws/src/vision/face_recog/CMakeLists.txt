cmake_minimum_required(VERSION 2.8.3)
project(face_recog)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  justina_tools
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  vision_msgs
)

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(face_recog_node 
  src/face_recog_node.cpp
  #src/MyClass.cpp
  #src/MyOtherClass.cpp
  #src/MagicalSourceCode.cpp
)

add_dependencies(face_recog_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(face_recog_node
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)
