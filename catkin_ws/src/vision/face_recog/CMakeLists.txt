cmake_minimum_required(VERSION 2.8.3)
project(face_recog)

#include(src/facerecog/dlib/cmake)

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
find_package(dlib REQUIRED)
set(CMAKE_PREFIX_PATH "/usr/local/")
set(OpenCV_INCLUDE_DIRS "/usr/local/include")
SET(OPENCV_DNN_SAMPLES_REQUIRED_DEPS opencv_core opencv_imgproc opencv_dnn
    opencv_imgcodecs opencv_videoio opencv_highgui
    ${OpenCV_LIB_COMPONENTS})

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(face_recog_node 
  src/face_recog_node.cpp
  src/facerecog/faceobj.cpp
  src/facerecog/facerecog.cpp
  #src/MyClass.cpp
  #src/MyOtherClass.cpp
  #src/MagicalSourceCode.cpp
)

#add_executable(face_opencv_dnn
#  src/face_opencv_dnn.cpp
#)

add_dependencies(face_recog_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(face_recog_node
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  dlib::dlib
)

#add_dependencies(face_opencv_dnn ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#target_link_libraries(face_opencv_dnn
#  ${OpenCV_LIBS}
#  ${PCL_LIBRARIES}
#  ${catkin_LIBRARIES}
#  /usr/local/lib/libopencv_dnn.so.3.3.1  
#  ${OPENCV_DNN_SAMPLES_REQUIRED_DEPS}
#)
