cmake_minimum_required(VERSION 2.8.3)
project(vision_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs 
  message_generation
)
 
add_message_files(
  FILES
  VisionObject.msg
  VisionObjectList.msg
  VisionPlane.msg
  SkeletonJoint.msg
  Skeleton.msg
  Skeletons.msg
  VisionFaceObject.msg
  VisionFaceObjects.msg
  VisionFaceTrainObject.msg
  GestureSkeleton.msg
  GestureSkeletons.msg
  HandSkeletonPos.msg
  VisionRect.msg
  TrackedObject.msg
  Cube.msg
  CubesSegmented.msg 
)

add_service_files(
  FILES
  RecognizeObject.srv
  RecognizeObjects.srv
  DetectObjects.srv
  TrainObject.srv
  FindLines.srv
  FindPlane.srv
  GetThermalAngle.srv
  GetPanoramic.srv
  DetectGripper.srv
  GetFacesFromImage.srv
  FindWaving.srv
  FaceRecognition.srv
  GetCubes.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs   sensor_msgs   std_msgs   vision_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

