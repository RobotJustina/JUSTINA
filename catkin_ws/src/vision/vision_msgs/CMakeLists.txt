cmake_minimum_required(VERSION 2.8.3)
project(vision_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
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
  MSG_VisionPlasticTray.msg
  MSG_ObjectsPile.msg
  MSG_VisionDishwasher.msg
  BoundingBox.msg
  BoundingBoxes.msg
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
  SRV_DetectPlasticTrayZones.srv
  SRV_FindObjectsPiles.srv
  SRV_FindDishwasher.srv
  image_srv.srv
)

add_action_files(
  FILES
  CheckForObjects.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs geometry_msgs   sensor_msgs   std_msgs   vision_msgs
)

catkin_package(
	CATKIN_DEPENDS
	actionlib_msgs
	geometry_msgs
	sensor_msgs
	message_runtime
	std_msgs
	)

include_directories(
	${catkin_INCLUDE_DIRS}
	)

