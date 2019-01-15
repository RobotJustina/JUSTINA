cmake_minimum_required(VERSION 2.8.3)
project(knowledge_msgs)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
	sensor_msgs
	vision_msgs
	nav_msgs
        message_generation
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  KnownLocations.srv
  AddUpdateKnownLoc.srv
  GetPredefinedQuestions.srv
  GetPredefinedArmsPoses.srv
  ask_store_name.srv
  find_person.srv
  parse_sentence_cfr.srv
  place_object.srv
  planning_cmd.srv
  search_remember_face.srv
  wait_for_command.srv
  wait_for_confirm.srv
  wait_for_switch.srv
  StrQueryKDB.srv
  InitKDB.srv
  IsPointInKnownArea.srv
  GetVisitLocationsPath.srv
  GetRoomOfPoint.srv
)

add_message_files(
  FILES
  MapKnownLocation.msg
  MapPredefinedQuestions.msg
  CFRParams.msg
  PlanningCFR.msg
  PlanningCmdClips.msg
  RecognizedSpeech.msg
  RepeatedSentence.msg
  SphinxSetFile.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
	vision_msgs
	sensor_msgs
	nav_msgs
        knowledge_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
