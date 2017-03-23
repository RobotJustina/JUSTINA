cmake_minimum_required(VERSION 2.8.3)
project(knowledge_msgs)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        message_generation
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  KnownLocations.srv
  AddUpdateKnownLoc.srv
  GetPredefinedQuestions.srv
)

add_message_files(
  FILES
  MapKnownLocation.msg
  MapPredefinedQuestions.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        knowledge_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
