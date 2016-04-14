#include "JustinaHardware.h"

bool SetNodeHandle(ros::NodeHandle* nh)
{
}

//Methods for operating the mobile base
bool JustinaHardware::MoveBase(float dist)
{
}
bool JustinaHardware::MoveBase(float dist, float angle)
{
}

bool JustinaHardware::MoveToPose(float x, float y)
{
}

bool JustinaHardware::MoveToPose(float x, float y, float theta)
{
}

bool JustinaHardware::MoveToPoseRel(float x, float y)
{
}

bool JustinaHardware::StartMoveBase(float dist)
{
}

bool JustinaHardware::StartMoveBase(float dist, float angle)
{
}

bool JustinaHardware::StartMoveToPose(float x, float y)
{
}

bool JustinaHardware::StartMoveToPose(float x, float y, float theta)
{
}

bool JustinaHardware::StartMoveToPoseRel(float x, float y)
{
}

bool JustinaHardware::GetCurrentRobotPos(float& robotX, float& robotY, float& robotTheta)
{
}

//Methods for speech synthesis and recognition
bool JustinaHardware::Say(std::string strToSay)
{
}

bool JustinaHardware::WaitForRecogSpeech(std::string& recognized, int time_out_ms)
{
}

//Methods for operating arms
bool JustinaHardware::LeftArmGoTo(float x, float y, float z)
{
}

bool JustinaHardware::LeftArmGoTo(float x, float y, float z, float roll, float pitch, float yaw)
{
}

bool JustinaHardware::LeftArmGoTo(float x, float y, float z, float roll, float pitch, float yaw, float elbow)
{
}

bool JustinaHardware::LeftArmArticular(std::vector<float> angles)
{
}

bool JustinaHardware::LeftArmGoTo(std::string location)
{
}

bool JustinaHardware::LeftArmMove(std::string movement)
{
}

bool JustinaHardware::StartLeftArmGoTo(float x, float y, float z)
{
}

bool JustinaHardware::StartLeftArmGoTo(float x, float y, float z, float roll, float pitch, float yaw)
{
}

bool JustinaHardware::StartLeftArmGoTo(float x, float y, float z, float roll, float pitch, float yaw, float elbow)
{
}

bool JustinaHardware::StartLeftArmGoTo(std::string location)
{
}

bool JustinaHardware::StartLeftArmMove(std::string movement)
{
}

bool JustinaHardware::RightArmGoTo(float x, float y, float z)
{
}

bool JustinaHardware::RightArmGoTo(float x, float y, float z, float roll, float pitch, float yaw)
{
}

bool JustinaHardware::RightArmGoTo(float x, float y, float z, float roll, float pitch, float yaw, float elbow)
{
}

bool JustinaHardware::RightArmArticular(std::vector<float> angles)
{
}

bool JustinaHardware::RightArmGoTo(std::string location)
{
}

bool JustinaHardware::RightArmMove(std::string movement)
{
}

bool JustinaHardware::StartRightArmGoTo(float x, float y, float z)
{
}

bool JustinaHardware::StartRightArmGoTo(float x, float y, float z, float roll, float pitch, float yaw)
{
}

bool JustinaHardware::StartRightArmGoTo(float x, float y, float z, float roll, float pitch, float yaw, float elbow)
{
}

bool JustinaHardware::StartRightArmGoTo(std::string location)
{
}

bool JustinaHardware::StartRightArmMove(std::string movement)
{
}

//Methods for operating head
bool JustinaHardware::HeadGoTo(float pan, float tilt)
{
}

bool JustinaHardware::HeadGoTo(std::string position)
{
}

bool JustinaHardware::HeadMove(std::string movement)
{
}

bool JustinaHardware::StartHeadGoTo(float pan, float tilt)
{
}

bool JustinaHardware::StartHeadGoTo(std::string position)
{
}

bool JustinaHardware::StartHeadMove(std::string movement)
{
}
