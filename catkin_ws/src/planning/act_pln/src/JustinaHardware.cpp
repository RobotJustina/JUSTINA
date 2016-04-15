#include "JustinaHardware.h"

ros::Publisher JustinaHardware::pub_Head_GoalPose;
ros::Publisher JustinaHardware::pub_La_GoalPose;
ros::Publisher JustinaHardware::pub_Ra_GoalPose;
ros::Publisher JustinaHardware::pub_Spg_Say;
ros::Subscriber JustinaHardware::sub_Spr_Recognized;
ros::Subscriber JustinaHardware::sub_Spr_Hypothesis;
ros::ServiceClient JustinaHardware::srv_Spg_Say;

bool JustinaHardware::SetNodeHandle(ros::NodeHandle* nh)
{
    JustinaHardware::pub_Head_GoalPose = nh->advertise<std_msgs::Float32MultiArray>("/hardware/head/goal_pose", 1);
    JustinaHardware::pub_La_GoalPose = nh->advertise<std_msgs::Float32MultiArray>("/hardware/left_arm/goal_pose", 1); 
    JustinaHardware::pub_Ra_GoalPose = nh->advertise<std_msgs::Float32MultiArray>("/hardware/right_arm/goal_pose", 1);
    JustinaHardware::sub_Spr_Recognized = nh->subscribe("/hri/sp_rec/recognized", 1, &JustinaHardware::callbackRecognized);
    JustinaHardware::sub_Spr_Hypothesis = nh->subscribe("/recognizedSpeech", 1, &JustinaHardware::callbackSpeechHypothesis);
    JustinaHardware::srv_Spg_Say = nh->serviceClient<bbros_bridge::Default_ROS_BB_Bridge>("/spg_say");
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
	std_msgs::Float32MultiArray laangles;
	laangles.data = angles;
	
	pub_La_GoalPose.publish(laangles);
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
	std_msgs::Float32MultiArray raangles;
	raangles.data = angles;
	
	pub_Ra_GoalPose.publish(raangles);
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

void JustinaHardware::callbackRecognized(const std_msgs::String::ConstPtr& msg)
{
    std::cout << "ACT-PLN.->Received recognized speech: " << msg->data << std::endl;
    std_msgs::Float32MultiArray angles;
    for(int i=0; i<7; i++) angles.data.push_back(0.5);
    JustinaHardware::pub_La_GoalPose.publish(angles);
}

void JustinaHardware::callbackSpeechHypothesis(const bbros_bridge::RecognizedSpeech::ConstPtr& msg)
{
    std::cout << "El action planner que sí sirve.->Received recognized:" << std::endl;
    for(int i=0; i< msg->hypotesis.size(); i++)
        std::cout << "Hypothesis:" << msg->hypotesis[i] << std::endl;

    bbros_bridge::Default_ROS_BB_Bridge srvSay;
	std_msgs::Float32MultiArray laangles;
	std_msgs::Float32MultiArray raangles;
	std_msgs::Float32MultiArray headangles;

    std::string reco = msg->hypotesis[0];
    std::cout << "Recognized speech: " << reco << std::endl;
    if(reco.compare("robot say hello") == 0)
    {
        srvSay.request.parameters = "Hello fellow life forms";
        srvSay.request.timeout = 10000;
        JustinaHardware::srv_Spg_Say.call(srvSay);
    }
    if(reco.compare("robot move your left arm") == 0)
    {
	std::vector<float> angles;
	angles.push_back(-.5);
	angles.push_back(0);
	angles.push_back(0);
	angles.push_back(1);
	angles.push_back(0);
	angles.push_back(0);
	angles.push_back(0);
	
	LeftArmArticular(angles);
    }
    if(reco.compare("robot move your right arm") == 0)
    {
	std::vector<float> angles;
	angles.push_back(-.5);
	angles.push_back(0);
	angles.push_back(0);
	angles.push_back(1);
	angles.push_back(0);
	angles.push_back(0);
	angles.push_back(0);
	
	RightArmArticular(angles);
    }
	if(reco.compare("robot stand by") == 0)
    {
	std::vector<float> angles;
	angles.push_back(0);
	angles.push_back(0);
	angles.push_back(0);
	angles.push_back(0);
	angles.push_back(0);
	angles.push_back(0);
	angles.push_back(0);
	
	LeftArmArticular(angles);
	RightArmArticular(angles);
    }

    if(reco.compare("robot move your head") == 0)
    {
	raangles.data.push_back(-1);
	raangles.data.push_back(0);
	
	pub_Head_GoalPose.publish(headangles);
    }
}
