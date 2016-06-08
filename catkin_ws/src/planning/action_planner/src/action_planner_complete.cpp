#include "action_planner/object_perception.h"
#include "action_planner/navigation_sm.h"
#include "action_planner/speech_understanding_sm.h"
#include "action_planner/know_home_sm.h"
#include "action_planner/granny_annie_sm.h"
#include "action_planner/default_sm.h"
#include "action_planner/welcoming_sm.h"
#include "action_planner/robot_knowledge.h"
#include "action_planner/service_manager.h"
#include "action_planner/primitives_tasks.h"
#include "action_planner/follow_guiding.h"
#include "action_planner/gpsr.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <sstream>
#include <algorithm>

char *getCmdOptions(char **begin, char **end, const std::string &option)
{
	char **itr = std::find(begin, end, option);

	if(itr != end && ++itr != end)
		return *itr;

	return 0;
}
bool cmdOptionsExists(char **begin, char **end, const std::string &option)
{
	return std::find(begin, end, option) != end;
}
void printUsage()
{
	std::cout << "usage: action_planner [-t <test_name>]" << std::endl;
	std::cout << "current states machines are:" << std::endl; 
	std::cout << "\tobj_perception\tRockIn Object Perception Functionality Benchmark" << std::endl;
	std::cout << "\tnavigation\tRockIn Navigation Functionality Benchmark" << std::endl;
	std::cout << "\tspeech_und\tRockIn Speech Understanding Functionality Benchmark" << std::endl;
	std::cout << "\tknow_home\tRockIn Task Getting to know my home" << std::endl;
	std::cout << "\twelcoming\tRockIn Task Welcoming visitors" << std::endl;
	std::cout << "\tgranny_annie\tRockIn Task Catering for Granny Annie's Comfort" << std::endl;
}
RobotKnowledge::SM parseArgument(std::string taskName)
{
	//get the test to execute
	//std::string taskName(getCmdOptions(argv, argv+argc, "-t"));

	if(taskName.compare("obj_perception")==0)
		return RobotKnowledge::Perception_FB;
	else if(taskName.compare("navigation")==0)
		return RobotKnowledge::Navigation_FB;
	else if(taskName.compare("speech_und")==0)
		return RobotKnowledge::Speech_FB;
	else if(taskName.compare("know_home")==0)
		return RobotKnowledge::KnowHome_TB;
	else if(taskName.compare("welcoming")==0)
		return RobotKnowledge::Welcoming_TB;
	else if(taskName.compare("granny_annie")==0)
		return RobotKnowledge::GrannyAnnie_TB;
	else if(taskName.compare("gpsr")==0)
		return RobotKnowledge::GPSR;
	else
		return RobotKnowledge::DefaultTest;
}
int main(int argc, char** argv)
{
	std::cout << "INITIALIZING ACT_PLN BY MARCOSOFT.... (La prueba que funciono primer fue la de marco...(como siempre)" << std::endl;

	if(!cmdOptionsExists(argv, argv+argc, "-t") || cmdOptionsExists(argv, argv+argc, "-h"))
	{
		printUsage();
		return 0;
	}
	
	RobotKnowledge::SM testToExecute = parseArgument(std::string(getCmdOptions(argv, argv+argc, "-t")));

	//Set up ROS env
	ros::init(argc, argv, "action_planner");

	ros::NodeHandle nh;
	ros::Subscriber rs;

	//Set up ACT_PLN env
	ServiceManager srv_man;
	PrimitivesTasks pt(nh, rs);	//pass the nodeHandler and Subscriber to suscribe to the recospeech topic
	RobotKnowledge know;
	//StatesMachines SM;

	ObjectPerceptionSM objSM(pt, &nh);
	NavigationSM navSM(pt, &nh);
	SpeechUnderstandingSM speechSM(pt);
	KnowHomeSM knowSM(pt);
	WelcomingSM welcomingSM(pt, &nh);
	GrannyAnnieSM gaSM(pt);
	GPSRSM gpsrSM(&nh);
	FollowGuidingSM followSM(pt);
	DefaultSM defSM(pt);

	//Execute the selected test
	switch(testToExecute)
	{
		case RobotKnowledge::Perception_FB:
			objSM.execute();
			break;
		case RobotKnowledge::Navigation_FB:
			navSM.execute();
			break;
		case RobotKnowledge::Speech_FB:
			speechSM.execute();
			break;
		case RobotKnowledge::KnowHome_TB:
			knowSM.execute();
			break;
		case RobotKnowledge::Welcoming_TB:
			welcomingSM.execute();
			break;
		case RobotKnowledge::GrannyAnnie_TB:
			gaSM.execute();
			break;
		case RobotKnowledge::GPSR:
			gpsrSM.execute();
			break;
		case RobotKnowledge::DefaultTest:
			defSM.execute();
			break;
		default:
			std::cout << "Invalid Test selection." << std::endl;
			break;
	}

	//TODO: test following
	//bool rememberHuman(std::string);
	//cmd_man.mpGetClose(std_msgs::String);
	//cmd_man.mpGetClose(std_msgs::Float32, std_msgs::Float32);
	//cmd_man.mpGetClose(std_msgs::Float32, std_msgs::Float32, std_msgs::Float32);
	//cmd_man.hdLookAt(distance, angle);
	//bool hdTorque(std_msgs::Bool);

	return 0;
}
