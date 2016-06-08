#ifndef ACT_PLN_SRV_MAN
#define ACT_PLN_SRV_MAN

#include "action_planner/robot_knowledge.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "visualization_msgs/MarkerArray.h"
//#include "vision/vsn_findbycolor.h"
#include "ros/console.h"
#include "ros/ros.h"
#include <string>
#include <vector>

class ServiceManager
{
//private:
	//CommandManager() {}
public:
	//CommandManager() {}	//default public constructor
	
	/*
	* SP-GEN services callers
	*/
	bool spgenSay(std::string, int);
	void spgenAsay(std::string);

	/*
	*	ARMS services callers
	*/
	bool armsOpenGrip(RobotKnowledge::ARM_SIDE, std_msgs::Float32);
	bool armsCloseGrip(RobotKnowledge::ARM_SIDE, std_msgs::Float32);
	bool armsAbsPos(RobotKnowledge::ARM_SIDE, std_msgs::Float32, std_msgs::Float32, std_msgs::Float32, std_msgs::Float32, std_msgs::Float32, std_msgs::Float32, std_msgs::Float32);
	bool armsGoTo(RobotKnowledge::ARM_SIDE, std_msgs::String);

	/*
	*	MVNPLN services callers
	*/
	bool mpGetClose(std_msgs::String);
	bool mpGetClose(std_msgs::Float32, std_msgs::Float32);
	bool mpGetClose(std_msgs::Float32, std_msgs::Float32, std_msgs::Float32);
	bool mpMove(std_msgs::Float32, std_msgs::Float32&);
	bool mpMove(std_msgs::Float32, std_msgs::Float32, std_msgs::Float32&, std_msgs::Float32&);

	/*
	*	HEAD services callers
	*/
	bool hdLookAt(std_msgs::Float32, std_msgs::Float32, std_msgs::Float32&, std_msgs::Float32&);
	bool hdTorque(std_msgs::Bool);

	/*
	*	VISION services callers
	*/
	bool vsnFindOnPlanes(std_msgs::String, visualization_msgs::MarkerArray&);
	bool vsnFindOnPlanes(std_msgs::String, visualization_msgs::MarkerArray&, std::vector<float> &);
	bool vsnPersonReco(std::string&);
	bool vsnFindByColor(std::string&);

	/*
	*	PRS-FND services callers
	*/
	bool prsfndFind(std::string, int);
	bool prsfndRemember(std::string, int);
	bool prsfndAmnesia(int);
	
	/*
	* LANG_UND service callers
	*/
	bool langundProcess(std::string, std::string&);
};

#endif
