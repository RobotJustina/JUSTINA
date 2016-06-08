#ifndef ACT_PLN_PRIM_TSKS
#define ACT_PLN_PRIM_TSKS

#include "ros/ros.h"
#include "action_planner/robot_knowledge.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "action_planner/service_manager.h"
#include "visualization_msgs/MarkerArray.h"
#include <string>
#include <limits>
#include <math.h>
#include <algorithm>

class PrimitivesTasks
{
public:
	PrimitivesTasks(){};
	PrimitivesTasks(ros::NodeHandle&, ros::Subscriber&);
	/*
	* Take object primitives
	*/
	//bool takeObject(std::string objectName);
	bool searchObject(std::string, visualization_msgs::Marker&);
	bool searchSingleObject(visualization_msgs::Marker&);
	bool searchAndTakeObject(std::string, RobotKnowledge::ARM_SIDE);
	bool rememberHuman(std::string);
	
	/*
	* RecoSpeech primitives
	*/
	bool listen(std::string&, int);
private:
	geometry_msgs::Point transRobotToArm(geometry_msgs::Point);
};

#endif
