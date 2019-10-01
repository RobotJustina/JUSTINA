#include <iostream>
#include <sstream>

#include "ros/ros.h"

#include "knowledge_msgs/PlanningCmdClips.h"

//#include "knowledge_msgs/kdbFilePath.h"

#include "std_msgs/Bool.h"
//#include "std_msgs/Empty.h"
//#include "std_msgs/String.h"
//#include "std_msgs/ColorRGBA.h"

ros::Publisher robot_pose_pub;

void callbackCmdNavigation(
		const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << "--------- Supervisor Command Navigation ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	std_msgs::Bool rp;
	rp.data = 1;

	robot_pose_pub.publish(rp);
}

void callbackCmdResponse(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << "------ Supervisor Command Finish -------" << std::endl;

	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;
	std::cout << "successful:" << msg->successful << std::endl;
	
	std_msgs::Bool rp;
	rp.data = 0;

	robot_pose_pub.publish(rp);
}

int main(int argc, char ** argv) {
	std::cout << "Supervisor Node" << std::endl;
	ros::init(argc, argv, "supervisor");
	ros::NodeHandle nh;
	ros::Rate rate(10);
	
	ros::Subscriber subCmdNavigation = nh.subscribe("/planning_clips/cmd_goto", 1, callbackCmdNavigation);
	
	ros::Subscriber subCmdResponse = nh.subscribe("/planning_clips/command_response", 1, callbackCmdResponse);
	robot_pose_pub = nh.advertise<std_msgs::Bool>("/supervisor/robot_pose", 1);
    

	while (ros::ok()) {
		rate.sleep();
		ros::spinOnce();
	}
	
	return 1; 
}
