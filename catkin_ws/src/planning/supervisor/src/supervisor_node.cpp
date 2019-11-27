#include <iostream>
#include <sstream>

#include "ros/ros.h"
#include "knowledge_msgs/PlanningCmdClips.h"
#include "knowledge_msgs/StrQueryKDB.h"

//#include "knowledge_msgs/kdbFilePath.h"

#include "std_msgs/Bool.h"
//#include "std_msgs/Empty.h"
//#include "std_msgs/String.h"
//#include "std_msgs/ColorRGBA.h"

#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaKnowledge.h"
#include "justina_tools/JustinaRepresentation.h"
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

using namespace boost::algorithm;

bool fnav = false;
float x, y, theta;
float sx, sy, st;
ros::ServiceClient srvCltQueryKDB;

ros::Publisher robot_pose_pub;

void callbackCmdNavigation(
		const knowledge_msgs::PlanningCmdClips::ConstPtr& msg) {
	std::cout << "--------- Supervisor Command Navigation ---------"
			<< std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	if(msg->name == "goto"){
    		std::map<std::string, std::vector<float> > locations;
        std::vector<std::string> tokens;
        std::string str = msg->params;
        split(tokens, str, is_any_of(" "));

		std::cout << "Location: " << tokens[1] << std::endl;

    		JustinaKnowledge::getKnownLocations(locations);
		if(locations.find(tokens[1]) == locations.end())
		{
			std::cout << "MvnPln.->Cannot get close to \"" << tokens[1] << "\". It is not a known location. " << std::endl;
		}
		sx = locations.find(tokens[1])->second[0];
		sy = locations.find(tokens[1])->second[1];

		std::cout << "Location: " << sx << ", " << sy << std::endl;
	}

	/*std_msgs::Bool rp;
	rp.data = 1;

	robot_pose_pub.publish(rp);*/
}

void callbackCmdResponse(const knowledge_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << "------ Supervisor Command Finish -------" << std::endl;

	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;
	std::cout << "successful:" << msg->successful << std::endl;

	float d;
	std::string kdb_loc, sup_loc;
	std::stringstream ss;
    std::vector<std::string> tokens;
    std::string str = msg->params;
    split(tokens, str, is_any_of(" "));

	if(msg->name == "goto"){
		JustinaNavigation::getRobotPose(x, y, theta);
		std::cout << "Robot pose: " << x << ", " << y << ", " << theta << std::endl;
		JustinaNavigation::getRobotPoseFromOdom(x, y, theta);
		std::cout << "Robot pose from Odom: " << x << ", " << y << ", " << theta << std::endl;
		std::cout << "Location: " << sx << ", " << sy << std::endl;

		d = pow(x - sx, 2) + pow(y - sy, 2);

		std::cout << "Distance: " << d << std::endl;

		sup_loc = "null";

		if(d < 0.25)
			sup_loc = tokens[1];
                    
		ss.str("");
                ss << "(assert (get_robot_location 1))";
                JustinaRepresentation::strQueryKDB(ss.str(), kdb_loc, 1000);

		if(kdb_loc == sup_loc){
			std::cout << "Supervisor and Kdb have the same information" << std::endl;
			std::cout << kdb_loc << std::endl;
			std::cout << sup_loc << std::endl;
		}
		else{
			std::cout << "Exist a difference between supervisor and kdb" << std::endl;
			std::cout << kdb_loc << std::endl;
			std::cout << sup_loc << std::endl;
		}
	}
}

int main(int argc, char ** argv) {
	std::cout << "Supervisor Node" << std::endl;
	ros::init(argc, argv, "supervisor");
	ros::NodeHandle nh;
	ros::Rate rate(10);
	
	ros::Subscriber subCmdNavigation = nh.subscribe("/planning_clips/cmd_goto", 1, callbackCmdNavigation);
	
	ros::Subscriber subCmdResponse = nh.subscribe("/planning_clips/command_response", 1, callbackCmdResponse);
	robot_pose_pub = nh.advertise<std_msgs::Bool>("/supervisor/robot_pose", 1);
	
	JustinaNavigation::setNodeHandle(&nh);
    	JustinaKnowledge::setNodeHandle(&nh);
	JustinaRepresentation::setNodeHandle(&nh);
    

	while (ros::ok()) {
		rate.sleep();
		ros::spinOnce();
	}
	
	return 1; 
}
