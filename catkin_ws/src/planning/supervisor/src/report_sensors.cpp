#include <iostream>
#include <sstream>

#include "ros/ros.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaKnowledge.h"
#include "std_msgs/Bool.h"

bool nav_flag = 0;

void callbackCmdNavigation(const std_msgs::Bool::ConstPtr& msg){
	std::cout << "Test report robot position" << std::endl;
	nav_flag = msg->data;	
}

void statusNav(){
        std::map<std::string, std::vector<float> > locations;
        JustinaKnowledge::getKnownLocations(locations);

        for(std::map<std::string, std::vector<float> >::iterator it=locations.begin(); it!=locations.end(); ++it){
            std::cout << "x: " << it->second[0] << std::endl;
            std::cout << "y: " << it->second[1] << std::endl;
            
        }
}

int main(int argc, char ** argv) {
	std::cout << "Report sensors to KDB Node" << std::endl;
	ros::init(argc, argv, "sensor_report");
	ros::NodeHandle nh;
	ros::Rate rate(10);

	float x, y, theta;
	
	ros::Subscriber subCmdNavigation = nh.subscribe("/supervisor/robot_pose", 1, callbackCmdNavigation);
	
	JustinaNavigation::setNodeHandle(&nh);

	while (ros::ok()) {

		if(nav_flag){
			JustinaNavigation::getRobotPose(x, y, theta);
			std::cout << "Robot pose: " << x << ", " << y << ", " << theta << std::endl;
			JustinaNavigation::getRobotPoseFromOdom(x, y, theta);
			std::cout << "Robot pose from Odom: " << x << ", " << y << ", " << theta << std::endl;
		}
		rate.sleep();
		ros::spinOnce();
	}
	
	return 1; 
}
