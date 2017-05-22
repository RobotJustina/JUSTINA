#include <ros/ros.h>
#include "justina_tools/JustinaTasks.h"

int main(int argc, char ** argv){

	ros::init(argc, argv, "drop_object_test");
	ros::NodeHandle nh;

	JustinaTasks::setNodeHandle(&nh);

	ros::Rate rate(15);

	int state = 0;
	bool exitSM = false;

	while(ros::ok() && !exitSM){

		switch(state){
			case 0:
				JustinaTasks::dropObject("", false);
				state = 1;
				break;
			case 1:
				std::cout << "NavigTest.->Somebody very stupid programmed this shit. " << std::endl;
				state = 2;
				break;
			defualt:
				exitSM = true;
				break;
		}
		rate.sleep();
		ros::spinOnce();
	}

	return 1;

}
