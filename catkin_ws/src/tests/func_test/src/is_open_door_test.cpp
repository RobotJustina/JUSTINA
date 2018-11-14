#include "ros/ros.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTasks.h"
#include "justina_tools/JustinaNavigation.h"

int main(int argc, char ** argv){
	
	std::cout << "INITIALIZING A TEST JUST FOR TEST..." << std::endl;
	ros::init(argc, argv, "is_open_door_test");
	ros::NodeHandle nh;

	JustinaTasks::setNodeHandle(&nh);
    JustinaNavigation::setNodeHandle(&nh);
	ros::Rate loop(10);
	
	int state = 0;
	bool finished = false;

	while(ros::ok() && cv::waitKey(1) != 'q'){

		switch(state){
			case 0:
                if(JustinaNavigation::doorIsOpen(0.9, 2000))
                    std::cout << "The door is open" << std::endl;
                else
                    std::cout << "The door is close" << std::endl;
				break;
                state = 0;
			case 1:
				finished = true;
				break;
		}
		loop.sleep();
		ros::spinOnce();
	}

	return 0;

}
