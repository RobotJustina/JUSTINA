#include <ros/ros.h>
#include "justina_tools/JustinaTasks.h"

int main(int argc, char ** argv){

	ros::init(argc, argv, "drop_object_test");
	ros::NodeHandle nh;

	JustinaTasks::setNodeHandle(&nh);
    std::cout << "Setting all nodes" << std::endl;
	ros::Rate rate(15);

	int state = 0;
	bool exitSM = false;
    vision_msgs::VisionRect rectWav;
    bool find;
    float minx;
    int indexMin = 0;
    int w = 1920, h = 1080;
	while(ros::ok() && !exitSM){

        std::cout << "State:" << state << std::endl;
		switch(state){
			case 0:
                std::cout << "Try to find waving" << std::endl;
                find = JustinaTasks::findWaving(-M_PI_2, M_PI_4, M_PI_2, -0.1, -0.15, -0.25, 500, rectWav);
                if(find){
                    state = 1;
                    JustinaHRI::waitAfterSay("Can someone call me", 10000);
                }else
                    state = 0;
				break;
            case 1:
                find = JustinaTasks::alignWithWaving(rectWav);
                if(find)
                    state = 2;
                else
                    state = 0;
                break;
			case 2:
				std::cout << "NavigTest.->Somebody very stupid programmed this shit. " << std::endl;
				state = 10000;
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
