#include "ros/ros.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTasks.h"

int main(int argc, char ** argv){
	
	std::cout << "INITIALIZING A TEST JUST FOR TEST..." << std::endl;
	ros::init(argc, argv, "segment_tray_test");
	ros::NodeHandle nh;

	JustinaTasks::setNodeHandle(&nh);
	ros::Rate loop(10);
	
	int state = 0;
	bool finished = false;
	vision_msgs::MSG_VisionPlasticTray tray;
    vision_msgs::MSG_VisionDishwasher dishwasher;

	
	while(ros::ok() && cv::waitKey(1) != 'q'){
		JustinaManip::hdGoTo(0, -0.7, 5000);

		switch(state){
			case 0:
				
				// JustinaVision::getTray(tray);
		        JustinaVision::getDishwasher(dishwasher);
                    
				state=0;

				break;
			case 1:
				finished = true;
				break;
		}
		loop.sleep();
		ros::spinOnce();
	}

	return 0;

}
