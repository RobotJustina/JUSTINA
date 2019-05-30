#include "ros/ros.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTasks.h"

int main(int argc, char ** argv){
	
	std::cout << "INITIALIZING A TEST JUST FOR TEST..." << std::endl;
	ros::init(argc, argv, "find_lines_test");
	ros::NodeHandle nh;

	JustinaTasks::setNodeHandle(&nh);
	ros::Rate loop(10);
	
	int state = 0;
	bool finished = false;
	float x1, x2,y1, y2, z1, z2;

	while(ros::ok() && cv::waitKey(1) != 'q'){
		

		switch(state){
			case 0:
				JustinaVision::findLine(x1, y1, z1, x2, y2, z2); 
				
				

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
