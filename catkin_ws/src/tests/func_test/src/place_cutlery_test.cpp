#include "ros/ros.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTasks.h"

int main(int argc, char ** argv){
	
	std::cout << "INITIALIZING A TEST JUST FOR TEST..." << std::endl;
	ros::init(argc, argv, "place_cutlery_test");
	ros::NodeHandle nh;

	JustinaTasks::setNodeHandle(&nh);
	ros::Rate loop(10);
	
	int state = 0;
	bool finished = false;
	bool withLeft = false;
	int objTaken = 0;
	int chances =0;
	

	while(ros::ok() && cv::waitKey(1) != 'q'){
		

		switch(state){
			case 0:
				std::cout << "P & G Test...-> delivering the objects" << std::endl;

      			if(withLeft){
      				JustinaHRI::say("I am going to deliver an object with my left arm");
      				if(!JustinaTasks::placeCutleryOnDishWasherMontreal(withLeft, 0, 0.16))
      					if(!JustinaTasks::placeCutleryOnDishWasherMontreal(withLeft, 0, 0.16))
      						std::cout << "P & G Test...-> cannot deliver the object" << std::endl;
      				//JustinaManip::laGoTo("home", 6000);
      				withLeft=false;
      				objTaken --;
      			}
      			else{
      				JustinaHRI::say("I am going to deliver an object with my right arm");
      				if(!JustinaTasks::placeCutleryOnDishWasherMontreal(withLeft, 0, 0.16))
      					if(!JustinaTasks::placeCutleryOnDishWasherMontreal(withLeft, 0, 0.16))
      						std::cout << "P & G Test...-> cannot deliver the object" << std::endl;
      				//JustinaManip::raGoTo("home", 6000);
      				withLeft=true;
      				objTaken --;
      			}

      			chances++;

      			if(objTaken==0 && chances ==2)
      				state=1;

      			else
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
