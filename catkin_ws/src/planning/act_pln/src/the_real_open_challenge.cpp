
#include "ros/ros.h"

#include "planning_msgs/PlanningCmdClips.h"
#include "planning_msgs/planning_cmd.h"

#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTasks.h"
#include "justina_tools/JustinaManip.h"

#include <vector>

enum SMState{
	SM_INIT,
	SM_SAY_WAIT_FOR_DOOR,
	SM_WAIT_FOR_DOOR,
	SM_NAVIGATE_TO_THE_LOCATION,
	SM_SEND_INIT_CLIPS,
	SM_RUN_SM_CLIPS
};

int main(int argc, char **argv){

	ros::init(argc, argv, "open_challenge_test");
	ros::NodeHandle n;
	
	ros::Rate rate(10);
	SMState state = SM_INIT;
    std::vector<vision_msgs::VisionObject> foundObjects;
    std::vector<vision_msgs::VisionFaceObject> foundFaces;
    int attempts = 10;

	while(ros::ok()){
 
		switch(state){
			case SM_INIT:
                //coke, choco-syrup, coconut-milk, shampoo
                JustinaHRI::say("I am ready for the open challenge test.");
                JustinaNavigation::moveDist(0.5, 3000);
                JustinaHRI::say("Please tell me what do you want me to do");
                while(!JustinaHRI::waitForSpecificSentence("explain the scene", 15000));
                JustinaHRI::say("O.K. I will search for objects");
                JustinaManip::hdGoTo(0, -0.9, 3000);
                JustinaManip::hdGoTo(-1, -0.9, 3000);
                JustinaManip::hdGoTo(1, -0.9, 6000);
                JustinaManip::hdGoTo(0, -0.9, 5000);
                while((!JustinaVision::detectObjects(foundObjects) || foundObjects.size() != 4 || foundObjects[0].id.compare("unknown") == 0  || foundObjects[1].id.compare("unknown") == 0
                       || foundObjects[2].id.compare("unknown") == 0  || foundObjects[3].id.compare("unknown") == 0 ));
                JustinaHRI::say("I found the following objects:");
                for(int i=0; i < foundObjects.size(); i++)
                {
                    JustinaHRI::say(foundObjects[i].id);
                }
                JustinaHRI::say("I will search for people");
                JustinaManip::hdGoTo(0, -0.5, 3000);
                JustinaManip::hdGoTo(-1, -0.5, 3000);
                JustinaManip::hdGoTo(1, -0.5, 6000);
                JustinaManip::hdGoTo(0, -0.5, 5000);
                attempts = 10;
                while((!JustinaVision::getLastRecognizedFaces(foundFaces) || foundFaces.size() != 2 || foundFaces[0].id.compare("unknown")==0 || foundFaces[1].id.compare("unknown")==0));
                JustinaHRI::say("I found two people:");
                JustinaHRI::say(foundFaces[0].id);
                JustinaHRI::say("and");
                JustinaHRI::say(foundFaces[1].id);
                JustinaHRI::say("Do you want me to do something else?");
				break;
			case SM_NAVIGATE_TO_THE_LOCATION:
				
				break;
			case SM_SEND_INIT_CLIPS:
				
				break;
			case SM_RUN_SM_CLIPS:
				break;
		}

		rate.sleep();
		ros::spinOnce();
	}

	return 0;

}
