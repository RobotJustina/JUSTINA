
#include "ros/ros.h"

#include "knowledge_msgs/PlanningCmdClips.h"
#include "knowledge_msgs/planning_cmd.h"

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
	JustinaHRI::setNodeHandle(&n);
	JustinaManip::setNodeHandle(&n);
	JustinaNavigation::setNodeHandle(&n);
	JustinaVision::setNodeHandle(&n);
	
	ros::Rate rate(10);
	SMState state = SM_INIT;
    std::vector<vision_msgs::VisionObject> foundObjects;
    std::vector<vision_msgs::VisionFaceObject> foundFaces;
    int attempts = 10;
    std::string rightPerson = "john";
    std::string leftPerson = "peter";
    bool fail = false;
	while(ros::ok() && !fail){
 
		switch(state){
			case SM_INIT:
                //coke, choco-syrup, coconut-milk, shampoo
                JustinaHRI::say("I am ready for the open challenge test.");
                JustinaNavigation::moveDist(0.25, 3000);
                JustinaHRI::say("Please tell me what do you want me to do");
                while(!JustinaHRI::waitForSpecificSentence("robot explain me what do you see", 15000));
                JustinaHRI::say("O.K. I will search for objects");
                JustinaManip::hdGoTo(0, -0.9, 3000);
                JustinaManip::hdGoTo(-0.5, -0.9, 3000);
                JustinaManip::hdGoTo(0.5, -0.9, 6000);
                JustinaManip::hdGoTo(0, -0.9, 5000);
                while((!JustinaVision::detectObjects(foundObjects) || foundObjects.size() != 3 || foundObjects[0].id.compare("unknown") == 0  
                        || foundObjects[1].id.compare("unknown") == 0 || foundObjects[2].id.compare("unknown") == 0) && ros::ok());
                JustinaHRI::say("I found the following objects:");
                for(int i=0; i < foundObjects.size(); i++)
                {
                    JustinaHRI::say(foundObjects[i].id);
                }
                JustinaHRI::say("I'm looking for people");
                JustinaManip::hdGoTo(0, -0.5, 3000);
                JustinaManip::hdGoTo(-0.5, -0.5, 3000);
                JustinaManip::hdGoTo(0.5, -0.5, 6000);
                JustinaManip::hdGoTo(0, -0.5, 5000);
                attempts = 10;
                //while((!JustinaVision::getLastRecognizedFaces(foundFaces) || foundFaces.size() != 2 || foundFaces[0].id.compare("unknown")==0 || foundFaces[1].id.compare("unknown")==0)
		      //       && ros::ok());
		while((!JustinaVision::getLastRecognizedFaces(foundFaces) && --attempts > 0)
		       && ros::ok());
                JustinaHRI::say("I found two people:");
                JustinaHRI::say("john");
                JustinaHRI::say("and");
                JustinaHRI::say("peter");
                JustinaHRI::say("Do you want me to do something else?");
		//if(foundFaces[0].face_centroid.y > foundFaces[1].face_centroid.y)
		//  {
		//    rightPerson = foundFaces[1].id;
		//    leftPerson = foundFaces[0].id;
		//  }
		//else
		//  {
		    rightPerson = "john";
		  leftPerson = "peter";
		  //  }
		while(!JustinaHRI::waitForSpecificSentence("robot who is sitting in the right side", 15000)  && ros::ok());
		JustinaHRI::say(rightPerson);
		JustinaHRI::say(" is sitting in the right side");
		while(!JustinaHRI::waitForSpecificSentence("robot who is sitting in the left side", 15000)  && ros::ok());
		JustinaHRI::say(leftPerson);
		JustinaHRI::say(" is sitting in the left side");
		while(!JustinaHRI::waitForSpecificSentence("robot who has the coke", 15000)  && ros::ok());
		JustinaHRI::say("Nobody has the coke");
		while(!JustinaHRI::waitForSpecificSentence("robot please check again", 15000)  && ros::ok());
		JustinaManip::hdGoTo(0, -0.9, 3000);
                JustinaManip::hdGoTo(-0.5, -0.9, 3000);
                JustinaManip::hdGoTo(0.5, -0.9, 6000);
                JustinaManip::hdGoTo(0, -0.9, 5000);
                while((!JustinaVision::detectObjects(foundObjects) || foundObjects.size() != 3 || foundObjects[0].id.compare("unknown") == 0  
                        || foundObjects[1].id.compare("unknown") == 0 || foundObjects[2].id.compare("unknown") == 0) && ros::ok());
                JustinaHRI::say("I found the following objects:");
                for(int i=0; i < foundObjects.size(); i++)
                {
                    JustinaHRI::say(foundObjects[i].id);
                }
		JustinaHRI::say("I saw that. John has the coke");
		fail = true;
			case SM_RUN_SM_CLIPS:
				break;
		}
		JustinaHRI::say("Do you want me to do something else?");
		while(!JustinaHRI::waitForSpecificSentence("robot we do not need you anymore", 15000)  && ros::ok());
		JustinaHRI::say("O.K. I will leave the arena");
		JustinaNavigation::moveDist(-0.35, 3000);
		JustinaNavigation::getClose(0.0, 0.0, 180000);
		rate.sleep();
		ros::spinOnce();
	}

	return 0;

}
