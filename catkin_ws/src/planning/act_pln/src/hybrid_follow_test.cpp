#include "ros/ros.h"

#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTasks.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaRepresentation.h"

#include <vector>
#include <ctime>
#include <map>


    enum STATE{
        SM_GUIDING_MEMORIZING_OPERATOR_SAY,
        SM_GUIDING_MEMORIZING_OPERATOR_ELF,
        SM_GUIDING_MEMORIZING_OPERATOR,
        SM_GUIDING_PHASE,
        SM_GUIDING_STOP,
        SM_GUIDING_FINISHED,
        SM_WAIT_FOR_OPERATOR,
        SM_MEMORIZING_OPERATOR,
        SM_WAIT_FOR_LEGS_FOUND,
        SM_FOLLOWING_PHASE,
        SM_FOLLOWING_FINISHED,
	SM_ROI_TRACKER_INIT
    };

ros::ServiceClient srvCltRoiTrack;
int main(int argc, char **argv) {

	ros::init(argc, argv, "hybrid_follow_test");
	ros::NodeHandle n;
	srvCltRoiTrack = n.serviceClient<std_srvs::Trigger>("/vision/roi_tracker/init_track_inFront");
    		std_srvs::Trigger srv;
    
	STATE nextState = SM_WAIT_FOR_OPERATOR;
	JustinaHRI::setNodeHandle(&n);
    bool success = false;
    ros::Rate rate(10);
    std::string lastRecoSpeech;
    std::string stopRecog = "stop follow me";
    std::vector<std::string> validCommandsStop;
    validCommandsStop.push_back(stopRecog);

    while(ros::ok() && !success){

        switch(nextState){
            case SM_WAIT_FOR_OPERATOR:
                std::cout << "State machine: SM_WAIT_FOR_OPERATOR" << std::endl;
                JustinaHRI::waitAfterSay("Please, tell me, follow me for start following you", 3000);
                if(JustinaHRI::waitForSpecificSentence("follow me" , 15000))
                    nextState = SM_MEMORIZING_OPERATOR;
                else
                    nextState = SM_WAIT_FOR_OPERATOR;    		
            break;

            case SM_MEMORIZING_OPERATOR:
                std::cout << "State machine: SM_MEMORIZING_OPERATOR" << std::endl;
                JustinaHRI::waitAfterSay("Human, please put in front of me", 2500);
                //JustinaHRI::enableLegFinder(true);
                nextState=SM_WAIT_FOR_LEGS_FOUND;
            break;

            case SM_WAIT_FOR_LEGS_FOUND:
                std::cout << "State machine: SM_WAIT_FOR_LEGS_FOUND" << std::endl;
		
    		    /*if (srvCltRoiTrack.call(srv)){
			    std::cout << "TRUE ROI TRACK" << std::endl;
		        }
		        else
		        std::cout << "FALSE ROI TRACK" << std::endl;*/
		        //  JustinaHRI::initRoiTracker();	    
                

                //if(JustinaHRI::frontalLegsFound()){
                    std::cout << "NavigTest.->Frontal legs found!" << std::endl;
                    //JustinaHRI::startFollowHuman();
                    JustinaHRI::waitAfterSay("I found you, i will start to follow you human, please walk and tell me, stop follow me, when we reached the goal location", 10000);
                    nextState = SM_ROI_TRACKER_INIT;//SM_FOLLOWING_PHASE;
                //}
            break;

	        case SM_ROI_TRACKER_INIT:
		            //JustinaHRI::initRoiTracker();	
                    JustinaHRI::startHybridFollow();    
		            nextState = SM_FOLLOWING_PHASE;
            break;
			
            case SM_FOLLOWING_PHASE:
                std::cout << "State machine: SM_FOLLOWING_PHASE" << std::endl;
                if(JustinaHRI::waitForSpecificSentence(validCommandsStop, lastRecoSpeech, 7000)){
                    if(lastRecoSpeech.find(stopRecog) != std::string::npos){
                        //JustinaHRI::stopFollowHuman();
                        //JustinaHRI::enableLegFinder(false);
                        JustinaHRI::stopHybridFollow();
                        JustinaHRI::waitAfterSay("I stopped", 1500);
                        nextState = SM_FOLLOWING_FINISHED;
                        break;
                    }
                }
                /*if(!JustinaHRI::frontalLegsFound()){
                    std::cout << "State machine: SM_FOLLOWING_PHASE -> Lost human!" << std::endl;
                    JustinaHRI::waitAfterSay("I lost you", 1500);
                }*/        
            break;

            case SM_FOLLOWING_FINISHED:
                std::cout << "State machine: SM_FOLLOWING_FINISHED" << std::endl;
                JustinaHRI::waitAfterSay("I have finished following you", 3000);
                success = true;
            break;
        }

        rate.sleep();
        ros::spinOnce();
    }
    return success;


}
