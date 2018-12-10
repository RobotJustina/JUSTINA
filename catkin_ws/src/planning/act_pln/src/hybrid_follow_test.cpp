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

vision_msgs::VisionFaceObjects myFaces;
bool recognized = false;
boost::posix_time::ptime curr;
boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
boost::posix_time::time_duration diff;



bool takePicture(vision_msgs::VisionFaceObjects &f)
{
    do{
        myFaces = JustinaVision::getFaces();
        if(myFaces.recog_faces.size()>0)
            recognized=true;
        else
            recognized=false;
        curr = boost::posix_time::second_clock::local_time();
        ros::spinOnce();
    }while(ros::ok() && (curr - prev).total_milliseconds()< 10000 && !recognized);

    return recognized;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "hybrid_follow_test");
	ros::NodeHandle n;
    
	STATE nextState = SM_WAIT_FOR_OPERATOR;
	JustinaHRI::setNodeHandle(&n);
    JustinaManip::setNodeHandle(&n);
    JustinaVision::setNodeHandle(&n);
    JustinaTasks::setNodeHandle(&n);
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
                JustinaManip::startHdGoTo(0.0, -0.25);
                JustinaHRI::waitAfterSay("Please, tell me, follow me for start following you", 3000);
                if(JustinaHRI::waitForSpecificSentence("follow me" , 15000))
                    nextState = SM_MEMORIZING_OPERATOR;
                else
                    nextState = SM_WAIT_FOR_OPERATOR;    		
            break;

            case SM_MEMORIZING_OPERATOR:
                std::cout << "State machine: SM_MEMORIZING_OPERATOR" << std::endl;
                JustinaHRI::waitAfterSay("Human, please put in front of me to take a picture of your face", 2500);

                /*if(!takePicture(myFaces))
                    if(!takePicture(myFaces))
                        takePicture(myFaces);


                JustinaHRI::waitAfterSay("thank you", 2500);

                if(!JustinaTasks::setRoi(myFaces))
                    if(!JustinaTasks::setRoi(myFaces))
                        JustinaTasks::setRoi(myFaces);
                    
                nextState = SM_WAIT_FOR_LEGS_FOUND;*/
                nextState = SM_ROI_TRACKER_INIT;
            break;

            case SM_WAIT_FOR_LEGS_FOUND:
                std::cout << "State machine: SM_WAIT_FOR_LEGS_FOUND" << std::endl;
		        JustinaHRI::waitAfterSay("now please, turn around to take a pattern of your back", 10000);
                ros::Duration(3.5).sleep();
    		    nextState = SM_ROI_TRACKER_INIT;//SM_FOLLOWING_PHASE;
                //}
            break;

	        case SM_ROI_TRACKER_INIT:
		            //JustinaHRI::initRoiTracker();	
                std::cout << "State machine: SM_ROI_TRACKER_INIT" << std::endl;
                //JustinaHRI::waitAfterSay("thank you, now please walk and tell me, stop follow me, when we reached the goal location", 10000); 
                JustinaHRI::startHybridFollow(); 
                ros::Duration(1.0).sleep();  
                JustinaHRI::waitAfterSay("thank you, now please walk and tell me, stop follow me, when we reached the goal location", 10000); 
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
