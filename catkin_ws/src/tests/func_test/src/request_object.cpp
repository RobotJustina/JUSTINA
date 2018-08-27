#include <iostream>
#include "justina_tools/JustinaTasks.h"

#define SM_INIT                     0
#define SM_WAIT_FOR_START_COMMAND   10
#define SM_CONFIRM_PETITION         15
#define SM_ALIGN_TO_TABLE           20
#define SM_FIND_OBJECT              25
#define SM_GRASP_OBJECT             30
#define SM_DROP_OBJECT              40

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING GRASP-TEST BY EDGAR-II..." << std::endl;
    ros::init(argc, argv, "act_pln");
    ros::NodeHandle n;

    JustinaTasks::setNodeHandle(&n);
    ros::Rate loop(10);

    int nextState;
    int indexFound = 0;
    bool found;
    bool isAlign;
    bool withLeftOrRightArm;
    bool fail = false; 
    bool success = false;

    geometry_msgs::Pose pose;

    std::vector<vision_msgs::VisionObject> recognizedObjects;
    std::vector<std::string> validCommands;
    std::string lastRecoSpeech;
    std::string confirmRecoSpeech;

    
    std::string idObject = "sugar";
    validCommands.push_back("bring me the pringles");
    validCommands.push_back("bring me the strawberry juice");
    validCommands.push_back("bring me the sugar");
    validCommands.push_back("bring me the chocolate milk");
    validCommands.push_back("robot yes");
    validCommands.push_back("robot no");

    
    nextState = SM_INIT;

    while(ros::ok() && !fail && !success){
        switch(nextState){
            case SM_INIT:
            {
                std::cout << "----->  State machine: INIT" << std::endl;
                JustinaHRI::say("I'm ready for atend your petition");
                boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

                nextState = SM_WAIT_FOR_START_COMMAND;
            }
            break;            

            case SM_WAIT_FOR_START_COMMAND:
            {
                std::cout << "" << std::endl;
                std::cout << "" << std::endl;
                std::cout << "----->  State machine: WAIT_FOR_START_COMMAND" << std::endl;
                if(!JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 15000))
                    JustinaHRI::say("Please repeat the command");
                else
                {
                    if(lastRecoSpeech.find("bring me the pringles") != std::string::npos)
                        idObject = "pringles";
                    else if(lastRecoSpeech.find("bring me the strawberry juice") != std::string::npos)
                        idObject = "juice";
                    else if(lastRecoSpeech.find("bring me the sugar") != std::string::npos)
                        idObject = "sugar";
                    else if(lastRecoSpeech.find("bring me the chocolate milk") != std::string::npos)
                        idObject = "milk";

                    nextState = SM_CONFIRM_PETITION;
            }
          }
          break;

          case SM_CONFIRM_PETITION:
            {
                std::cout << "" << std::endl;
                std::cout << "" << std::endl;
                std::cout << "----->  State machine: SM_CONFIRM_PETITION" << std::endl;
                JustinaHRI::say("Do you say  ");
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                JustinaHRI::say(lastRecoSpeech);
                boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
                if(!JustinaHRI::waitForSpecificSentence(validCommands, confirmRecoSpeech, 15000))
                    JustinaHRI::say("Please repeat the command");
                else
                {
                    if(confirmRecoSpeech.find("robot yes") != std::string::npos)
                        nextState = SM_ALIGN_TO_TABLE;
                    else
                        nextState = SM_INIT;
            }
          }
          break;

        case SM_ALIGN_TO_TABLE:
	  JustinaHRI::say("Ok, wait a moment.");
            isAlign = JustinaTasks::alignWithTable(0.42);
            std::cout << "Align With table " << std::endl;
            if(!isAlign){
                std::cout << "Can not align with table." << std::endl;
                nextState = SM_ALIGN_TO_TABLE;
            }
            else
                nextState = SM_FIND_OBJECT;
            break;

        case SM_FIND_OBJECT:
        	found = JustinaTasks::findObject(idObject, pose, withLeftOrRightArm);
            std::cout << "withLeftOrRightArm:" << withLeftOrRightArm << std::endl;
            if(!found){
                std::cout << "Not found a object" << std::endl;
                nextState = SM_FIND_OBJECT;
            }
            else{
                std::cout << "Found a object" << std::endl;
                nextState = SM_GRASP_OBJECT;
            }
            break;

        case SM_GRASP_OBJECT:
            JustinaTasks::moveActuatorToGrasp(pose.position.x, pose.position.y, pose.position.z, withLeftOrRightArm, idObject, true);
            //JustinaTasks::graspObjectFeedback(pose.position.x, pose.position.y, pose.position.z, withLeftOrRightArm, idObject, true);
            nextState = SM_DROP_OBJECT;
            break;

        case SM_DROP_OBJECT:
            JustinaNavigation::moveDistAngle(0.0, M_PI, 2000);
	    JustinaNavigation::moveDistAngle(0.60, 0.0, 2000);
            JustinaTasks::dropObject("", withLeftOrRightArm);
	    JustinaNavigation::moveDistAngle(0.0, -M_PI, 2000);
	    JustinaNavigation::moveDistAngle(0.20, 0.0, 2000);
	    nextState = SM_INIT;
            break;

        default:
            std::cout << ".->Somebody very stupid programmed this shit. " << std::endl;
            fail = true;
            break;
        }
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}
