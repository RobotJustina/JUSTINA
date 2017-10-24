#include <iostream>
#include "ros/ros.h"
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaKnowledge.h"
#include "std_msgs/Bool.h"
#include "string"

#define SM_INIT 0
#define SM_WAIT_FOR_INIT_COMMAND 10
#define SM_TRAINING_PHASE 20
#define SM_RETRAINING_PHASE 21
#define SM_FOLLOWING_PHASE 30
#define SM_FOLLOWING_PAUSE 40
#define SM_FOLLOWING_CHECKPOINT 50
#define SM_FOLLOWING_GOALPOINT 60
#define SM_RETURN_HOME_COMMAND 70
#define SM_RETURN_HOME 80
#define SM_RETURN_CHECKPOINT_3 90
#define SM_RETURN_CHECKPOINT_2 100
#define SM_RETURN_CHECKPOINT_1 110
#define SM_FINAL_STATE 200
#define SM_WAIT_FOR_LEGS_FOUND 120

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING ACT_PLN-FOLLOW ME BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "act_pln");
    ros::NodeHandle n;
    JustinaHardware::setNodeHandle(&n);
    JustinaHRI::setNodeHandle(&n);
    JustinaManip::setNodeHandle(&n);
    JustinaNavigation::setNodeHandle(&n);
    JustinaTools::setNodeHandle(&n);
    JustinaVision::setNodeHandle(&n);
    JustinaKnowledge::setNodeHandle(&n);
    ros::Rate loop(10);

    int c_point=0,i=1;
    int nextState = 0;
    bool fail = false;
    bool success = false;
    bool stop=false;
    std::string lastRecoSpeech;
    std::vector<std::string> validCommands;
    validCommands.push_back("robot start");
    validCommands.push_back("stop follow me");
    validCommands.push_back("continue");
    validCommands.push_back("checkpoint");
    validCommands.push_back("goal");
    validCommands.push_back("return home");
    //validCommands.push_back("help me");
    validCommands.push_back("robot no");

    ros::Publisher pubFollow = n.advertise<std_msgs::Bool>("/hri/human_following/start_follow",1); 
    std_msgs::Bool startFollow;


    while(ros::ok() && !fail && !success)
    {
        switch(nextState)
        {

            case SM_INIT:
                {
                    std::cout << "State machine: SM_INIT" << std::endl;	
                    /*JustinaHRI::say("I'm ready for the follow me test");
                      sleep(2);
                      JustinaHRI::say("You may tell me one of the following commands:");
                      sleep(1);
                      JustinaHRI::say("To start the test, robot start");
                      sleep(1); 
                      JustinaHRI::say("To indicate any checkpoint, checkpoint");
                      sleep(1);
                      JustinaHRI::say("To indicate goal location, goal");
                      sleep(1);	
                      JustinaHRI::say("To start guiding, return home");
                    //JustinaHRI::say("You can tell me one of the next commands: robot start, stop follow me, continue , checkpoint, goal, return to home, help me");
                    sleep(2);
                    */
                    JustinaHRI::say("I'm waiting for the start command");
                    JustinaKnowledge::addUpdateKnownLoc("arena", -0.5, 0);
                    nextState = SM_WAIT_FOR_INIT_COMMAND;
                }
                break;

            case SM_WAIT_FOR_INIT_COMMAND:
                {
                    std::cout << "State machine: SM_WAIT_FOR_INIT_COMMAND" << std::endl;
                    if(!JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 15000))
                        JustinaHRI::say("Please repeat the command");
                    else{
                        if(lastRecoSpeech.find("robot start") != std::string::npos)
                            nextState = SM_TRAINING_PHASE;
                        else
                            nextState = SM_WAIT_FOR_INIT_COMMAND;    		
                    }
                }
                break;

            case SM_TRAINING_PHASE:
                {
                    std::cout << "State machine: SM_TRAINING_PHASE" << std::endl;
                    JustinaHRI::say("Human, please put in front of me");
                    JustinaHRI::enableLegFinder(true);
                    nextState=SM_WAIT_FOR_LEGS_FOUND;	    
                }
                break;

            case SM_RETRAINING_PHASE:
                {
                    std::cout << "State machine: SM_RETRAINING_PHASE" << std::endl;
                    JustinaHRI::say("Human, please put in front of me, again");
                    JustinaHRI::enableLegFinder(true);
                    nextState=SM_WAIT_FOR_LEGS_FOUND;	    
                    break;
                }

            case SM_WAIT_FOR_LEGS_FOUND:
                {
                    std::cout << "State machine: SM_WAIT_FOR_LEGS_FOUND" << std::endl;
                    if(JustinaHRI::frontalLegsFound())
                    {
                        std::cout << "NavigTest.->Frontal legs found!" << std::endl;
                        //JustinaHRI::say("You can tell me one of the next commands: stop follow me, continue, checkpoint, goal");
                        JustinaHRI::say("I found you");
                        sleep(1);	
                        JustinaHRI::say("I will start to follow you human, please walk");
                        nextState = SM_FOLLOWING_PHASE;
                    }
                }    

                break;

            case SM_FOLLOWING_PHASE:
                {
                    std::cout << "State machine: SM_FOLLOWING_PHASE" << std::endl;
                    stop=false;
                    JustinaHRI::startFollowHuman();
                    ros::spinOnce();

                    while(!stop && ros::ok()){
                        if(!JustinaHRI::frontalLegsFound()){
                            JustinaHRI::say("I lost you");
                            JustinaHRI::enableLegFinder(false);
                            nextState = SM_RETRAINING_PHASE;
                            std::cout << "Te perdí" << std::endl;
                            break;
                        }

                        if(JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 7000)){
                            if(lastRecoSpeech.find("stop follow me") != std::string::npos){
                                stop=true;
                                JustinaHRI::stopFollowHuman();
                                JustinaHRI::say("I stopped");
                                sleep(1);
                                //JustinaHRI::say("I'm waiting for the continue commnad");
                                nextState = SM_FOLLOWING_PAUSE;

                            }
                            else if(lastRecoSpeech.find("checkpoint") != std::string::npos){
                                stop=true;
                                nextState = SM_FOLLOWING_CHECKPOINT;
                            }
                            else if(lastRecoSpeech.find("goal") != std::string::npos  && i>3){
                                stop=true;
                                nextState = SM_FOLLOWING_GOALPOINT;					
                            }
                            /*							else if(lastRecoSpeech.find("help me") != std::string::npos  && i>3){
                                                        JustinaHRI::say("You can tell me one of the next commands: stop follow me, continue, checkpoint, goal, help me, return home");					
                                                        }
                                                        */							else{
                                                            std::cout << "Command ERROR!" << std::endl;
                                                            JustinaHRI::say("Please repeat the command");
                                                        }
                        }
                    }


                    }
                    break;


                    case SM_FOLLOWING_PAUSE:
                    {
                        std::cout << "State machine: SM_FOLLOWING_PAUSE" << std::endl;
                        /*		if (i==1){	
                                JustinaHRI::say("I saved the checkpoint 1");
                                JustinaNavigation::addLocation("checkpoint_1");	
                                i++;					
                                }
                                else if (i==2){                            
                                JustinaHRI::say("I saved the checkpoint 2");
                                JustinaNavigation::addLocation("checkpoint_2");
                                i++;				
                                }
                                else if (i==3){                     
                                JustinaHRI::say("I saved the checkpoint 3");
                                JustinaNavigation::addLocation("checkpoint_3");
                                i++;				
                                }

                                JustinaHRI::say("Do you want continue?");
                                */		stop=false;
                        while(!stop && ros::ok()){
                            if(JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 7000)){
                                if(lastRecoSpeech.find("continue") != std::string::npos){
                                    std::cout << "Command CONTINUE!" << std::endl;
                                    stop=true;
                                    nextState = SM_FOLLOWING_PHASE;
                                    JustinaHRI::say("OK");
                                }

                                else if(lastRecoSpeech.find("robot no") != std::string::npos){
                                    std::cout << "Command ROBOT NO!" << std::endl;
                                    stop=true;
                                    nextState = SM_RETURN_HOME_COMMAND;
                                    JustinaHRI::say("OK");
                                    JustinaKnowledge::addUpdateKnownLoc("goal_point");
                                    JustinaHRI::say("I saved the goal location");
                                    sleep(2);
                                    JustinaHRI::say("I'm waiting the command to guiding you to back home ");
                                }

                                else{
                                    std::cout << "Command ERROR!" << std::endl;
                                    JustinaHRI::say("Please repeat the command");
                                }
                            }
                        }
                    }
                    break;

                    case SM_FOLLOWING_CHECKPOINT:
                    {         
                        std::cout << "State machine: SM_FOLLOWING_CHECKPOINT" << std::endl;
                        if (i==1){	
                            JustinaHRI::say("I saved the checkpoint 1");
                            JustinaKnowledge::addUpdateKnownLoc("checkpoint_1");
                            i++;
                            //std::cout << system("rosrun map_server map_saver -f ~/JUSTINA/catkin_ws/src/planning/knowledge/navigation/occupancy_grids/Floor_FollowMe") << std::endl;

                        }
                        else if (i==2){                            
                            JustinaHRI::say("I saved the checkpoint 2");
                            JustinaKnowledge::addUpdateKnownLoc("checkpoint_2");
                            i++;
                            //std::cout << system("rosrun map_server map_saver -f ~/JUSTINA/catkin_ws/src/planning/knowledge/navigation/occupancy_grids/Floor_FollowMe") << std::endl;

                        }
                        else if (i==3){                     
                            JustinaHRI::say("I saved the checkpoint 3");
                            JustinaKnowledge::addUpdateKnownLoc("checkpoint_3");
                            i++;
                            //std::cout << system("rosrun map_server map_saver -f ~/JUSTINA/catkin_ws/src/planning/knowledge/navigation/occupancy_grids/Floor_FollowMe") << std::endl;

                        }		

                        nextState = SM_FOLLOWING_PHASE;
                    }               
                    break;

                    case SM_FOLLOWING_GOALPOINT:
                    {
                        std::cout << "State machine: SM_FOLLOWING_GOALPOINT" << std::endl;
                        JustinaHRI::stopFollowHuman();
                        JustinaKnowledge::addUpdateKnownLoc("goal_point");
                        JustinaHRI::say("I saved the goal location");
                        sleep(1);
                        JustinaHRI::say("I waiting for the command to return the initial point");
                        //std::cout << system("rosrun map_server map_saver -f ~/JUSTINA/catkin_ws/src/planning/knowledge/navigation/occupancy_grids/Floor_FollowMe") << std::endl;
                        nextState = SM_RETURN_HOME_COMMAND;
                    }
                    break;

                    case SM_RETURN_HOME_COMMAND:
                    {

                        if(JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 15000))
                        {
                            if(lastRecoSpeech.find("return home") != std::string::npos)
                                //JustinaHRI::say("I'm sorry human, i can't guiding you");
                                if (i==4)
                                    nextState = SM_RETURN_CHECKPOINT_3;
                                else if (i==3)
                                    nextState = SM_RETURN_CHECKPOINT_2;
                                else if (i==2)
                                    nextState = SM_RETURN_CHECKPOINT_1;
                                else if (i==1)
                                    nextState = SM_RETURN_HOME;
                        }
                        else{
                            JustinaHRI::say("Please reapeat the commnad");
                            sleep(2);	
                        }
                    }
                    break;

                    case SM_RETURN_CHECKPOINT_3:
                    {
                        JustinaHRI::say("I will go to checkpoint 3");
                        if(!JustinaNavigation::getClose("checkpoint_3",200000))
                            if(!JustinaNavigation::getClose("checkpoint_3",200000))
                                JustinaNavigation::getClose("checkpoint_3",200000);
                        JustinaHRI::say("I arrived to checkpoint 3");
                        nextState=SM_RETURN_CHECKPOINT_2;
                    }
                    break;

                    case SM_RETURN_CHECKPOINT_2:
                    {
                        JustinaHRI::say("I will go to checkpoint 2");	
                        if(!JustinaNavigation::getClose("checkpoint_2",200000))
                            if(!JustinaNavigation::getClose("checkpoint_2",200000))
                                JustinaNavigation::getClose("checkpoint_2",200000);
                        JustinaHRI::say("I arrived to checkpoint 2");
                        nextState=SM_RETURN_CHECKPOINT_1; 		
                    }
                    break;

                    case SM_RETURN_CHECKPOINT_1:
                    {
                        JustinaHRI::say("I will go to checkpoint 1");	
                        if(!JustinaNavigation::getClose("checkpoint_1",200000))
                            if(!JustinaNavigation::getClose("checkpoint_1",200000))
                                JustinaNavigation::getClose("checkpoint_1",200000);
                        JustinaHRI::say("I arrived to checkpoint 1");
                        nextState=SM_RETURN_HOME;
                    }
                    break;

                    case SM_RETURN_HOME:		
                    {
                        JustinaHRI::say("I will go to initial point");
                        if(!JustinaNavigation::getClose("arena",200000))
                            if(!JustinaNavigation::getClose("arena",200000))
                                JustinaNavigation::getClose("arena",200000);
                        JustinaHRI::say("I arrived to initial point");
                        nextState=SM_FINAL_STATE;
                        JustinaHRI::say("I finish the test");	
                    }
                    break;

                    case SM_FINAL_STATE:
                    {
                        std::cout << "State machine: SM_FINAL_STATE" << std::endl;

                    }    

                }
                ros::spinOnce();
                loop.sleep();
        }

        return 0;
    }




