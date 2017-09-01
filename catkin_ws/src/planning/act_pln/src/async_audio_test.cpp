#include <iostream>
#include <stdlib.h>
#include "ros/ros.h"
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaKnowledge.h"
#include "justina_tools/JustinaAudio.h"
#include "justina_tools/JustinaRepresentation.h"
#include "justina_tools/JustinaTasks.h"
#include "std_msgs/Bool.h"
#include "string"

JustinaHRI::Queue *tas;
#define SM_InitialState 0
#define	SM_WaitingandTurn 10
#define SM_StatingtheCrowd 20
#define SM_RequestingOperator 30
#define	SM_RiddleGame 40
#define SM_WaitBlindGame 50
#define SM_BlindGame 60
#define SM_BlindGameRepeatQ 70
#define	SM_FinalState 80


int main(int argc, char** argv)
{
	std::cout << "Initializing Speech and Person Recognition Test..." << std::endl;
  	ros::init(argc, argv, "act_pln");
  	ros::NodeHandle n;
  	JustinaHardware::setNodeHandle(&n);
  	JustinaHRI::setNodeHandle(&n);
  	JustinaManip::setNodeHandle(&n);
  	JustinaNavigation::setNodeHandle(&n);
  	JustinaTools::setNodeHandle(&n);
  	JustinaVision::setNodeHandle(&n);
	JustinaAudio::setNodeHandle(&n);
	JustinaRepresentation::setNodeHandle(&n);
	JustinaTasks::setNodeHandle(&n);
	JustinaKnowledge::setNodeHandle(&n);//knowledge
	std::stringstream auxAudio;
	std::string str1;

	JustinaRepresentation::initKDB("", true, 20000);

  	//ros::Rate loop(10);

	bool fail = false;
	bool success = false;

  	//int nextState = SM_WaitBlindGame;
  	int nextState = 0;
  	bool recog=false;
  	int numQuestion = 1;
  	std::string answer;
	std::stringstream ss;
	
	int contChances=0;
	str1 = "/home/biorobotica/Script/stop_arecord.sh ";

//	JustinaHRI::Queue *tas;
	std::string nombre;
	char numero;
    //	if((tas = (JustinaHRI::Queue*)malloc(sizeof(JustinaHRI::Queue)))==NULL)
      //  	return -1;
    
	//if((nombre = (std::string*)malloc(sizeof(std::string)*50)) == NULL)
        //	return -1;

	/*JustinaHRI::inicializa();
	JustinaHRI::push( "hello my name is Justina, I meet another robot, its name is robbie");
	JustinaHRI::push( "my team is pumas, since 2011, i am very happy");
	JustinaHRI::push( "of the university of mexico");
	JustinaHRI::push( "I have two arms, how many arms do you have");*/
	//JustinaHRI::pop();
	


  	while(ros::ok() && !fail && !success)
  	{
		ros::Rate rate(10);
  		switch(nextState)
    	{
		case SM_InitialState:
			//JustinaHRI::inicializa();
			JustinaHRI::push( "i am ready for the speech and person recognition test");
			JustinaHRI::push( "i want to play a ridle game");
			JustinaHRI::pop();
			JustinaHardware::setHeadGoalPose(0.0, 0.0);
			nextState = SM_WaitingandTurn;
		break;

    		case SM_WaitingandTurn:
			JustinaHRI::push("i am turning arround to find you");
			JustinaHRI::push("i am moving my head to find you");
			JustinaHRI::pop();
        		JustinaNavigation::moveDistAngle(0.0, 3.141592, 5000);
			JustinaManip::startHdGoTo(0.0, -0.15);
			JustinaManip::startHdGoTo(0.0, -.9);
			JustinaManip::startHdGoTo(0.0, -0.15);
        		nextState = SM_InitialState;
      		break;
	}
		rate.sleep();
		ros::spinOnce();
	}
}
