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

#define SM_InitialState 0
#define	SM_InspectTheObjetcs 10
#define SM_TakeObject 20
#define	SM_FinalState 80

int main(int argc, char** argv)
{
	std::cout << "Initializing P & G Test..." << std::endl;
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
	
	bool fail = false;
	bool success = false;

  	//int nextState = SM_WaitBlindGame;
  	int nextState = 0;
  	
  	//set the KINECT as the input device 
  	JustinaHRI::setInputDevice(JustinaHRI::KINECT);

  	vision_msgs::CubesSegmented my_cutlery;
  	my_cutlery.recog_cubes.resize(5);

	my_cutlery.recog_cubes[0].color="red";
	my_cutlery.recog_cubes[1].color="green";
	my_cutlery.recog_cubes[2].color="blue";
	my_cutlery.recog_cubes[3].color="purple";
	my_cutlery.recog_cubes[4].color="yellow";

	bool cutlery_found = false;
	geometry_msgs::Pose pose;
	bool withLeftOrRightArm;
	std::string id_cutlery;



  	while(ros::ok() && !fail && !success)
  	{
		ros::Rate loop(10);
  		switch(nextState)
    	{

    		case SM_InitialState:
      			std::cout << "P & G Test...-> start the P & G test" << std::endl;
        		JustinaManip::startHdGoTo(0.0, 0.0);
        		JustinaHRI::say("I am ready for the Procter & Gamble test");
        		ros::Duration(2.0).sleep();
           		nextState = SM_InspectTheObjetcs;
      		break;

      		case SM_InspectTheObjetcs:
      			std::cout << "P & G Test...-> inspecting the objets on the table" << std::endl;
      			if(!JustinaTasks::alignWithTable(0.42)){
      				if(!JustinaTasks::alignWithTable(0.42)){
      					std::cout << "P & G Test...-> Can not align with table." << std::endl;
      					nextState = SM_InspectTheObjetcs;
      				}
      			}
      			else{
      				std::cout << "P & G Test...-> trying to detect the objects" << std::endl;
      				JustinaHRI::say("I am looking for an object on the table");
        			ros::Duration(2.0).sleep();
      				if(!JustinaVision::getCutlerySeg(my_cutlery)){
      					if(!JustinaVision::getCutlerySeg(my_cutlery)){
      						std::cout << "P & G Test...-> Can not detect any object" << std::endl;
      						nextState = SM_InspectTheObjetcs;
      					}
      				}
      				else{
      					std::cout << "P & G Test...-> selecting one object" << std::endl;
      					for(int i=0; i < my_cutlery.recog_cubes.size(); i ++){
      						if(my_cutlery.recog_cubes[i].detected_cube == true){
      							std::cout << "P & G Test...-> detect the " << my_cutlery.recog_cubes[i].color << " object" << std::endl;
      							pose.position.x = my_cutlery.recog_cubes[i].cube_centroid.x;
                				pose.position.y = my_cutlery.recog_cubes[i].cube_centroid.y;
                				pose.position.z = my_cutlery.recog_cubes[i].cube_centroid.z;
                				id_cutlery = my_cutlery.recog_cubes[i].color;
                				JustinaHRI::say("I've found an object on the table");
        						ros::Duration(2.0).sleep();
                				cutlery_found = true;
                				nextState = SM_TakeObject;
                				break;
      						}
						} 
      				}
      			}

      		break;

      		case SM_TakeObject:
      			std::cout << "P & G Test...-> taking objects" << std::endl;

      			if(pose.position.y > 0){
					withLeftOrRightArm = true;
					std::cout << "P & G Test...-> using  left arm" << std::endl;
					JustinaHRI::say("I am going to take an object with my left arm");
        			ros::Duration(2.0).sleep();
      			}
				else{
					withLeftOrRightArm = false;
					std::cout << "P & G Test...-> using  right arm" << std::endl;
					JustinaHRI::say("I am going to take an object with my right arm");
        			ros::Duration(2.0).sleep();
				}

				JustinaTasks::graspBlockFeedback(pose.position.x, pose.position.y, pose.position.z, withLeftOrRightArm, id_cutlery, true);
      			nextState = SM_FinalState;
      		break;

			case SM_FinalState:
				std::cout <<"P & G Test...->finalState reached" << std::endl;
				JustinaHRI::say("I have finished the procter & Gamble challenge");
				ros::Duration(2.0).sleep();
				success=true;
			break;

    }
    ros::spinOnce();
    loop.sleep();
  }
  return 0;
}
