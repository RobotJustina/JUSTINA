#include <iostream>
#include <stdlib.h>
#include <algorithm>
#include <vector>
#include "ros/ros.h"
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTasks.h"
#include "vision_msgs/VisionObject.h"
#include "vision_msgs/DetectObjects.h"

#define SM_INIT 0
#define SM_LOOK_IN_SHELVES 10
#define SM_FINAL_STATE 20

int main(int argc, char** argv)
{
    	std::cout << "INITIALIZING ACT_PLN BY MARCOSOFT..." << std::endl;
    	ros::init(argc, argv, "act_pln");
    	ros::NodeHandle n;
    	JustinaHardware::setNodeHandle(&n);
    	JustinaHRI::setNodeHandle(&n);
    	JustinaManip::setNodeHandle(&n);
    	JustinaNavigation::setNodeHandle(&n);
    	JustinaTools::setNodeHandle(&n);
    	JustinaVision::setNodeHandle(&n);
    	ros::Rate loop(10);
		//STATES
    	int nextState = 0;
    	bool fail = false;
    	bool success = false;
		//ITERATION VARIABLES FOR STATES
		int shelfCount=0;
		int numShelves=1;
		//HEAD MOVEMENTS
		int headAngles[1] = {-25};
		int headRotation[3] = {0, 20, -20}; // = {start, left, right};
		int headMovements = 3;
		float tempAng = 0;
		float tempAng2 = 0;
		//OBJECTS LIST
		std::vector<std::string> object;
		std::vector<float> xCoord;
		std::vector<float> yCoord;
		std::vector<float> zCoord;
		std::vector<std::string>::const_iterator toSearch;
		std::vector<vision_msgs::VisionObject> detectedObjects;
		std::string objId = "empty";
		float x = 0.0;
		float y = 0.0;
		float z = 0.0;
		//TIME
		float timeOutSpeech = 8000;
		float timeOutHead = 3000;
		//STRINGS
		std::string objfnd = "Object-found...";
		std::string shlfr = "I-am-still-searching-for-objects-at-my-righ-side...";
		std::string shlfl = "I-am-still-searching-for-objects-at-my-left-side...";

    	while(ros::ok() && !fail && !success)
    	{
        	switch(nextState)
        	{
        		case SM_INIT:
				std::cout << "Press any key to start this test... " << std::endl;
				std::cin.ignore();
				nextState = SM_LOOK_IN_SHELVES;
      			break;

		        case SM_LOOK_IN_SHELVES:
				tempAng=(headAngles[shelfCount]*3.1416)/180;
				JustinaManip::hdGoTo(0, tempAng, timeOutHead);
				///Vision///
				JustinaVision::startObjectFindingWindow();
				for(int j=0; j<headMovements; j++) //inicio de cabeza, varias vistas
				{
					tempAng2=(headRotation[j]*3.1416)/180;
					JustinaManip::hdGoTo(tempAng2,tempAng, timeOutHead);
					if(j==1 || j==4)//left
						JustinaHRI::say(shlfl);
					if(j==2 || j==5)//right
						JustinaHRI::say(shlfr);
					if(JustinaVision::detectObjects(detectedObjects)) //inicio de vista actual
					{
						for(int i=0; i<detectedObjects.size(); i++)
						{
							objId = detectedObjects[i].id;
							x = detectedObjects[i].pose.position.x;
							y = detectedObjects[i].pose.position.y;
							z = detectedObjects[i].pose.position.z;
							std::cout << i << " found" << objId << std::endl;
							std::cout << "x(" << x << ")y(" << y << ")z(" << z << ")" << std::endl;
							//Descartacion de objetos repetidos
							toSearch = find (object.begin(), object.end(), objId);
							if (toSearch != object.end()){//encontrado
								std::cout << objId  <<" found, but already on list" << std::endl;
							}else{ //no encontrado
								object.push_back(objId);
								xCoord.push_back(x);
								yCoord.push_back(y);
								zCoord.push_back(z);
							}
							//fin de descartacion
						}
					}//fin de vista actual
				}
				///Vision///
				JustinaVision::stopObjectFindingWindow();
				detectedObjects.empty();
                shelfCount++;
                if(shelfCount>=numShelves)
				{
					if(!object.empty()){
							std::cout << std::endl << "List of objects found... " << std::endl;
							for(int i=0; i<object.size(); i++){	//Objects Found
								objId=object[i];
								x=xCoord[i];
								y=yCoord[i];
								z=zCoord[i];
								std::cout << "(" << objId << "): x," << x << " y," << y << " z," << z << std::endl;
								JustinaHRI::say(objfnd);
								JustinaHRI::say(objId);
							}
						nextState = SM_FINAL_STATE;
					}
				}
				 else
					nextState = SM_LOOK_IN_SHELVES;
				break;


				case SM_FINAL_STATE:
					JustinaManip::hdGoTo(0, 0, timeOutHead);
					//manipulation vars clear
					success = true;
				break;
		}
        	ros::spinOnce();
	        loop.sleep();
	}
    	return 0;
}
