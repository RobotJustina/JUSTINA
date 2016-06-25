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
#include "vision_msgs/VisionObject.h"
#include "vision_msgs/DetectObjects.h"

#define SM_INIT 0
#define SM_WAIT_INIT 10
#define SM_SETUP 20
#define SM_WAIT_FOR_COMMAND 30
#define SM_ASK_REPEAT_COMMAND 40
#define SM_PARSE_SPOKEN_COMMAND 50
#define SM_FIND_BOOKCASE 60
#define SM_NAVIGATE_TO_BOOKCASE 70
#define SM_WAITING_TO_BOOKCASE 80
#define SM_LOOK_IN_SHELVES 90
#define SM_FINAL_REPORT 100
#define SM_FINAL_STATE 110

void fullReport(std::string fl, std::string theString){
	std::string jst = "Justina says: ";
	std::stringstream ss;
	std::cout << theString << std::endl;
        ss << jst << theString;
        JustinaTools::pdfAppend(fl,ss.str());
        ss.str(std::string());
        ss.clear();
        JustinaHRI::say(theString);
	sleep(3);
}

void fullReport(std::string fl, int theString){
        std::string jst = "Justina says: ";
        std::stringstream ss;
        std::cout << theString << std::endl;
        ss << jst;
	JustinaHRI::say(ss.str());
	ss << theString;
        JustinaTools::pdfAppend(fl,ss.str());
        ss.str(std::string());
        ss.clear();
        sleep(3);
}

void fullReport(std::string fl, float theString){
        std::string jst = "Justina says: ";
        std::stringstream ss;
        std::cout << theString << std::endl;
        ss << jst;
        JustinaHRI::say(ss.str());
        ss << theString;
        JustinaTools::pdfAppend(fl,ss.str());
        ss.str(std::string());
        ss.clear();
        sleep(3);
}

void writeReport(std::string fl, std::string theString){
        std::cout << theString << std::endl;
	JustinaTools::pdfAppend(fl,theString);
}

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
	int objectCount=0;
	//NUMBER OF SHELVES
	int numShelves=4; //starts on 0
	//PRE-DEFINED HEAD ANGLES
	int headAngles[5] = {-25, -30, -45, -50, -55};
	int headRotation[5] = {0, 45, -45, 70, -70}; // = {start, left, right};
	int headMovements = 3;
	float tempAng = 0;
	float tempAng2 = 0;
	//PRE-DEFINED ROBOT HEIGHT CENTIMETERS
	float height[5] = {0, 17, 0, -0.1, 0};
	int startTorso=0.13;
	//OBJECTS LIST
	std::string imgPath = "/home/$USER/Pictures/";
	std::string testName = "Object recognition and manipulation test";
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
	int maxOb[5]= {0}; //1 container per shelve, number of objects found
	int currentMaxOb = 0;
	//ARMS MOVEMENT
	//initial (from camera)
	float xi = 0;
	float yi = 0;
	float zi = 0;
	//final (to Arm)
	float xf = 0;
	float yf = 0;
	float zf = 0;
	float roll = 0;
	float pitch = 0;
	float yaw = 0;
	float elbow = 0;
	//time
	float timeOutArm = 20;
	//NAVIGATION
	std::string location = "coffe_table";
	float timeOutMove = 73489;
	//SPEECH
	std::string okCmd = "start";
    	std::string lastRecoSpeech;
    	std::vector<std::string> validCommands;
    	validCommands.push_back(okCmd);
	//TIME
	float timeOutSpeech = 9000;
	float timeOutHead = 5000;
	float timeOutTorso = 2000;
	//STRINGS
	std::string fl = "ManipAndObjectReco_Plans";
	std::string init0 = "Initializing justina nodes...";
	std::string tors0 = "Sending justina to zero height...";
	std::string speak0 = "I am ready to manipulation object test...";
	std::string wait4ord = "I am waiting for the  command...";
	std::string rptcmd = "Please repeat the command...";
	std::string strtst = "I will now start the object recognition test...";
	std::string shlf = "I am gonna navigate to the shelves...";
	std::string shlfr = "I am still searching for objects at my right side...";
	std::string shlfl = "I am still searching for objects at my left side...";
	std::string objfnd = "Object found...";
	std::string hgtrch = "I will reach the shelve number...";
	std::string torsmv = "I am going to move my height...";
	std::string fnladv = "I can not grab the object...";
	std::string eot = "end of the test reached...";
        std::stringstream ss;

    	while(ros::ok() && !fail && !success)
    	{
        	switch(nextState)
        	{
        		case SM_INIT:
				std::cout << "Press any key to start this test... " << std::endl;
				std::cin.ignore();
				JustinaTools::pdfStart(fl);
				writeReport(fl,init0);
				nextState = SM_WAIT_INIT;
            			break;

                        case SM_WAIT_INIT:
                                if(!JustinaManip::torsoGoTo(startTorso, 0, 0, timeOutTorso)){
					writeReport(fl,tors0);
                                        nextState = SM_WAIT_INIT;
				}
                          	else
                                        nextState = SM_SETUP;
                                break;

                        case SM_SETUP:
				fullReport(fl,speak0);
                                nextState = SM_WAIT_FOR_COMMAND;
                                break;

        		case SM_WAIT_FOR_COMMAND:
                                fullReport(fl,wait4ord);
            			if(!JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, timeOutSpeech))
                			nextState = SM_ASK_REPEAT_COMMAND;
            			else
                			nextState = SM_PARSE_SPOKEN_COMMAND;
            			break;

        		case SM_ASK_REPEAT_COMMAND:
				fullReport(fl,rptcmd);
            			nextState = SM_WAIT_FOR_COMMAND;
			        break;

		        case SM_PARSE_SPOKEN_COMMAND:
            			if(lastRecoSpeech.find(okCmd) != std::string::npos){
					fullReport(fl,strtst);
					fullReport(fl,shlf);
					nextState = SM_NAVIGATE_TO_BOOKCASE;
				}
	            		break;

		        case SM_NAVIGATE_TO_BOOKCASE:
				if(JustinaNavigation::getClose(location,timeOutMove))
	                		nextState = SM_LOOK_IN_SHELVES;
				else
					nextState = SM_WAITING_TO_BOOKCASE;
            			break;

		        case SM_WAITING_TO_BOOKCASE:
				JustinaVision::startObjectFinding();
				nextState = SM_NAVIGATE_TO_BOOKCASE;
            			break;

		        case SM_LOOK_IN_SHELVES:
				tempAng=(headAngles[shelfCount]*3.1416)/180;
				JustinaManip::hdGoTo(0, tempAng, timeOutHead);
				height[shelfCount]=height[shelfCount]/100;
				fullReport(fl,hgtrch);
				fullReport(fl,shelfCount);
				sleep(3);
				JustinaManip::torsoGoToRel(height[shelfCount], 0, 0, timeOutTorso);
				fullReport(fl,torsmv);
				fullReport(fl,height[shelfCount]);
                                sleep(4);
				///Vision///
				JustinaVision::startObjectFindingWindow();
				for(int j=0; j<headMovements; j++) //inicio de cabeza, varias vistas
				{
					tempAng2=(headRotation[j]*3.1416)/180;
					JustinaManip::hdGoTo(tempAng2,tempAng, timeOutHead);
					if(j==1 || j==4)//left
						fullReport(fl,shlfl);
					if(j==2 || j==5)//right
						fullReport(fl,shlfr);
					fullReport(fl,tempAng2);
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
								std::cout << objId  <<" found, but already seen" << std::endl;
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
				//Manipulacion de objeto mas cercano y reporte
				std::cout << std::endl << "List of founded objects... " << std::endl;
				for(int i=0; i<detectedObjects.size(); i++){
					objId=object[i];
                                        x=xCoord[i];
                                        y=yCoord[i];
                                        z=zCoord[i];
					std::cout << "(" << objId << "): " << x << " " << y << " " << z << std::endl;
				}
                                fullReport(fl,objfnd);
                                sleep(3);
                                fullReport(fl,objId);
				//fin de manipulacion
				object.clear();
				xCoord.clear();
				yCoord.clear();
				zCoord.clear();
				shelfCount++;
				if(shelfCount>=numShelves)
					nextState = SM_FINAL_REPORT;
				 else
					nextState = SM_LOOK_IN_SHELVES;
				break;
/*
			case SM_GRAB_OBJECTS:
				if(currentMaxOb>0){
	//				JustinaTools::transformFromPoint(src,xi,yi,xi,dst,xf,yf,zf);
	//				JustinaManip::laGoToCartesian(xf, yf, zf, roll, pitch, yaw, elbow, timeOutArm);
	//				JustinaHardware::laCloseGripper(-0.8);
					objectCount++;
					if(objectCount>currentMaxOb)
						objectCount=0;
					else
						nextState = SM_GRAB_OBJECTS;
				}
				nextState = SM_LOOK_IN_SHELVES;
				break;

*/			case SM_FINAL_REPORT:
				JustinaVision::stopObjectFinding();
				JustinaTools::pdfImageExport(testName,imgPath);
				nextState = SM_FINAL_STATE;
				break;

			case SM_FINAL_STATE:
				fullReport(fl,fnladv);
				fullReport(fl,eot);
				JustinaTools::pdfStop(fl);
				success = true;
				break;
        	}
        	ros::spinOnce();
	        loop.sleep();
	}
    	return 0;
}
