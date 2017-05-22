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
#define SM_WAIT_INIT 10
#define SM_SETUP 20
#define SM_WAIT_FOR_COMMAND 30
#define SM_ASK_REPEAT_COMMAND 40
#define SM_PARSE_SPOKEN_COMMAND 50
#define SM_FIND_BOOKCASE 60
#define SM_NAVIGATE_TO_BOOKCASE 70
#define SM_WAITING_TO_BOOKCASE 80
#define SM_LOOK_IN_SHELVES 90
#define SM_GRAB_OBJECTS 100
#define SM_FINAL_STATE 110
#define SM_WAITING 120
#define SM_CRAZY_STUFF 130

void fullReport(std::string fl, std::string theString){
	std::string jst = "Justina-Says...";
	std::stringstream ss;
	std::cout << theString << std::endl;
        ss << jst;
	ss << theString;
        JustinaTools::pdfAppend(fl,ss.str());
        ss.str(std::string());
        ss.clear();
        JustinaHRI::say(theString);
}

void fullReport(std::string fl, int theString){
        std::string jst = "Justina-Says...";
        std::stringstream ss;
        std::cout << theString << std::endl;
        ss << jst;
	ss << theString;
        JustinaTools::pdfAppend(fl,ss.str());
        ss.str(std::string());
        ss.clear();
	ss << theString;
	JustinaHRI::say(ss.str());
        ss.str(std::string());
        ss.clear();
}

void fullReport(std::string fl, float theString){
        std::string jst = "Justina-Says...";
        std::stringstream ss;
        std::cout << theString << std::endl;
        ss << jst;
        ss << theString;
        JustinaTools::pdfAppend(fl,ss.str());
        ss.str(std::string());
        ss.clear();
        JustinaHRI::say(ss.str());
        ss.str(std::string());
        ss.clear();
}

void writeReport(std::string fl, std::string theAction, std::string theString, std::string theAmount){
        std::string jst = "Justina-";
        std::stringstream ss;
	ss << jst << theAction << theString << theAmount;
        std::cout << ss.str() << std::endl;
	JustinaTools::pdfAppend(fl,ss.str());
        ss.str(std::string());
        ss.clear();
	sleep(1);
}

void writeReport(std::string fl, std::string theAction, float theString, std::string theAmount){
        std::string jst = "Justina-";
        std::stringstream ss;
        ss << jst << theAction << theString << theAmount;
        std::cout << ss.str() << std::endl;
        JustinaTools::pdfAppend(fl,ss.str());
        ss.str(std::string());
        ss.clear();
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
	int numShelves=3; //starts on 0
	//PRE-DEFINED HEAD ANGLES
	//height head displacement
	//0 0.9 -15cm backwards
	//0 0.9 -15cm backwards
	//0.25 0.9
	//0.43 0.8
	int headAngles[5] = {-25, -45, -55, -65, -75};
	int headRotation[5] = {0, 20, -20, 30, -30}; // = {start, left, right};
	int headMovements = 3;
	float tempAng = 0;
	float tempAng2 = 0;
	//PRE-DEFINED ROBOT HEIGHT CENTIMETERS
	float height[5] = {30, 30, 20, 10, 0}; //relatives
	int startTorso=30;
	//OBJECTS LIST
	std::vector<std::string> knownObjects;
	knownObjects.push_back("baby-sweets");
	knownObjects.push_back("egg");
	knownObjects.push_back("shampoo");
	knownObjects.push_back("coke");
	knownObjects.push_back("sponge");
	std::string imgPath = "/home/$USER/objs/";
	std::string testName = "ObjectRecognitionAndManipulationTest";
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
	std::string safePose = "navigation";
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
	int trys = 0;
	float timeOutArm = 20;
	//NAVIGATION
	std::string location = "shelf";
	float timeOutMove = 73489;
	//SPEECH
	std::string okCmd = "start";
    	std::string lastRecoSpeech;
    	std::vector<std::string> validCommands;
    	validCommands.push_back(okCmd);
	//TIME
	float timeOutSpeech = 8000;
	float timeOutHead = 3000;
	float timeOutArms = 700;
	float timeOutTorso = 2000;
	//STRINGS
	std::string fl = "ManipAndObjectRecoPlans";
	std::string init0 = "Initializing-Justina-Nodes...";
	std::string tors0 = "Sending-justina-to-zero-height...";
	std::string speak0 = "I-am-ready-to-manipulation-object-test...";
	std::string wait4ord = "I-am-waiting-for-the-command...";
	std::string rptcmd = "Please-repeat-the-command...";
	std::string strtst = "I-will-now-start-the-object-recognition-test...";
	std::string shlf = "I-am-gonna-navigate-to-the-shelves...";
	std::string shlfr = "I-am-still-searching-for-objects-at-my-righ-side...";
	std::string shlfl = "I-am-still-searching-for-objects-at-my-left-side...";
	std::string objfnd = "Object-found...";
	std::string hgtrch = "I-will-reach-the-shelve-number...";
	std::string torsmv = "I-am-going-to-change-my-height...";
	std::string fnladv = "I-can-not-grab-the-object...";
	std::string eot = "End-of-the-test-reached...";
	std::string cm = "-meters-over-0-reference";
	std::string db = "-from the database";
	std::string dg = "-radians-over-0-reference";
	std::string srvs = "-service";
	std::string nde = "-node";
	std::string lct = "-location";
	std::string rcg = "recognize-the-word-";
	std::string xtr = "moves-the-torso-to-";
	std::string xhdx = "moves-the-head-on-x-Axis-to-";
        std::string xhdy = "moves-the-head-on-y-Axis-to-";
	std::string strt = "start-the-";
	std::string rqst = "request-to-";
	std::string stp = "stop-the-";
	std::string mtp = "execute-the-motion-planning-to-";
        std::stringstream ss;

    	while(ros::ok() && !fail && !success)
    	{
        	switch(nextState)
        	{
        		case SM_INIT:
				std::cout << "Press any key to start this test... " << std::endl;
				std::cin.ignore();
				JustinaTools::pdfStart(fl);
				writeReport(fl,init0,"","");
				nextState = SM_WAIT_INIT;
            			break;

                        case SM_WAIT_INIT:
				JustinaManip::torsoGoTo(startTorso, 0, 0, timeOutTorso);
				sleep(4);
				writeReport(fl,tors0,"","");
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
					writeReport(fl,mtp,location,lct);
					fullReport(fl,strtst);
					fullReport(fl,shlf);
					writeReport(fl,rcg,okCmd,db);
					JustinaNavigation::moveDist(0.45,3000);
					//nextState = SM_NAVIGATE_TO_BOOKCASE;
					nextState = SM_CRAZY_STUFF;
				}
	            		break;

		        case SM_NAVIGATE_TO_BOOKCASE:
				//if(JustinaNavigation::getClose(location,timeOutMove)){
				//if(JustinaTasks::alignWithTable()){
				if(JustinaNavigation::getClose(location,timeOutMove)){
					writeReport(fl,strt,"ObjectFinding",srvs);
					JustinaVision::startObjectFinding();
	                		nextState = SM_CRAZY_STUFF;
				}
				else
					nextState = SM_WAITING_TO_BOOKCASE;
            			break;


			case SM_CRAZY_STUFF:
				if(JustinaTasks::alignWithTable())
						nextState = SM_LOOK_IN_SHELVES;
				else{
                                        trys++;
                                        std::cout << "*try: " << trys << std::endl << std::endl;
                                        if(trys>3)
                                                nextState = SM_WAITING;
                                        else
                                                nextState = SM_LOOK_IN_SHELVES;
				}
				break;

                        case SM_WAITING:
                                nextState = SM_CRAZY_STUFF;
                                break;

		        case SM_WAITING_TO_BOOKCASE:
				nextState = SM_NAVIGATE_TO_BOOKCASE;
            			break;

		        case SM_LOOK_IN_SHELVES:
				tempAng=(headAngles[shelfCount]*3.1416)/180;
				JustinaManip::hdGoTo(0, tempAng, timeOutHead);
				height[shelfCount]=height[shelfCount]/100.0;
				fullReport(fl,hgtrch);
				fullReport(fl,shelfCount);
				writeReport(fl,xhdy,tempAng,dg);
				//sleep(3);
				JustinaManip::torsoGoTo(height[shelfCount], 0, 0, timeOutTorso);
				fullReport(fl,torsmv);
				writeReport(fl,xtr,height[shelfCount],cm);
				//sleep(4);
				///Vision///
				writeReport(fl,strt,"ObjectFindingWindow",srvs);
				JustinaVision::startObjectFindingWindow();
				for(int j=0; j<headMovements; j++) //inicio de cabeza, varias vistas
				{
					tempAng2=(headRotation[j]*3.1416)/180;
					JustinaManip::hdGoTo(tempAng2,tempAng, timeOutHead);
					if(j==1 || j==4)//left
						fullReport(fl,shlfl);
					if(j==2 || j==5)//right
						fullReport(fl,shlfr);
					writeReport(fl,xhdx,tempAng2,dg);
					writeReport(fl,rqst,"detectObjects",nde);
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
				writeReport(fl,stp,"ObjectFindingWindow",srvs);
				JustinaVision::stopObjectFindingWindow();
				detectedObjects.empty();
                                shelfCount++;
				if(shelfCount>=numShelves-1){
					JustinaManip::laGoTo(safePose,timeOutArms);
                                	JustinaManip::raGoTo(safePose,timeOutArms);
				}
                                if(shelfCount>=numShelves)
				{
                                        nextState = SM_GRAB_OBJECTS;
					if(!object.empty()){
	                                	//Objects filter
		                                std::cout << std::endl << "List of objects found... " << std::endl;
        		                        for(int i=0; i<object.size(); i++){	//Objects Found
                		                        objId=object[i];
                        		                x=xCoord[i];
                                		        y=yCoord[i];
                                        		z=zCoord[i];
	                                        	std::cout << "(" << objId << "): x," << x << " y," << y << " z," << z << std::endl;
	                                              	fullReport(fl,objfnd);
        	                                        fullReport(fl,objId);
						}
					/*//Objects on set
					for(std::vector<int>::size_type it = 0; it !=  object.size(); it++ ) {
						for(std::vector<int>::size_type jt = 0 ; jt != knownObjects.size(); jt++ ) {
							if(knownObjects.at(jt)==object.at(it)){
								std::cout << knownObjects.at(jt) << " and " << object.at(it)  << " are the same!" << std::endl;*/
					}
				}
				 else
					nextState = SM_LOOK_IN_SHELVES;
				break;

			case SM_GRAB_OBJECTS:
                                writeReport(fl,stp,"ObjectFinding",srvs);
                                JustinaVision::stopObjectFinding();
                                JustinaTools::pdfStop(fl);
                                JustinaTools::pdfImageExport(testName,imgPath);
				JustinaTasks::graspNearestObject(true);
				JustinaTasks::graspNearestObject(false);
				nextState = SM_FINAL_STATE;
				break;

			case SM_FINAL_STATE:
                                //manipulation vars clear
                                object.clear();
                                xCoord.clear();
                                yCoord.clear();
                                zCoord.clear();
				//
                                fullReport(fl,fnladv);
				fullReport(fl,eot);
				success = true;
				break;
        	}
        	ros::spinOnce();
	        loop.sleep();
	}
    	return 0;
}
