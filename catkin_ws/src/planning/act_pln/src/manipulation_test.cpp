#include <iostream>
#include <stdlib.h>
#include "ros/ros.h"
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"
#include "vision_msgs/VisionObject.h"
#include "vision_msgs/DetectObjects.h"

	/*RoboCup 2016.
		Task:
		Find (Vision) and Grasp (Arm control) objects

		Situation:
		Bookcase with 10 objects over the shelves, identify 5 and group
		them in a different shelf, optionaly open a door or drawer in
		the bookcase.

		Steps to resolve in 3min: (150p on this)
		* Start from a voice command or a button
		* Search for a bookcase
		* Translate from a random distance between 1m and 1.5m from the
		 bookcase
		(info) Bookcase with a elevation of 0.30m over the floor and height
		 of 1.8m, at least 5 shelves
		* Recognize the empty shelf
		* 10 objects over all the shelves, find and grasp 5 from a standard
		 set. For each object, until 5
			* Each time an object is recognized, take his picture
			 and place a precise label, optionally also an overview
			 of the shelf with labeled bounding boxes. Add this to a
			 file (10p)
			* Labeling false positive objects is penalized (-5p)
			* Grasp the object for more than 10 sec and 5 cm out of the
			 shelf (10p)
			* Move the identified object to the empty shelf, the
			 object must stay there 10 sec. min (10p)
		* When 5 objects identified, the test ends, the file of the objects
		 must be saved as PDF
		Bonus. Open the drawer or door
			+50 points
		Bonus. According to sec. 3.9.3 of the rulebook
			+15 points max, up to comitee decision and a team anounce,
			 before the test. Mostly upon innovative approachs
		Bonus. According to sec. 3.4 of the rulebook, "dataset contribution"
			+10 points max, if all its done. points (current p/ max p)*10

		Solve:
		* Find objects:

		Minimal identified objects in position:
		* Upper shelf
		* Middle shelf
		* Lower shelf
		* Optional ocluded object in a middle shelf
		* Optional object behind the bookcase door

		Minimal recognition needs:
		Objects from dataset
		One ocluded object from dataset

		Minimal manupulation and navigation needs:
		First time:
			* Navigate to the shelf
			* Arm to object
			* Grab object (close gripper)
			* Lift object linearly 5cm
			* Move arm with object to a "safe" position, to avoid
			 collitions
			* Move arm to shelf
			* Leave object (open gripper)
			* Move arm without object to a "safe" position, to avoid
			 collitions

Last proposed algorithm 	(27-May-16)
Task still to be acomplished: Move the object to the empty shelf
--------------------------Known conditions-------------------------------
	Objects set and his ID's
	"PDF folder" archives location
	Distance to "full bookcase" (to see objects)
	Distance from "full bookcase" to bookcase
	Angle from head to 1st shelf with objects
	Angle from head to 2nd shelf with objects
	Angle from head to 3rth shelf with objects
	Angle from head to 4rth shelf with objects
	Angle from head to empty shelf
	Order of shelf importance (1 to 4)
------------------------------Pseudocode---------------------------------
Empty "PDF folder"
Verify arms and head "resting" pose
Wait for voice command "start"
Find bookcase
Navigate to "full bookcase"
From shelf 1 to 4, var k
	Move head from angle here to shelf "k" angle
	Full scene of current shelf and his plane
	Segment objects in current plane
	Obtain ID and x,y data from each object
	Add ID to name list
	From ID 1 to "n", var h
	Y) Search if ID "h" already exist in "PDF folder"
	 	N) Add object with ID "h" to PDF folder
		   Add objects position to positions list/array/etc
		  -The position list will include shelf number and x,y data-
Create PDF from PDF Folder files
Navigate to the bookcase
Align to the bookcases
From shelf 1 to 4, or time > 3min, var i
	Move head y-axis from here to shelf "i" angle
	Compare stored objects related to shelf "i" and...
	 ...order from less to more distance from object to arm
	From object 1 to "n", var j
		Math: Head pose from here to 'x' coordinates of object "j"...
		  ...return "m" signed degres
		Move head x-axis "m" degrees
		Move Arm from "resting" position to object position
		Close gripper
		Speak simething like "Object j found"
		Move hand section upwards 5cm
		Hold on 10 sec
		Speak something like "Move this object is out boundaries"
		Move hand section downwards 5cm
		Open gripper
		Move Arm from object position to "resting" position
---------------------------Neededfunctions------------------------------
*justina_tools/JustinaManip.h
Read head position on "b" axis
Move head "a" degrees on "b" axis
*
Read arm pose
Move arm from "a" to "b"

*..JustinaHardware.h
Gripper angle close control
Read gripper pose
*
Servo individual control (Read/Write)
Servo group control (Read/Write)

*..JustinaNavigation.h
Displace robot from "a" to "b"

*..JustinaHRI.h
Robot speak phrase
Robot listen and identify phrase

*..JustinaVision.h (T.B.A.)
Receive "Cloud" of object w/ bounding box with centroid data (Jesus program?)
Compare two (or more) "Cloud" objects (?)
Export "Cloud" data to JPG

* InverseKinematics -> True -> Move Resp. robot (False DONT)
Calculate euclidean distance from robot kinect (or head?) to object
Calculate euclidean distance from robot gripper (or hand?) to object
	*/

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
	std::cout << theString << std::endl;
        ss << jst << theString;
        justinaTools::pdfAppend(fl,ss.str());
        ss.str(std::string());
        ss.clear();
        JustinaHRI::say(theString);
	sleep(3);
}

void writeReport(std::string fl, std::string theString){
        std::cout << theString << std::endl;
	JustinaTools::pdfAppends(fl,theString);
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
	float tempAng=0;
	//PRE-DEFINED ROBOT HEIGHT CENTIMETERS
	float height[5] = {0, 17, 0, -0.1, 0};
	int startTorso=0.13;
	//OBJECTS LIST
	std::string imgPath = "/home/$USER/Pictures/";
	std::string testName = "Object recognition and manipulation test";
	std::vector<std::string> object;
	std::vector<vision_msgs::VisionObject> detectedObjects;
	std::string objId = "empty";
	float x = 0.0;
	float y = 0.0;
	float z = 0.0;
	int maxOb[5]= {0}; //1 container per shelve, number of objects found
	int currentMaxOb = 0;
	//ARMS MOVEMENT
	//initial (from camera)
	float xi =0;
	float yi =0;
	float zi =0;
	//final (to Arm)
	float xf =0;
	float yf =0;
	float zf =0;
	float roll = 0;
	float pitch = 0;
	float yaw = 0;
	float elbow = 0;
	//time
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
	float timeOutSpeech = 9000;
	float timeOutHead = 5000;
	float timeOutTorso = 2000;
	//STRINGS
	std::stringstream ss;
	std::string jst = "Justina says: "
	std::string fl = "ManipAndObjectReco_Plans";
	std::string init0 = "Initializing justina nodes...";
	std::string tors0 = "Sending justina to zero height...";
	std::string speak0 = "I am ready to manipulation object test...";
	std::string wait4ord = "I am waiting for the  command...";
	std::string rptcmd = "Please repeat the command...";
	std::string strtst = "I will now start the object recognition test...";
	std::string shlf = "I am gonna navigate to the shelfs...";
	std::string objFnd = "Object found...";
	std::string fnladv = "I can not grab the object...";
	std::string eot = "end of the test reached...";

    	while(ros::ok() && !fail && !success)
    	{
        	switch(nextState)
        	{
        		case SM_INIT;
				JustinaTools::pdfStart(fl);
				writeReport(fl,init0);
				nextState = SM_WAIT_INIT;
            			break;

                        case SM_WAIT_INIT:
                                if(!JustinaManip::torsoGoTo(startTorso, 0, 0, timeOutTorso)){
					writeReport(fl,torso0);
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
				nextState = SM_NAVIGATE_TO_BOOKCASE;
            			break;

		        case SM_LOOK_IN_SHELVES:
				currentMaxOb=0;
				tempAng=(headAngles[shelfCount]*3.1416)/180;
				JustinaManip::hdGoTo(0, tempAng, timeOutHead);
				height[shelfCount]=height[shelfCount]/100;
				JustinaManip::torsoGoToRel(height[shelfCount], 0, 0, timeOutTorso);
				sleep(6);
				std::cout << "Posicion " << shelfCount << ": " << tempAng <<std::endl;
				std::cout << "Altura " << shelfCount << ": " << height[shelfCount] << std::endl;
				//Vision
				JustinaVision::startObjectFinding();
				JustinaVision::startObjectFindingWindow();
				if(JustinaVision::detectObjects(detectedObjects))
				{
					for(int i=0; i<detectedObjects.size(); i++)
					{
						objId = detectedObjects[i].id;
						x = detectedObjects[i].pose.position.x;
						y = detectedObjects[i].pose.position.y;
						z = detectedObjects[i].pose.position.z;
						std::cout << "ID(" << i << ") " << objId << std::endl;
						std::cout << "x(" << x << ")y(" << y << ")z(" << z << ")" <<std::endl;
						fullReport(objfnd);
						sleep(3);
						FullReport(objId);
					}
				}
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

*/			case SM_FINAL_REPORT;
				JustinaVision::stopObjectFindingWindow();
				JustinaVision::stopObjectFinding();
				JustinaTools::pdfImageExport(testName,imgPath);
				JustinaTools::pdfStop(fl);
				nextState = SM_FINAL_STATE;
				break;

			case SM_FINAL_STATE:
				fullReport(fnladv);
				fullReport(eot);
				success = true;
				break;
        	}
        	ros::spinOnce();
	        loop.sleep();
	}
    	return 0;
}
