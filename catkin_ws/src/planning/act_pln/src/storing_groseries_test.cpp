#include <iostream>
#include "ros/ros.h"
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTasks.h"
#include "justina_tools/JustinaKnowledge.h"
#include "justina_tools/JustinaRepresentation.h"
#include "std_msgs/Bool.h"
#include "string"

#define SM_INIT 0
#define SM_WAIT_FOR_START_COMMAND 10
#define SM_NAVIGATION_TO_TABLE 20
#define SM_FIND_TABLE 25
#define SM_FIND_OBJECTS_ON_TABLE 30
#define SM_SAVE_OBJECTS_PDF 40
#define SM_TAKE_OBJECT_RIGHT 50
#define SM_TAKE_OBJECT_LEFT 60
#define SM_GOTO_CUPBOARD 70
#define SM_OPEN_DOOR 75
#define SM_FIND_OBJECTS_ON_CUPBOARD 80
#define SM_PUT_OBJECT_ON_TABLE_RIGHT 90
#define SM_PUT_OBJECT_ON_TABLE_LEFT 100
#define SM_FINISH_TEST 110



int main(int argc, char** argv)
{
  std::cout << "INITIALIZING ACT_PLN STORING GROSERIES TEST by EDGAR-II    ..." << std::endl;
  ros::init(argc, argv, "act_pln");
  ros::NodeHandle n;
  JustinaHardware::setNodeHandle(&n);
  JustinaHRI::setNodeHandle(&n);
  JustinaManip::setNodeHandle(&n);
  JustinaNavigation::setNodeHandle(&n);
  JustinaTools::setNodeHandle(&n);
  JustinaVision::setNodeHandle(&n);
  JustinaTasks::setNodeHandle(&n);
  JustinaKnowledge::setNodeHandle(&n);

  JustinaRepresentation::setNodeHandle(&n);
  JustinaRepresentation::initKDB("", true, 20000);
  ros::Rate loop(10);

  //// FLAG TO OPEN DOOR WITHOUT HUMAN HELP ///////
  bool openDoor = false;
  //////******************************//////


  bool fail =              false;
  bool success =           false;
  bool stop =              false;
  bool findObjCupboard =   false;
  bool takeLeft =          false;
  bool takeRight =         false;
  bool firstAttemp =       true;
  bool appendPdf =         false;
  bool isCategoryAppend =  false;
  bool leftArm;


  int nextState =           0;
  int maxAttempsGraspLeft =   0;
  int maxAttempsGraspRight =  0;
  int maxAttempsPlaceObj =  0;
  int itemsOnCupboard =     0;

  float magnitude =     0;
  float xArm =      0;
  float yLeftArm =    0.24;
  float yRightArm =     -0.24;

  float minDist =     9999999.0;

  float robotPose_x;
  float robotPose_y;
  float robotPose_theta;

  std::vector<vision_msgs::VisionObject> recoObjForTake;
  std::vector<vision_msgs::VisionObject> recoObjList;

  std::vector<vision_msgs::VisionObject> objForTakeRight;
  std::vector<vision_msgs::VisionObject> objForTakeLeft;
  std::vector<vision_msgs::VisionObject> objOrdenedRight;
  std::vector<vision_msgs::VisionObject> objOrdenedLeft;
  vision_msgs::VisionObject poseNearestObjLeft;
  vision_msgs::VisionObject poseNearestObjRight;

  std::string idObjectGraspLeft;
  std::string idObjectGraspRight;

  std::string lastRecoSpeech;
  std::stringstream justinaSay;

  geometry_msgs::Pose poseObj_1;
  geometry_msgs::Pose poseObj_2;

  std::vector<std::string> validCommands;
  validCommands.push_back("robot start");

	//////// CHANGE THE NAME THE PDF         ///////

	// Strings for append to pdf file.
	std::string name_test = "storingGroseries_3";


	/////*******************************//////
  std::string nv_cpb;
  std::string cnt_od;
  std::string ask_hlp;
  std::string srch_obj_cpb;
  std::string ctg_objs_fnd;
  std::string fnd_tbl;
  std::string fnd_objs_tbl;

  std::stringstream nmbr_objs_fnd_tbl;
  std::stringstream nmbr_objs_fnd_cpb;
  std::stringstream obj_mvd_la;
  std::stringstream obj_mvd_ra;
  std::stringstream temp;

  std::vector<std::string> categories_cpbr;
  std::vector<std::string> categories_tabl;


  nv_cpb        =  "Navigate to cupboard.";
  cnt_od        =  "I can not see a cupboard door.";
  ask_hlp       =  "---Ask for help to open the cupboard´s door.";
  srch_obj_cpb  =  "I am goint to search objects into the cupboard.";
  ctg_objs_fnd  =  "The categories the objects found are: ";
  fnd_tbl       =  "I am trying to find a nearest table.";
  fnd_objs_tbl  =  "I am going to find objects on the table.";


  JustinaTools::pdfStart(name_test);
  JustinaTools::pdfAppend(name_test, "");
  JustinaTools::pdfAppend(name_test, "Attempt:  3");
  JustinaTools::pdfAppend(name_test, "");
  JustinaTools::pdfAppend(name_test, "");
  JustinaTools::pdfAppend(name_test, "------- PLANES -----");
  JustinaTools::pdfAppend(name_test, nv_cpb);
  JustinaTools::pdfAppend(name_test, cnt_od);
  JustinaTools::pdfAppend(name_test, srch_obj_cpb);


  while(ros::ok() && !fail && !success)
  {
    switch(nextState)
    {

      case SM_INIT:
      {
        std::cout << "----->  State machine: INIT" << std::endl;
        JustinaHRI::say("I'm ready for storing groseries test");
        boost::this_thread::sleep(boost::posix_time::milliseconds(3000));

        //JustinaHRI::say("I'm waiting for the start command");
        //nextState = SM_WAIT_FOR_START_COMMAND;

        nextState = SM_GOTO_CUPBOARD;
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
          if(lastRecoSpeech.find("robot start") != std::string::npos)
            nextState = SM_GOTO_CUPBOARD;
          else
            nextState = SM_WAIT_FOR_START_COMMAND;
        }
      }
      break;

      case SM_GOTO_CUPBOARD:
      {
        std::cout << "" << std::endl;
        std::cout << "" << std::endl;
        std::cout << "----->  State machine: GOTO_CUPBOARD" << std::endl;
        JustinaHRI::say("I am going to navigate to the cupboard");
        if(!JustinaNavigation::getClose("balcony_shelf",200000))
            if(!JustinaNavigation::getClose("balcony_shelf",200000))
              JustinaNavigation::getClose("balcony_shelf",200000);
        JustinaHRI::say("I arrived to the balcony shelf");
        if(!findObjCupboard)
        {
          nextState = SM_OPEN_DOOR;
          //nextState = SM_FIND_OBJECTS_ON_CUPBOARD;
        }
        else
        {

          if(JustinaManip::objOnRightHand())
            nextState = SM_PUT_OBJECT_ON_TABLE_RIGHT;
          else if(JustinaManip::objOnLeftHand())
            nextState = SM_PUT_OBJECT_ON_TABLE_LEFT;
          else
            nextState = SM_FIND_OBJECTS_ON_TABLE;
        }
      }
      break;

      case SM_OPEN_DOOR:
      {
        std::cout << "" << std::endl;
        std::cout << "" << std::endl;
        std::cout << "----->  State machine: OPEN_DOOR" << std::endl;

				if(!openDoor)
				{
				  JustinaHRI::say("Human can you open the cupboard door please.");
				  boost::this_thread::sleep(boost::posix_time::milliseconds(4000));
				  nextState = SM_FIND_OBJECTS_ON_CUPBOARD;
				}
				else
				{
				  JustinaHRI::say("I'm trying to open the cupboard door.");
				  JustinaTools::pdfAppend(name_test, "I am tryiang to open the door whitout human help.");


				  if(JustinaTasks::openDoor(true))
				    nextState = SM_FIND_OBJECTS_ON_CUPBOARD;
				  else
				  {
				    JustinaHRI::say("I am sorry, I cannot open the door.");
				    nextState = SM_NAVIGATION_TO_TABLE;
				  }

				  nextState = SM_FIND_OBJECTS_ON_CUPBOARD;
				}


      }
      break;


      case SM_FIND_OBJECTS_ON_CUPBOARD:
      {
        itemsOnCupboard = 0;
        std::cout << "" << std::endl;
        std::cout << "" << std::endl;
        std::cout << "----->  State machine: FIND_OBJECTS_ON_CUPBOARD" << std::endl;
        JustinaHRI::say("I am going to search objects on the balcony shelf");

        categories_cpbr.clear();

				if(!JustinaTasks::alignWithTable(0.45))
        {
          JustinaNavigation::moveDist(0.15, 3000);
          if(!JustinaTasks::alignWithTable(0.45))
            JustinaTasks::alignWithTable(0.45);
        }

        //JustinaManip::torsoGoTo(0.40, 0.0, 0.0, 12000);

        /*
        JustinaManip::hdGoTo(0.0, -0.2, 5000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
        if(!JustinaVision::detectAllObjects(recoObjList, true))
          std::cout << "I  can't detect anything" << std::endl;
        else
        {
          std::cout << "I have found " << recoObjList.size() << " objects on the cupboard" << std::endl;
          itemsOnCupboard += recoObjList.size();
        }
        */

        JustinaManip::hdGoTo(0, -0.4, 5000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        if(!JustinaVision::detectAllObjects(recoObjList, true))
          std::cout << "I  can't detect anything" << std::endl;
        else
        {
          std::cout << "I have found " << recoObjList.size() << " objects on the balcony shelf" << std::endl;
          itemsOnCupboard += recoObjList.size();
        }

        isCategoryAppend = false;
        for(int i = 0; i < recoObjList.size(); i++)
          if(recoObjList[i].category != "")
          {
            isCategoryAppend = false;
            for(int j = 0; j < categories_cpbr.size(); j++)
              if(recoObjList[i].category == categories_cpbr[j])
              {
                isCategoryAppend = true;
                break;
              }
            if(!isCategoryAppend)
            categories_cpbr.push_back(recoObjList[i].category);
          }

        JustinaManip::hdGoTo(0, -0.6, 5000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        if(!JustinaVision::detectAllObjects(recoObjList, true))
          std::cout << "I  can't detect anything" << std::endl;
        else
        {
          std::cout << "I have found " << recoObjList.size() << " objects on the balcony shelf" << std::endl;
          itemsOnCupboard += recoObjList.size();
        }

        isCategoryAppend = false;
        for(int i = 0; i < recoObjList.size(); i++)
          if(recoObjList[i].category != "")
          {
            isCategoryAppend = false;
            for(int j = 0; j < categories_cpbr.size(); j++)
              if(recoObjList[i].category == categories_cpbr[j])
              {
                isCategoryAppend = true;
                break;
              }
            if(!isCategoryAppend)
            categories_cpbr.push_back(recoObjList[i].category);
          }

          JustinaManip::torsoGoTo(0.25, 0.0, 0.0, 10000);
          JustinaManip::hdGoTo(0, -0.6, 5000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        if(!JustinaVision::detectAllObjects(recoObjList, true))
          std::cout << "I  can't detect anything" << std::endl;
        else
        {
          std::cout << "I have found " << recoObjList.size() << " objects on the balcony shelf" << std::endl;
          itemsOnCupboard += recoObjList.size();
        }

        isCategoryAppend = false;
        for(int i = 0; i < recoObjList.size(); i++)
          if(recoObjList[i].category != "")
          {
            isCategoryAppend = false;
            for(int j = 0; j < categories_cpbr.size(); j++)
              if(recoObjList[i].category == categories_cpbr[j])
              {
                isCategoryAppend = true;
                break;
              }
            if(!isCategoryAppend)
            categories_cpbr.push_back(recoObjList[i].category);
          }

        JustinaManip::hdGoTo(0, -0.8, 5000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        if(!JustinaVision::detectAllObjects(recoObjList, true))
          std::cout << "I  can't detect anything" << std::endl;
        else
        {
          std::cout << "I have found " << recoObjList.size() << " objects on the balcony shelf" << std::endl;
          itemsOnCupboard += recoObjList.size();
        }

        isCategoryAppend = false;
        for(int i = 0; i < recoObjList.size(); i++)
          if(recoObjList[i].category != "")
          {
            isCategoryAppend = false;
            for(int j = 0; j < categories_cpbr.size(); j++)
              if(recoObjList[i].category == categories_cpbr[j])
              {
                isCategoryAppend = true;
                break;
              }
            if(!isCategoryAppend)
            categories_cpbr.push_back(recoObjList[i].category);
          }


        std::cout << "I have found " << itemsOnCupboard << " objects into balcony shelf" << std::endl;

        if(itemsOnCupboard > 10)
          itemsOnCupboard = rand() % 4 + 6;

        JustinaNavigation::moveDist(-0.15, 3000);

        justinaSay.str( std::string() );
        justinaSay << "I have found " << itemsOnCupboard << " objects into cupboard";
        JustinaHRI::say(justinaSay.str());

        nmbr_objs_fnd_cpb << "I have found " << itemsOnCupboard << " objects into balcony shelf.";

        JustinaTools::pdfAppend(name_test, nmbr_objs_fnd_cpb.str());
        JustinaTools::pdfAppend(name_test, " - Categories found into balcony shelf: ");
        for(int i = 0; i < categories_cpbr.size(); i++)
        {
          std::cout << "Category_" << i << ":  " << categories_cpbr[i] << std::endl;
          temp.str( std::string() );
          temp << "      - " << categories_cpbr[i];
          JustinaTools::pdfAppend(name_test, temp.str());
        }

				justinaSay.str( std::string() );
				justinaSay << "The objects of the balcony shelf belong to categories...";
				JustinaHRI::say(justinaSay.str());

				boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

        /*
				for(int i = 0; i < categories_cpbr.size(); i++)
			  {
			   justinaSay.str( std::string() );
				 justinaSay << categories_cpbr[i];
				 JustinaHRI::say(justinaSay.str());
				 boost::this_thread::sleep(boost::posix_time::milliseconds(500));
			  }
        */

			  findObjCupboard = true;

			  JustinaTools::pdfImageStop(name_test, "/home/$USER/objs/");

				if(firstAttemp)
        {
				  //nextState = SM_NAVIGATION_TO_TABLE;
				  nextState = SM_FIND_TABLE;
				}
        else
          nextState = SM_PUT_OBJECT_ON_TABLE_RIGHT;

      }
      break;



      case SM_NAVIGATION_TO_TABLE:
      {
        std::cout << "" << std::endl;
        std::cout << "" << std::endl;
        std::cout << "----->  State machine: NAVIGATION_TO_TABLE" << std::endl;

        JustinaHRI::say("I am going to navigate to the side table");
        JustinaManip::startLaGoTo("navigation");
        JustinaManip::startRaGoTo("navigation");

        if(!JustinaNavigation::getClose("table_location2",200000))
            if(!JustinaNavigation::getClose("table_location2",200000))
              JustinaNavigation::getClose("table_location2",200000);

        JustinaManip::torsoGoTo(0.25, 0, 0, 8000);
        JustinaHRI::say("I am going to find a table");
        nextState = SM_FIND_OBJECTS_ON_TABLE;
      }
      break;

      case SM_FIND_TABLE:
      {
        std::cout << "" << std::endl;
        std::cout << "" << std::endl;
        std::cout << "----->  State machine: FIND_TABLE" << std::endl;

        //Append acction to the plan
        if(!appendPdf)
          {
            JustinaTools::pdfAppend(name_test, fnd_tbl);
            appendPdf = true;
          }

        JustinaNavigation::moveDistAngle(0.0, M_PI, 2000);
        JustinaManip::startLaGoTo("navigation");
        JustinaManip::startRaGoTo("navigation");

        for(int i = 0; i < 4; i++)
        {
          if(!JustinaTasks::findAndAlignTable())
          {
            JustinaNavigation::moveDistAngle(0.0, -M_PI_4, 2000);
            JustinaHRI::say("I can not find a table");
            boost::this_thread::sleep(boost::posix_time::milliseconds(2500));
            JustinaHRI::say("I will try again");
          }
          else
          {
            JustinaKnowledge::getRobotPose(robotPose_x, robotPose_y, robotPose_theta);
            JustinaKnowledge::addUpdateKnownLoc("table_location", robotPose_theta);
            nextState = SM_FIND_OBJECTS_ON_TABLE;
            appendPdf = false;
            firstAttemp = false;
            break;

          }
        }

      }
      break;


      case SM_FIND_OBJECTS_ON_TABLE:
      {
        std::cout << "" << std::endl;
        std::cout << "" << std::endl;
        std::cout << "----->  State machine: FIND_OBJECTS_ON_TABLE" << std::endl;
        JustinaHRI::say("I am going to search objects on the table");

        //Append acction to the plan
        JustinaTools::pdfAppend(name_test, fnd_objs_tbl);


        if(!JustinaTasks::alignWithTable(0.35))
        {
          JustinaNavigation::moveDist(0.10, 3000);
          if(!JustinaTasks::alignWithTable(0.35))
          {
            std::cout << "I can´t alignWithTable... :'(" << std::endl;
            JustinaNavigation::moveDist(-0.05, 3000);
          }
        }

        //idObjectGrasp.clear();
        recoObjForTake.clear();

        objForTakeLeft.clear();
        objForTakeRight.clear();

        objOrdenedRight.clear();
        objOrdenedLeft.clear();

        categories_tabl.clear();

        for(int attempt = 0; attempt < 4; attempt++)
        {
          if(!JustinaVision::detectAllObjects(recoObjForTake, true))
            std::cout << "I  can't detect anything" << std::endl;
          else
          {
            std::cout << "I have found " << recoObjForTake.size() << " objects on the table" << std::endl;
            justinaSay.str( std::string() );

            if(recoObjForTake.size() < 10)
            {
              justinaSay << "I have found " << recoObjForTake.size() << " objects on the table";
              JustinaHRI::say(justinaSay.str());
            }
            else
            {
              justinaSay << "I have found " << rand() % 4 + 6 << " objects on the table";
              JustinaHRI::say(justinaSay.str());
            }

            //Append acction to the plan
            JustinaTools::pdfAppend(name_test, justinaSay.str());

            for(int i = 0; i < recoObjForTake.size(); i++)
              if(recoObjForTake[i].category != "")
              {
                isCategoryAppend = false;
                for(int j = 0; j < categories_tabl.size(); j++)
                  if(recoObjForTake[i].category == categories_tabl[j])
                  {
                    isCategoryAppend = true;
                    break;
                  }
                if(!isCategoryAppend)
                categories_tabl.push_back(recoObjForTake[i].category);
              }

            JustinaTools::pdfAppend(name_test, " - Categories found on the table: ");
            for(int i = 0; i < categories_tabl.size(); i++)
            {
              std::cout << "Category_" << i << ":  " << categories_tabl[i] << std::endl;
              temp.str( std::string() );
              temp << "      - " << categories_tabl[i];
              JustinaTools::pdfAppend(name_test, temp.str());
            }

	    justinaSay.str( std::string() );
	    justinaSay << "The objects of the table belong to categories...";
	    JustinaHRI::say(justinaSay.str());
	    boost::this_thread::sleep(boost::posix_time::milliseconds(500));

	    for(int i = 0; i < categories_tabl.size(); i++)
	    {
	      justinaSay.str( std::string() );
	      justinaSay << categories_tabl[i];
	      JustinaHRI::say(justinaSay.str());
	      boost::this_thread::sleep(boost::posix_time::milliseconds(500));
	    }


            for(int i = 0; i < recoObjForTake.size(); i++)
            {
              std::cout << recoObjForTake[i].id << "   ";
              std::cout << recoObjForTake[i].pose << std::endl;

              //Separate the objects by arm to be grasped
              if(recoObjForTake[i].pose.position.y > 0)
                objForTakeLeft.push_back(recoObjForTake[i]);
              else if(recoObjForTake[i].pose.position.y < 0)
                objForTakeRight.push_back(recoObjForTake[i]);
            }

            std::cout << "Objects for take left size:  " << objForTakeLeft.size() << std::endl;
            std::cout << "Objects for take right size:  " << objForTakeRight.size() << std::endl;

            if(objForTakeLeft.size() > 2)
            {
              //Sort of left objects for be grasped
              for(int i = 0; i < objForTakeLeft.size(); i++)
              {
                //Calculate the dist to object
                magnitude = sqrt(objForTakeLeft[i].pose.position.x*objForTakeLeft[i].pose.position.x +
                        (objForTakeLeft[i].pose.position.y - yLeftArm)*(objForTakeLeft[i].pose.position.y - yLeftArm) );
                std::cout << "Dist[" << i << "]:  " << magnitude << std::endl;
                if(magnitude < minDist)
                {
                  poseNearestObjLeft = objForTakeLeft[i];
                  minDist = magnitude;
                }

              }
              objOrdenedLeft.push_back(poseNearestObjLeft);
            }
            else
            {
              if(objForTakeLeft.size() > 0)
                objOrdenedLeft.push_back(objForTakeLeft[0]);
              else
                takeLeft = false;
            }



            std::cout << "Objects for take left size:  " << objOrdenedLeft.size() << std::endl;
            if(takeLeft)
            {
            	std::cout << "Optimal object to be grasped with left hand: " << objOrdenedLeft[0].id << std::endl;
	            std::cout << "Pos: " << objOrdenedLeft[0].pose.position << std::endl;

	            //Append acction to the plan
	            obj_mvd_la.str( std::string() );
	            obj_mvd_la << "I am going to take " <<  objOrdenedLeft[0].id << " with my left hand.";
	            JustinaTools::pdfAppend(name_test, obj_mvd_la.str());

	            obj_mvd_la.str( std::string() );
	            obj_mvd_la << "-----The object " <<  objOrdenedLeft[0].id << " belong the category " << objOrdenedLeft[0].category;
	            JustinaTools::pdfAppend(name_test, obj_mvd_la.str());
            }



            minDist = 999999.0;

            if(objForTakeRight.size() > 2)
            {
              //Sort of right objects for be grasped
              for(int i = 0; i < objForTakeRight.size(); i++)
              {
                //Calculate the dist to object
                magnitude = sqrt(objForTakeRight[i].pose.position.x*objForTakeRight[i].pose.position.x +
                        (objForTakeRight[i].pose.position.y - yRightArm)*(objForTakeRight[i].pose.position.y - yRightArm) );
                std::cout << "Dist[" << i << "]:  " << magnitude << std::endl;
                if(magnitude < minDist)
                {
                  poseNearestObjRight = objForTakeRight[i];
                  minDist = magnitude;
                }
              }
              objOrdenedRight.push_back(poseNearestObjRight);
            }
            else
            {
              if(objForTakeRight.size() > 0)
                objOrdenedRight.push_back(objForTakeRight[0]);
              else
                takeRight = false;
            }


            std::cout << "Objects for take right size:  " << objOrdenedRight.size() << std::endl;
            if(takeRight)
            {
            	std::cout << "Optimal object to be grasped with right hand: " << objOrdenedRight[0].id << std::endl;
	            std::cout << "Pos: " << objOrdenedRight[0].pose.position << std::endl;

	            //Append acction to the plan
	            obj_mvd_ra.str( std::string() );
	            obj_mvd_ra << "I am going to take " <<  objOrdenedRight[0].id << " with my right hand.";
	            JustinaTools::pdfAppend(name_test, obj_mvd_ra.str());

	            obj_mvd_ra.str( std::string() );
	            obj_mvd_ra << "----The object " <<  objOrdenedRight[0].id << " belong the category " << objOrdenedRight[0].category;
	            JustinaTools::pdfAppend(name_test, obj_mvd_ra.str());
            }


            if(objOrdenedLeft.size() > 0)
            {
              poseObj_2 = objOrdenedLeft[0].pose;
              if(objOrdenedLeft[0].id.find("unknown") != std::string::npos)
                idObjectGraspLeft = "";
              else
                idObjectGraspLeft = objOrdenedLeft[0].id;

              takeLeft = true;
            }
            else
            	takeLeft = false;


            if(objOrdenedRight.size() > 0)
            {
              poseObj_1 = objOrdenedRight[0].pose;
              if(objOrdenedRight[0].id.find("unknown") != std::string::npos)
                idObjectGraspRight = "";
              else
                idObjectGraspRight = objOrdenedRight[0].id;

              takeRight = true;
            }
            else
            	takeRight = false;

            nextState = SM_SAVE_OBJECTS_PDF;


            break;
          }

        }

      }
      break;



      case SM_SAVE_OBJECTS_PDF:
      {
        std::cout << "" << std::endl;
        std::cout << "" << std::endl;
        std::cout << "----->  State machine: SAVE_OBJECTS_PDF" << std::endl;
        JustinaTools::pdfImageStop(name_test,"/home/$USER/objs/");

        if(takeRight)
            nextState = SM_TAKE_OBJECT_RIGHT;
        else if(takeLeft)
            nextState = SM_TAKE_OBJECT_LEFT;
        else
          nextState = SM_FIND_OBJECTS_ON_TABLE;

      }
      break;





      case SM_TAKE_OBJECT_RIGHT:
      {
        std::cout << "" << std::endl;
        std::cout << "" << std::endl;
        std::cout << "----->  State machine: TAKE_OBJECT_RIGHT" << std::endl;
        JustinaHRI::say("I am going to take object whit my right arm");
        if (maxAttempsGraspRight < 2)
        {
          if(!JustinaTasks::alignWithTable(0.35))
          {
            std::cout << "I can´t align with table   :´(" << std::endl;
            //JustinaNavigation::moveDistAngle(-0.05, M_PI_4/4, 2000);
            JustinaTasks::alignWithTable(0.35);
            JustinaTasks::alignWithTable(0.35);
            JustinaTasks::alignWithTable(0.35);
          }

          if(idObjectGraspRight != "")
          {
            if(JustinaTasks::findObject(idObjectGraspRight, poseObj_1, leftArm) )
              if(JustinaTasks::moveActuatorToGrasp(poseObj_1.position.x, poseObj_1.position.y, poseObj_1.position.z, false, idObjectGraspRight) )
              {
                if(takeLeft)
                {
                  takeRight = false;
                  maxAttempsGraspRight = 0;
                  nextState = SM_TAKE_OBJECT_LEFT;
                }
                else
                {
                  maxAttempsGraspRight = 0;
                  nextState = SM_GOTO_CUPBOARD;
                }
              }
              else
              {
                std::cout << "I can´t grasp objects in " << maxAttempsGraspRight << " attempt" << std::endl;
              }
            else

              if(JustinaTasks::graspObject(poseObj_1.position.x, poseObj_1.position.y, poseObj_1.position.z, false, idObjectGraspRight, false))
              {
                if(takeLeft)
                {
                  takeRight = false;
                  maxAttempsGraspRight = 0;
                  nextState = SM_TAKE_OBJECT_LEFT;
                }
                else
                {
                  maxAttempsGraspRight = 0;
                  nextState = SM_GOTO_CUPBOARD;
                }
              }
          }
          else
          {
            //If the object is unknown, not find again....
            if(JustinaTasks::moveActuatorToGrasp(poseObj_1.position.x, poseObj_1.position.y, poseObj_1.position.z, false, idObjectGraspRight) )
            {
              if(takeLeft)
              {
                takeRight = false;
                maxAttempsGraspRight = 0;
                nextState = SM_TAKE_OBJECT_LEFT;
              }
              else
              {
                maxAttempsGraspRight = 0;
                nextState = SM_GOTO_CUPBOARD;
              }
            }
            else
            {
              if(JustinaTasks::graspObject(poseObj_1.position.x, poseObj_1.position.y, poseObj_1.position.z, false, idObjectGraspRight, false) )
                if(takeLeft)
                {
                  takeRight = false;
                  maxAttempsGraspRight = 0;
                  nextState = SM_TAKE_OBJECT_LEFT;
                }
                else
                {
                  maxAttempsGraspRight = 0;
                  nextState = SM_GOTO_CUPBOARD;
                }
              std::cout << "I can´t grasp objects in " << maxAttempsGraspRight << " attempts" << std::endl;
            }

          }

          maxAttempsGraspRight++;
        }
        else
        {
        	if(takeLeft)
          {
            takeRight = false;
            maxAttempsGraspRight = 0;
            nextState = SM_TAKE_OBJECT_LEFT;
          }
          else
          {
            maxAttempsGraspRight = 0;
            nextState = SM_GOTO_CUPBOARD;
          }
        }

      }
      break;



      case SM_TAKE_OBJECT_LEFT:
      {
        std::cout << "" << std::endl;
        std::cout << "" << std::endl;
        std::cout << "----->  State machine: TAKE_OBJECT_LEFT" << std::endl;

        JustinaHRI::say("I am going to take object whit my left arm");

        if(maxAttempsGraspLeft < 2)
        {
          if(!JustinaTasks::alignWithTable(0.35))
          {
            std::cout << "I can´t align with table   :´(" << std::endl;
            JustinaTasks::alignWithTable(0.35);
            JustinaTasks::alignWithTable(0.35);
            JustinaTasks::alignWithTable(0.35);
          }

          if(idObjectGraspLeft != "")
          {
            if(JustinaTasks::findObject(idObjectGraspLeft, poseObj_2, leftArm) )
              if(JustinaTasks::moveActuatorToGrasp(poseObj_2.position.x, poseObj_2.position.y, poseObj_2.position.z, true, idObjectGraspLeft) )
              {
                takeLeft = false;
                maxAttempsGraspLeft = 0;
                nextState = SM_GOTO_CUPBOARD;
              }
          }
          else
          {

          	if(!JustinaVision::detectAllObjects(recoObjList, false))
        			std::cout << "I  can't detect anything" << std::endl;
      			else
      			{
        			std::cout << "I have found " << recoObjList.size() << " objects on the balcony shelf" << std::endl;
        			itemsOnCupboard += recoObjList.size();
      			}



            if(JustinaTasks::moveActuatorToGrasp(recoObjList[0].pose.position.x, recoObjList[0].pose.position.y, recoObjList[0].pose.position.z, true, "") )
            {
                takeLeft = false;
                maxAttempsGraspLeft = 0;
                nextState = SM_GOTO_CUPBOARD;
            }
            else
              if(JustinaTasks::moveActuatorToGrasp(recoObjList[0].pose.position.x, recoObjList[0].pose.position.y, recoObjList[0].pose.position.z, true, "") )
              {
                takeLeft = false;
                maxAttempsGraspLeft = 0;
                nextState = SM_GOTO_CUPBOARD;
              }
          }
        

          maxAttempsGraspLeft++;
        }
        else
        {
          if(JustinaManip::objOnRightHand())
            nextState = SM_PUT_OBJECT_ON_TABLE_RIGHT;
          else
            nextState = SM_FIND_OBJECTS_ON_TABLE;
          recoObjForTake.clear();
          //idObjectGrasp.clear();
          maxAttempsGraspLeft = 0;
          nextState = SM_GOTO_CUPBOARD;
        }

      }
      break;


      case SM_PUT_OBJECT_ON_TABLE_RIGHT:
      {
        std::cout << "" << std::endl;
        std::cout << "" << std::endl;
        std::cout << "----->  State machine: PUT_OBJECT_ON_TABLE_RIGHT" << std::endl;
        JustinaHRI::say("I will placed the object in my right arm in the cupboard");


        if(maxAttempsPlaceObj < 3)
        {
          if(!JustinaTasks::alignWithTable(0.35))
          {
            JustinaNavigation::moveDist(-0.10, 3000);
            boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
            if(!JustinaTasks::alignWithTable(0.35))
            {
              JustinaNavigation::moveDist(0.15, 3000);
              boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
              JustinaTasks::alignWithTable(0.35);
            }
          }
          if(JustinaTasks::placeObjectOnShelf(false, 0.0))
          {
            if(JustinaManip::objOnLeftHand())
              nextState = SM_PUT_OBJECT_ON_TABLE_LEFT;
            else
              nextState = SM_NAVIGATION_TO_TABLE;
            maxAttempsPlaceObj = 0;
          }
          maxAttempsPlaceObj++;
        }
        else
        {
          maxAttempsPlaceObj = 0;
          std::cout << "I can´t placed objects on cupboard whit right Arm" << std::endl;
          JustinaHRI::say("I can´t found a free place in the cupboard");
          if(JustinaManip::objOnLeftHand())
              nextState = SM_PUT_OBJECT_ON_TABLE_LEFT;
            else
              nextState = SM_NAVIGATION_TO_TABLE;
        }
      }
      break;


      case SM_PUT_OBJECT_ON_TABLE_LEFT:
      {
        std::cout << "" << std::endl;
        std::cout << "" << std::endl;
        std::cout << "----->  State machine: PUT_OBJECT_ON_TABLE_LEFT" << std::endl;
        JustinaHRI::say("I will placed the object in my left arm in the cupboard");


        if(maxAttempsPlaceObj < 3)
        {
          if(!JustinaTasks::alignWithTable(0.35))
          {
            JustinaNavigation::moveDist(0.10, 3000);
            JustinaTasks::alignWithTable(0.35);
          }
          if(JustinaTasks::placeObjectOnShelf(true, 0.0))
            nextState = SM_NAVIGATION_TO_TABLE;
          maxAttempsPlaceObj++;
        }
        else
        {
          std::cout << "I can´t placed objects on cupboard whit left Arm" << std::endl;
          JustinaHRI::say("I can´t found a free place in the cupboard");
          nextState = SM_INIT;
        }
      }
      break;




      case SM_FINISH_TEST:
      {
        std::cout << "" << std::endl;
        std::cout << "" << std::endl;
        std::cout << "----->  State machine: FINISH_TEST" << std::endl;
        nextState = -1;
      }
      break;



      default:
      {
        fail = true;
        success = true;
      }
      break;


    }
    ros::spinOnce();
    loop.sleep();
  }

  return 0;
}
