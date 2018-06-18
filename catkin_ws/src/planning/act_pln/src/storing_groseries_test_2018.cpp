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
#define SM_NAVIGATION_WAIT_REACHED_TO_TABLE 21
#define SM_FIND_TABLE 25
#define SM_FIND_OBJECTS_ON_TABLE 30
#define SM_INF_TAKE_OBJECT 41
#define SM_TAKE_OBJECT  50
#define SM_TAKE_OBJECT_RIGHT 55
#define SM_TAKE_OBJECT_LEFT 60
#define SM_GOTO_CUPBOARD 70
#define SM_OPEN_DOOR 75
#define SM_FIND_OBJECTS_ON_CUPBOARD 80
#define SM_PUT_OBJECT_ON_CUPBOARD 100
#define SM_FINISH_TEST 110

std::string stateMachine = "stroing_groceries.->";

bool funCompNearestVisionObject(vision_msgs::VisionObject obj1, vision_msgs::VisionObject obj2){
    float ecDistObj1 = sqrt(pow(obj1.pose.position.x, 2) + pow(obj1.pose.position.y ,2) + pow(obj1.pose.position.z ,2));
    float ecDistObj2 = sqrt(pow(obj2.pose.position.x, 2) + pow(obj2.pose.position.y ,2) + pow(obj2.pose.position.z ,2));
    return (ecDistObj1 < ecDistObj2 );
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING ACT_PLN STORING GROSERIES TEST by REYNALDO ..." << std::endl;
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
    bool openDoor = true;
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
    bool useLastPoseGrasp = false;
    bool graspObject = false;

    bool objectGrasped[2] = {false, false};

    int nextState =           0;
    int itemsOnCupboard =     0;

    float magnitude =     0;
    float xArm =      0;
    float yLeftArm =    0.24;
    float yRightArm =     -0.24;

    float minDist =     9999999.0;

    float minConfidence = 0.0;

    float robotPose_x;
    float robotPose_y;
    float robotPose_theta;

    std::vector<vision_msgs::VisionObject> recoObjForTake;
    std::vector<vision_msgs::VisionObject> recoObjList;

    vision_msgs::VisionObject poseNearestObjLeft;
    vision_msgs::VisionObject poseNearestObjRight;

    std::string idObjectGraspLeft;
    std::string idObjectGraspRight;
    int indexObjectGraspLeft;
    int indexObjectGraspRight;
    int indexObjectGrasp;

    std::stringstream justinaSay;

    geometry_msgs::Pose poseObj_1;
    geometry_msgs::Pose poseObj_2;

    int countFindObjectsOnTable = 1;
    int countFindObjectsOnCupboard = 1;
    bool alignWithTable = true;
    /***************************************
     * This flag is to only grasp a one object
     ***************************************/
    bool onlyOneGraspObject = true;
    /****************************************/
    // This is for attemps to navigation on the table
    int attempsNavigation = 0;
    // This is for attemps to find objects on the table
    int attempsFindObjectsTable = 0;
    // This is for attemps to grasp object
    int attempsGraspObject = 0;
    // This is for attemps to grasp object with the left arm
    int attempsGraspLeft =   0;
    // This is for attemps to grasp object with the right arm
    int attempsGraspRight =  0;
    // This is for attemps to place objects on the table
    int attempsPlaceObj =  0;

    // This is for the max attemps to find Object table
    int maxAttempsFindObjectsTable = 2;
    // This is for the max attemps to navigation
    int maxAttempsNavigation = 3;
    // This is for the max attemps to take object
    int maxAttempsTakeObject = 1; 
    // This is for the max attemps to grasp with the left arm
    int maxAttempsGraspLeft = 3;
    // This is for the max attemps to grasp with the right arm
    int maxAttempsGraspRight = 3;
    // This is for the max attemps to place a object in the shelft
    int maxAttempsPlaceObj = 3;

    // This is for have a time to reached to the table
	boost::posix_time::ptime curr;
	boost::posix_time::ptime prev;

    sensor_msgs::Image image;

    //////// CHANGE THE NAME THE PDF         ///////

    // Strings for append to pdf file.
    std::string name_test = "storingGroseries_1";

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
    cnt_od        =  "I am search a cupboards door.";
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
    JustinaTools::pdfAppend(name_test, "I am trying to open the cupboards door");
    JustinaTools::pdfAppend(name_test, srch_obj_cpb);

    while(ros::ok() && !fail && !success){
        switch(nextState){

            case SM_INIT:
                {
                    std::cout << stateMachine << "SM_INIT" << std::endl;
                    // JustinaHRI::say("I'm ready for storing groseries test");
                    JustinaHRI::insertAsyncSpeech("I'm ready for storing groseries test", 3000);
                    JustinaHRI::asyncSpeech();
                    nextState = SM_GOTO_CUPBOARD;
                    attempsNavigation = 0;
                    findObjCupboard = false;
                }
                break;

            case SM_GOTO_CUPBOARD:
                {
                    std::cout << stateMachine << "SM_GOTO_CUPBOARD" << std::endl;
                    // JustinaHRI::say("I am going to navigate to the cupboard");
                    JustinaHRI::insertAsyncSpeech("I am going to navigate to the cupboard", 3000);
                    JustinaHRI::asyncSpeech();
                    JustinaManip::startTorsoGoTo(0.2, 0, 0);
                    if(!JustinaNavigation::getClose("cupboard",200000))
                        if(!JustinaNavigation::getClose("cupboard",200000))
                            JustinaNavigation::getClose("cupboard",200000);
                    JustinaHRI::insertAsyncSpeech("I Have reached the cupboard", 3000);
                    JustinaHRI::asyncSpeech();
                    if(!findObjCupboard)
                        nextState = SM_OPEN_DOOR;
                    else
                        nextState = SM_PUT_OBJECT_ON_CUPBOARD;
                }
                break;

            case SM_OPEN_DOOR:
                {
                    std::cout << stateMachine << "SM_OPEN_DOOR" << std::endl;
                    if(!openDoor){
                        JustinaHRI::waitAfterSay("Human can you open the cupboard door please", 3000);
                        findObjCupboard = true;
                        nextState = SM_NAVIGATION_TO_TABLE;
                    }
                    else{
                        JustinaHRI::say("I'm trying to open the cupboard door");
                        JustinaTools::pdfAppend(name_test, "I am tryiang to open the door whitout human help.");
                        if(JustinaTasks::openDoor(false))
                            nextState = SM_PUT_OBJECT_ON_CUPBOARD;
                        else{
                            JustinaHRI::say("I am sorry, I cannot open the door.");
                            nextState = SM_NAVIGATION_TO_TABLE;
                        }
                        nextState = SM_NAVIGATION_TO_TABLE;
                        findObjCupboard = true;
                    }
                }
                break;


            case SM_FIND_OBJECTS_ON_CUPBOARD:
                {
                    std::cout << stateMachine << "SM_FIND_OBJECTS_ON_CUPBOARD" << std::endl;

                    // JustinaHRI::say("I am going to search objects on the shelf");
                    JustinaHRI::insertAsyncSpeech("I am going to search objects on the shelf", 500);
                    JustinaHRI::asyncSpeech();
                    itemsOnCupboard = 0;

                    categories_cpbr.clear();

                    if(!JustinaTasks::alignWithTable(0.45)){
                        JustinaNavigation::moveDist(0.15, 3000);
                        if(!JustinaTasks::alignWithTable(0.45))
                            JustinaTasks::alignWithTable(0.45);
                    }

                    JustinaManip::hdGoTo(0, -0.6, 5000);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                    if(!JustinaVision::detectAllObjectsVot(recoObjList, image, 5))
                        std::cout << "I  can't detect anything" << std::endl;
                    else{
                        std::cout << "I have found " << recoObjList.size() << " objects on the cupboard" << std::endl;
                        itemsOnCupboard = recoObjList.size();
                    }

                    for(int i = 0; i < recoObjList.size(); i++){
                        std::string category;
                        JustinaRepresentation::selectCategoryObjectByName(recoObjList[i].id, category, 0);
                        recoObjList[i].category = category;
                    }
                    
                    temp.str("");
                    temp << "/home/biorobotica/objs/cubpoard" << countFindObjectsOnCupboard++ << "/"; 
                    JustinaTools::saveImageVisionObject(recoObjList, image, temp.str());
                   
                    JustinaTools::getCategoriesFromVisionObject(recoObjList, categories_cpbr);

                    if(itemsOnCupboard > 10)
                        itemsOnCupboard = rand() % 4 + 6;

                    JustinaNavigation::moveDist(-0.15, 3000);


                    int countObject = recoObjList.size();
                    justinaSay.str("");
                    justinaSay << "I have found " << countObject << " objects into cupboard";
                    JustinaHRI::insertAsyncSpeech(justinaSay.str(), 500);

                    justinaSay.str("");
                    justinaSay << "The objects of the cupboard belong to categories";
                    JustinaHRI::insertAsyncSpeech(justinaSay.str(), 500);
                    
                    justinaSay.str("");
                    for(int i = 0; i < categories_cpbr.size(); i++)
                       justinaSay << ", " << categories_cpbr[i];
                    JustinaHRI::insertAsyncSpeech(justinaSay.str(), 500);
                    nextState = SM_NAVIGATION_TO_TABLE;

                    nmbr_objs_fnd_cpb << "I have found " << itemsOnCupboard << " objects into cupboard.";

                    JustinaTools::pdfAppend(name_test, nmbr_objs_fnd_cpb.str());
                    JustinaTools::pdfAppend(name_test, " - Categories found into cupboard: ");
                    for(int i = 0; i < categories_cpbr.size(); i++){
                        std::cout << "Category_" << i << ":  " << categories_cpbr[i] << std::endl;
                        temp.str( std::string() );
                        temp << "      - " << categories_cpbr[i];
                        JustinaTools::pdfAppend(name_test, temp.str());
                    }

                    findObjCupboard = true;
                    attempsPlaceObj = 1;

                    JustinaTools::pdfImageStopRec(name_test,"/home/$USER/objs/");

                    nextState = SM_PUT_OBJECT_ON_CUPBOARD;
                }
                break;

            case SM_NAVIGATION_TO_TABLE:
                {
                    std::cout << stateMachine << "SM_NAVIGATION_TO_TABLE" << std::endl;
                    // JustinaHRI::say("I am going to navigate to the side table");
                    JustinaHRI::insertAsyncSpeech("I am going to navigate to the table", 500);
                    JustinaManip::startLaGoTo("navigation");
                    JustinaManip::startRaGoTo("navigation");
                    
                    JustinaManip::startTorsoGoTo(0.25, 0, 0);

                    JustinaNavigation::startGetClose("table_location");
                    JustinaHRI::asyncSpeech();
                    
                    curr = boost::posix_time::second_clock::local_time();
                    prev = curr;

                    nextState = SM_NAVIGATION_WAIT_REACHED_TO_TABLE;
                    attempsNavigation++;
                }
                break;

            case SM_NAVIGATION_WAIT_REACHED_TO_TABLE:
                {
                    std::cout << stateMachine << "SM_NAVIGATION_WAIT_REACHED_TO_TABLE" << std::endl;
                    if(attempsNavigation > maxAttempsNavigation)
                        nextState = SM_FIND_OBJECTS_ON_TABLE;
                    else{ 
                        curr = boost::posix_time::second_clock::local_time();
                        if(JustinaNavigation::isGlobalGoalReached()){
                            JustinaHRI::insertAsyncSpeech("I arrived to the table", 500);
                            JustinaHRI::asyncSpeech();
                            attempsFindObjectsTable = 0; 
                            alignWithTable = true;
                            attempsNavigation = 0;
                            attempsFindObjectsTable = 0;
                            nextState = SM_FIND_OBJECTS_ON_TABLE;
                        }
                        else if((curr - prev).total_milliseconds() > 200000)
                            nextState = SM_NAVIGATION_TO_TABLE;
                    }
                }
                break;

            case SM_FIND_TABLE:
                {
                    std::cout << "" << std::endl;
                    std::cout << "" << std::endl;
                    std::cout << "----->  State machine: FIND_TABLE" << std::endl;

                    //Append acction to the plan
                    if(!appendPdf){
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
                    std::cout << stateMachine << "SM_FIND_OBJECTS_ON_TABLE" << std::endl;
                    if(attempsFindObjectsTable == 0 && alignWithTable){
                        // JustinaHRI::say("I am going to search objects on the table");
                        //Append acction to the plan
                        JustinaTools::pdfAppend(name_test, fnd_objs_tbl);

                        if(!JustinaTasks::alignWithTable(0.35)){
                            JustinaNavigation::moveDist(0.10, 3000);
                            if(!JustinaTasks::alignWithTable(0.35)){
                                std::cout << "I can´t alignWithTable... :'(" << std::endl;
                                JustinaNavigation::moveDist(-0.15, 3000);
                                alignWithTable = false;
                                break;
                            }
                        }
                    }

                    recoObjForTake.clear();
                    categories_tabl.clear();

                    if(!JustinaVision::detectAllObjectsVot(recoObjForTake, image, 5)){
                        std::cout << "I  can't detect anything" << std::endl;
                        attempsFindObjectsTable++;
                        nextState = SM_FIND_OBJECTS_ON_TABLE;
                    }else{
                        // std::cout << stateMachine << "I have found " << recoObjForTake.size() << " objects on the table" << std::endl;
                        justinaSay.str("");
                        if(recoObjForTake.size() <= 10){
                            int countObject = recoObjForTake.size();
                            justinaSay << "I have found " << countObject << " objects on the table";
                            // JustinaHRI::say(justinaSay.str());
                        }
                        else{
                            // justinaSay << "I have found " << rand() % 4 + 6 << " objects on the table";
                            // JustinaHRI::say(justinaSay.str());
                        }

                        for(int i = 0; i < recoObjForTake.size(); i++){
                             std::size_t found = recoObjForTake[i].id.find("unkown");
                                if(found == std::string::npos){
                                    recoObjForTake[i].confidence = 0.0;      
                                }
                        }
                   
                        std::cout << stateMachine << "Order of the objects to take." << std::endl;
                        std::sort(recoObjForTake.begin(), recoObjForTake.end(),  funCompNearestVisionObject);
                        for(int i = 0; i < recoObjForTake.size(); i++){
                            std::string category;
                            float ecDistObj = sqrt(pow(recoObjForTake[i].pose.position.x, 2) + pow(recoObjForTake[i].pose.position.y ,2) + pow(recoObjForTake[i].pose.position.z ,2));
                            std::cout << stateMachine << "ecDistObj:"  << ecDistObj << ", object:" << recoObjForTake[i].id << std::endl;
                            // JustinaRepresentation::selectCategoryObjectByName(recoObjForTake[i].id, category, 0);
                            float confidence = recoObjForTake[i].confidence; 
                            confidence *= (float)(recoObjForTake.size() - i) / (float) recoObjForTake.size();
                            
                            std::size_t found = recoObjForTake[i].id.find("unkown");
                            if(found == std::string::npos){
                                JustinaRepresentation::insertConfidenceAndGetCategory(recoObjForTake[i].id, i, confidence, category, 0);
                                recoObjForTake[i].category = category;
                            }
                        }
                    
                        std::cout << stateMachine << "Saving objs recog." << std::endl;
                        temp.str("");
                        temp << "/home/biorobotica/objs/table" << countFindObjectsOnTable++ << "/"; 
                        JustinaTools::saveImageVisionObject(recoObjForTake, image, temp.str());
                            
                        //Append acction to the plan
                        JustinaTools::pdfAppend(name_test, justinaSay.str());
                        JustinaTools::getCategoriesFromVisionObject(recoObjForTake, categories_tabl);
                        JustinaTools::pdfAppend(name_test, " - Categories found on the table: ");
                        for(int i = 0; i < categories_tabl.size(); i++){
                            std::cout << "Category_" << i << ":  " << categories_tabl[i] << std::endl;
                            temp.str( std::string() );
                            temp << "      - " << categories_tabl[i];
                            JustinaTools::pdfAppend(name_test, temp.str());
                        }

                        justinaSay.str( std::string() );
                        justinaSay << "The objects of the table belong to categories...";
                        //JustinaHRI::say(justinaSay.str());
                        //boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                            
                        /*for(int i = 0; i < categories_tabl.size(); i++){
                            justinaSay.str("");
                            justinaSay << categories_tabl[i];
                            JustinaHRI::say(justinaSay.str());
                            boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                        }*/
                        JustinaTools::pdfImageStopRec(name_test,"/home/$USER/objs/");
                        nextState = SM_INF_TAKE_OBJECT;
                    }
                }
                break;

            case SM_INF_TAKE_OBJECT:
                {
                    std::cout << stateMachine << "SM_INF_TAKE_OBJECT" << std::endl;
                    // Here is to the inference to take a object.
                    int index1, index2;
                    JustinaRepresentation::selectTwoObjectsToGrasp(index1, index2, 0);
                    std::cout << stateMachine << "Obj1:" << recoObjForTake[index1].id << ", Obj2:" << recoObjForTake[index2].id << std::endl;

                    // Here is to get wich of the two objects is optimal to grasp with left or right arm
                    float y1 = recoObjForTake[index1].pose.position.y;
                    float y2 = recoObjForTake[index2].pose.position.y;

                    if(onlyOneGraspObject){
                        takeLeft = false;
                        takeRight = true;
                    }
                    else{
                        takeLeft = true;
                        takeRight = true;
                    }

                    if(y1 > 0 && y2 < 0 || y1 < 0 && y2 > 0){
                        if(y1 > 0){
                            idObjectGraspLeft = recoObjForTake[index1].id;
                            idObjectGraspRight = recoObjForTake[index2].id;
                            indexObjectGraspLeft = index1;
                            indexObjectGraspRight = index2;
                        }
                        else if(y2 > 0){
                            idObjectGraspLeft = recoObjForTake[index2].id;
                            idObjectGraspRight = recoObjForTake[index1].id;
                            indexObjectGraspLeft = index2;
                            indexObjectGraspRight = index1;
                        }
                    }
                    else if(y1 <= 0 && y2 <= 0){
                        if(y1 <= y2){
                            idObjectGraspLeft = recoObjForTake[index2].id;
                            idObjectGraspRight = recoObjForTake[index1].id;
                            indexObjectGraspLeft = index2;
                            indexObjectGraspRight = index1;
                        }
                        else if(y1 >= y2){
                            idObjectGraspLeft = recoObjForTake[index1].id;
                            idObjectGraspRight = recoObjForTake[index2].id;
                            indexObjectGraspLeft = index1;
                            indexObjectGraspRight = index2;
                        }
                    }
                    else if(y1 >= 0 && y2 >= 0){
                        if(y1 >= y2){
                            idObjectGraspLeft = recoObjForTake[index1].id;
                            idObjectGraspRight = recoObjForTake[index2].id;
                            indexObjectGraspLeft = index1;
                            indexObjectGraspRight = index2;
                        }
                        else if(y1 <= y2){
                            idObjectGraspLeft = recoObjForTake[index2].id;
                            idObjectGraspRight = recoObjForTake[index1].id;
                            indexObjectGraspLeft = index2;
                            indexObjectGraspRight = index1;
                        }
                    }
                    if(takeRight || takeLeft){
                        useLastPoseGrasp = true;
                        attempsGraspObject = 0;
                        alignWithTable = true;
                        nextState = SM_TAKE_OBJECT;
                    }
                    else{
                        attempsFindObjectsTable = 0;
                        nextState = SM_FIND_OBJECTS_ON_TABLE;
                    }
                }
                break;

            case SM_TAKE_OBJECT:
                {
                    std::cout << stateMachine << "SM_TAKE_OBJECT" << std::endl;
                    if (attempsGraspObject < maxAttempsTakeObject && (takeLeft || takeRight)){
                        if(!JustinaTasks::alignWithTable(0.35) && alignWithTable){
                            std::cout << "I can´t align with table   :´(" << std::endl;
                            JustinaNavigation::moveDistAngle(-0.05, M_PI_4/4, 2000);
                            JustinaTasks::alignWithTable(0.35);
                            JustinaTasks::alignWithTable(0.35);
                            JustinaTasks::alignWithTable(0.35);
                            alignWithTable = false;
                            break;
                        }
                        else{
                            bool withLeftOrRightArm;
                            if(takeRight){
                                withLeftOrRightArm = false;
                                indexObjectGrasp = indexObjectGraspRight;
                                if(attempsGraspObject == 0)
                                    JustinaHRI::say("I am going to take object whit my right arm");
                            }else if(takeLeft){
                                withLeftOrRightArm = true;
                                indexObjectGrasp = indexObjectGraspLeft;
                                if(attempsGraspObject == 0)
                                    JustinaHRI::say("I am going to take object whit my left arm");
                            }
                                
                            std::string idObjectGrasp = recoObjForTake[indexObjectGrasp].id;
                            geometry_msgs::Pose pose = recoObjForTake[indexObjectGrasp].pose;
                            if(idObjectGrasp.compare("unkown") != 0){
                                if(JustinaTasks::moveActuatorToGrasp(pose.position.x, pose.position.y, pose.position.z, withLeftOrRightArm, idObjectGrasp)){
                                    if(!withLeftOrRightArm){
                                        objectGrasped[0] = true;
                                        takeRight = false;
                                        nextState = SM_TAKE_OBJECT;
                                    }
                                    else{
                                        objectGrasped[1] = true;
                                        takeLeft = false;
                                        nextState = SM_GOTO_CUPBOARD;
                                    }
                                    attempsGraspObject = 0;
                                    alignWithTable = true;
                                    graspObject = true;
                                    useLastPoseGrasp = false;
                                }
                                else{
                                    std::cout << "I can´t grasp objects in " << attempsGraspObject << " attempt" << std::endl;
                                    if(JustinaTasks::findObject(idObjectGrasp, pose, leftArm)){
                                        recoObjForTake[indexObjectGrasp].pose = pose;
                                        useLastPoseGrasp = true;
                                    }
                                    else
                                        useLastPoseGrasp = false;
                                    attempsGraspObject++;
                                }
                            }
                            else{ 
                                if(JustinaTasks::graspObject(pose.position.x, pose.position.y, pose.position.z, withLeftOrRightArm, "", false)){
                                    if(!withLeftOrRightArm){
                                        takeRight = false;
                                        nextState = SM_TAKE_OBJECT;
                                    }
                                    else{
                                        takeLeft = false;
                                        nextState = SM_GOTO_CUPBOARD;
                                    }
                                    graspObject = true;
                                }
                                else{
                                    if(!withLeftOrRightArm){
                                        takeRight = false;
                                        nextState = SM_TAKE_OBJECT;
                                    }
                                    else{
                                        takeLeft = false;
                                        nextState = SM_FIND_OBJECTS_ON_TABLE;
                                    }
                                }
                                attempsGraspObject = 0;
                                alignWithTable = true;
                                useLastPoseGrasp = false;
                            }
                        }
                    }
                    else{
                        if(useLastPoseGrasp){
                            std::string idObjectGrasp = recoObjForTake[indexObjectGrasp].id;
                            geometry_msgs::Pose pose = recoObjForTake[indexObjectGrasp].pose;
                            bool withLeftOrRightArm;
                            if(takeRight)
                                withLeftOrRightArm = false;
                            else if(takeLeft)
                                withLeftOrRightArm = true;
                            if(JustinaTasks::graspObject(pose.position.x, pose.position.y, pose.position.z, withLeftOrRightArm, "", false))
                                graspObject = true;

                        }

                        if(takeRight && takeLeft){
                            takeRight = false;
                            nextState = SM_TAKE_OBJECT;
                        }
                        else if(!takeRight && takeLeft){
                            takeLeft = false;
                            nextState = SM_TAKE_OBJECT;
                        }
                        else if(takeRight && !takeLeft){
                            takeRight = false;
                            nextState = SM_TAKE_OBJECT;
                        }
                        if(!takeLeft && !takeRight){
                            if(graspObject)
                                nextState = SM_GOTO_CUPBOARD;
                            else{
                                attempsFindObjectsTable = 0;
                                nextState = SM_FIND_OBJECTS_ON_TABLE;
                            }
                        }
                        attempsGraspObject = 0;
                        alignWithTable = true;
                    }
                }
                break;

            case SM_PUT_OBJECT_ON_CUPBOARD:
                {
                    std::cout << stateMachine << "SM_PUT_OBJECT_ON_CUPBOARD" << std::endl;
                    bool withLeftOrRightArm;
                    if(objectGrasped[0] && JustinaManip::objOnRightHand())
                        withLeftOrRightArm = false;
                    else if(objectGrasped[1] && JustinaManip::objOnLeftHand())
                        withLeftOrRightArm = true;
                    if(objectGrasped[0] || objectGrasped[1]){
                        if(attempsPlaceObj < maxAttempsPlaceObj){
                            if(!JustinaTasks::alignWithTable(0.35)){
                                JustinaNavigation::moveDist(-0.10, 3000);
                                boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
                                if(!JustinaTasks::alignWithTable(0.35)){
                                    JustinaNavigation::moveDist(0.15, 3000);
                                    boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
                                    JustinaTasks::alignWithTable(0.35);
                                }
                            }
                            if(JustinaTasks::placeObjectOnShelfHC(withLeftOrRightArm)){
                                if(!withLeftOrRightArm)
                                    objectGrasped[0] = false;
                                else
                                    objectGrasped[1] = false;
                                attempsPlaceObj = 0;
                            }
                            attempsPlaceObj++;
                        }
                        else{
                            attempsPlaceObj = 1;
                            std::cout << "I can´t placed objects on cupboard whit right Arm" << std::endl;
                            JustinaHRI::say("I can´t found a free place in the cupboard");
                            if(objectGrasped[0] && JustinaManip::objOnRightHand())
                                takeRight = false;
                            else
                                takeRight = true;
                            if(objectGrasped[1] && JustinaManip::objOnLeftHand())
                                takeLeft = false;
                            else
                                takeLeft = true;

                            if(objectGrasped[0] && objectGrasped[1])
                                nextState = SM_PUT_OBJECT_ON_CUPBOARD;
                            else{
                                nextState = SM_NAVIGATION_TO_TABLE;
                                attempsNavigation = 0;
                            }
                        }
                    }
                    else{
                        attempsPlaceObj = 1;
                        nextState = SM_NAVIGATION_TO_TABLE;
                    }
                }
                break;

            default:{
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
