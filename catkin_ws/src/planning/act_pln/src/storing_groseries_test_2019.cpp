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
#define SM_FIND_AND_TAKE_OBJECTS  51
#define SM_TAKE_OBJECT_RIGHT 55
#define SM_TAKE_OBJECT_LEFT 60
#define SM_GOTO_CUPBOARD         70
#define SM_WAIT_FOR_COMMAND      71
#define SM_PARSE_SPOKEN_COMMAND  72
#define SM_WAIT_FOR_CONFIRMATION 73
#define SM_SIMUL                 74
#define SM_OPEN_DOOR 75
#define SM_FIND_OBJECTS_ON_CUPBOARD 80
#define SM_PUT_OBJECT_ON_CUPBOARD 100
#define SM_FINISH_TEST 110
#define SM_PLACE_ARMS 120

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
    bool useLastPoseGrasp = false;
    bool graspObject = false;

    bool objectGrasped[2] = {false, false};
    std::string objectGraspedObj[2] = {"", ""};
    std::string objectGraspedCat[2] = {"", ""};

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
    bool onlyOneGraspObject = false;
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
    // This is for the number of the objects knowns
    int countKnownObjects = 0;

    //This flag is for the behavior of the take object
    // 0 is to find and tale, 1 is for only find one attempt
    int behaviorTake = 1;

    // This is for have a time to reached to the table
	boost::posix_time::ptime curr;
	boost::posix_time::ptime prev;

    sensor_msgs::Image image;

    //////// CHANGE THE NAME THE PDF         ///////

    // Strings for append to pdf file.
    std::string name_test = "storingGroseries_1";

    /////*******************************//////

    std::string lastRecoSpeech;
    std::vector<std::string> validCommands;
    validCommands.push_back("level one");
    validCommands.push_back("level two");
    validCommands.push_back("level three");

    int arm = 0;
    int level_in_[2];
    bool ask;

    std::vector<std::string> categories;
    std::vector<int>              level;
    
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

    // This is for generate PDF
    /*JustinaTools::pdfStart(name_test);
    JustinaTools::pdfAppend(name_test, "");
    JustinaTools::pdfAppend(name_test, "Attempt:  3");
    JustinaTools::pdfAppend(name_test, "");
    JustinaTools::pdfAppend(name_test, "");
    JustinaTools::pdfAppend(name_test, "------- PLANES -----");
    JustinaTools::pdfAppend(name_test, nv_cpb);
    JustinaTools::pdfAppend(name_test, cnt_od);
    JustinaTools::pdfAppend(name_test, "I am trying to open the cupboards door");
    JustinaTools::pdfAppend(name_test, srch_obj_cpb);*/


/*    nextState = 71;				
	objectGrasped[0] = true;
	objectGrasped[1] = true; 
	objectGraspedCat[0] = "drinks";
	objectGraspedCat[1] = "snacks";	//*/

    while(ros::ok() && !fail && !success){
        switch(nextState){

            case SM_INIT:
                {
                    std::cout << stateMachine << "SM_INIT" << std::endl;
                    // JustinaHRI::say("I'm ready for storing groseries test");
                    // JustinaHRI::insertAsyncSpeech("I'm ready for storing groseries test", 3000);
                    // JustinaHRI::asyncSpeech();
                    nextState = SM_OPEN_DOOR;
                    attempsNavigation = 0;
                    findObjCupboard = false;
                }
                break;

            case SM_GOTO_CUPBOARD:
                {
                    std::cout << stateMachine << "SM_GOTO_CUPBOARD" << std::endl;
                    // JustinaHRI::say("I am going to navigate to the cupboard");
                    //JustinaHRI::insertAsyncSpeech("I am going to navigate to the cupboard", 3000);
                    //JustinaHRI::asyncSpeech();
                    JustinaManip::startTorsoGoTo(0.1, 0, 0);
                    if(!JustinaNavigation::getClose("cupboard",200000))
                        if(!JustinaNavigation::getClose("cupboard",200000))
                            JustinaNavigation::getClose("cupboard",200000);
                    //JustinaHRI::insertAsyncSpeech("I Have reached the cupboard", 3000);
                    //JustinaHRI::asyncSpeech();
                    if(!findObjCupboard)
                        nextState = SM_OPEN_DOOR;
                    else
                        nextState = SM_WAIT_FOR_COMMAND;
                }
                break;

            case SM_OPEN_DOOR:
                {
                    std::cout << stateMachine << "SM_OPEN_DOOR" << std::endl;
                    if(!openDoor){
                        JustinaHRI::waitAfterSay("Human can you open the cupboard door please", 6000);
                        findObjCupboard = true;
                        nextState = SM_NAVIGATION_TO_TABLE;
                    }
                    else{
                        JustinaHRI::say("I'm trying to open the cupboard door");
                        // This is for generate PDF
                        // JustinaTools::pdfAppend(name_test, "I am tryiang to open the door whitout human help.");
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
                    // JustinaHRI::insertAsyncSpeech("I am going to search objects on the shelf", 500);
                    //JustinaHRI::asyncSpeech();
                    itemsOnCupboard = 0;

                    categories_cpbr.clear();

                    /*if(!JustinaTasks::alignWithTable(0.45)){
                        JustinaNavigation::moveDist(0.15, 3000);
                        if(!JustinaTasks::alignWithTable(0.45))
                            JustinaTasks::alignWithTable(0.45);
                    }*/

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
                    // JustinaHRI::insertAsyncSpeech(justinaSay.str(), 500);
                    JustinaHRI::say(justinaSay.str());

                    justinaSay.str("");
                    justinaSay << "The objects of the cupboard belong to categories";
                    // JustinaHRI::insertAsyncSpeech(justinaSay.str(), 500);
                    JustinaHRI::say(justinaSay.str());
                    
                    justinaSay.str("");
                    for(int i = 0; i < categories_cpbr.size(); i++)
                       justinaSay << ", " << categories_cpbr[i];
                    // JustinaHRI::insertAsyncSpeech(justinaSay.str(), 500);
                    JustinaHRI::say(justinaSay.str());
                    nextState = SM_NAVIGATION_TO_TABLE;

                    nmbr_objs_fnd_cpb << "I have found " << itemsOnCupboard << " objects into cupboard.";

                    // This is for generate PDF
                    /*JustinaTools::pdfAppend(name_test, nmbr_objs_fnd_cpb.str());
                    JustinaTools::pdfAppend(name_test, " - Categories found into cupboard: ");
                    for(int i = 0; i < categories_cpbr.size(); i++){
                        std::cout << "Category_" << i << ":  " << categories_cpbr[i] << std::endl;
                        temp.str( std::string() );
                        temp << "      - " << categories_cpbr[i];
                        JustinaTools::pdfAppend(name_test, temp.str());
                    }*/

                    findObjCupboard = true;
                    attempsPlaceObj = 1;

                    // This is for generate PDF
                    //JustinaTools::pdfImageStopRec(name_test,"/home/$USER/objs/");

                    nextState = SM_PUT_OBJECT_ON_CUPBOARD;
                }
                break;

            case SM_NAVIGATION_TO_TABLE:
                {
                    std::cout << stateMachine << "SM_NAVIGATION_TO_TABLE" << std::endl;
                    // JustinaHRI::say("I am going to navigate to the dining table");
                    // JustinaHRI::insertAsyncSpeech("I am going to navigate to the table", 500);
                    JustinaManip::startTorsoGoTo(0.1, 0.0, 0.0);
                    JustinaManip::startLaGoTo("navigation");
                    JustinaManip::startRaGoTo("navigation");
                    
                    JustinaNavigation::startGetClose("table_location");
                    //JustinaHRI::asyncSpeech();
                    
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
                            //JustinaHRI::insertAsyncSpeech("I arrived to the table", 500);
                            //JustinaHRI::asyncSpeech();
                            attempsFindObjectsTable = 0; 
                            alignWithTable = true;
                            attempsNavigation = 0;
                            attempsFindObjectsTable = 0;
                            nextState = SM_FIND_OBJECTS_ON_TABLE;
                        }
                        else if((curr - prev).total_milliseconds() > 20000)
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
                    JustinaManip::startTorsoGoTo(0.0, 0.0, 0.0);
                    if(attempsFindObjectsTable == 0 && alignWithTable){
                        JustinaHRI::say("I am going to search objects on the table");
                        std::cout<< "I am going to search objects on the table"<<std::endl;
                        //Append acction to the plan
                        // This is for generate PDF
                        // JustinaTools::pdfAppend(name_test, fnd_objs_tbl);
                        JustinaTasks::alignWithTable(0.35);
                    }

                    recoObjForTake = std::vector<vision_msgs::VisionObject>();
                    categories_tabl = std::vector<std::string>();

                    if(!JustinaVision::detectAllObjectsVot(recoObjForTake, image, 5)){
                        if(objectGrasped[0] || objectGrasped[1])
                            nextState = SM_GOTO_CUPBOARD;
                        else{
                            std::cout << "I  can't detect anything" << std::endl;
                            attempsFindObjectsTable++;
                            nextState = SM_FIND_OBJECTS_ON_TABLE;
                        }
                    }else{
                        // std::cout << stateMachine << "I have found " << recoObjForTake.size() << " objects on the table" << std::endl;
                        if(recoObjForTake.size() == 0){
                            if(objectGrasped[0] || objectGrasped[1])
                                nextState = SM_GOTO_CUPBOARD;
                            else{
                                std::cout << "I  can't detect anything" << std::endl;
                                attempsFindObjectsTable++;
                                nextState = SM_FIND_OBJECTS_ON_TABLE;
                            }
                        }
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
                             std::size_t found = recoObjForTake[i].id.find("unknown");
                                if(found != std::string::npos){
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
                            
                            std::size_t found = recoObjForTake[i].id.find("unknown");
                            countKnownObjects = 0;
                            if(found == std::string::npos){
                                JustinaRepresentation::insertConfidenceAndGetCategory(recoObjForTake[i].id, i, confidence, category, 0);
                                recoObjForTake[i].category = category;
                                countKnownObjects++;
                            }
                        }
                    
                        // This is for saving the images in this test is nt necesary
                        /*std::cout << stateMachine << "Saving objs recog." << std::endl;
                        temp.str("");
                        temp << "/home/biorobotica/objs/table" << countFindObjectsOnTable++ << "/";
                        JustinaTools::saveImageVisionObject(recoObjForTake, image, temp.str());*/
                            
                        //Append acction to the plan
                        // This is for generate PDF
                        //JustinaTools::pdfAppend(name_test, justinaSay.str());
                        JustinaTools::getCategoriesFromVisionObject(recoObjForTake, categories_tabl);
                        for(int i = 0; i < categories_tabl.size(); i++)
                            std::cout << "Category_" << i << ":  " << categories_tabl[i] << std::endl;
                        // This is for generate PDF
                        /*JustinaTools::pdfAppend(name_test, " - Categories found on the table: ");
                        for(int i = 0; i < categories_tabl.size(); i++){
                            std::cout << "Category_" << i << ":  " << categories_tabl[i] << std::endl;
                            temp.str( std::string() );
                            temp << "      - " << categories_tabl[i];
                            JustinaTools::pdfAppend(name_test, temp.str());
                        }*/

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
                        // This is for generate PDF
                        // JustinaTools::pdfImageStopRec(name_test,"/home/$USER/objs/");
                        nextState = SM_INF_TAKE_OBJECT;
                    }
                }
                break;

            case SM_INF_TAKE_OBJECT:
                {
                    std::cout << stateMachine << "SM_INF_TAKE_OBJECT" << std::endl;
                    // Here is to the inference to take a object.
                    int index1, index2;
                    if(recoObjForTake.size() > 1 && !objectGrasped[0] && !objectGrasped[1]){
                        
                        if(countKnownObjects > 1)
                            JustinaRepresentation::selectTwoObjectsToGrasp(index1, index2, 0);
                        else{
                            if(countKnownObjects == 1){
                                JustinaRepresentation::selectTwoObjectsToGrasp(index1, index2, 0);
                                for(int i = 0; i < recoObjForTake.size(); i++){
                                    if(i != index1)
                                        index2 = i;
                                }
                            }
                            else{
                                index1 = 0;
                                index2 = 1;
                            }
                        }

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
                            /***********************
                             * This is for not align with the table when grasping
                             * *********************/
                            //alignWithTable = false;
                            if(behaviorTake)
                                nextState = SM_TAKE_OBJECT;
                            else
                                nextState = SM_FIND_AND_TAKE_OBJECTS;
                        }
                        else{
                            attempsFindObjectsTable = 0;
                            nextState = SM_FIND_OBJECTS_ON_TABLE;
                        }
                    }
                    else{
                        if(recoObjForTake.size() >= 1 && countKnownObjects >= 1)
                            JustinaRepresentation::selectTwoObjectsToGrasp(index1, index2, 0);
                        else
                            index1 = 0;

                        float y1 = recoObjForTake[index1].pose.position.y;

                        if(!objectGrasped[0] && !objectGrasped[1]){
                            if(y1 > 0){
                                takeLeft = true;
                                takeRight = false;
                                indexObjectGraspRight = 0;
                            }
                            else{
                                takeLeft = false;
                                takeRight = true;
                                indexObjectGraspLeft = 0;
                            }
                        }
                        else{
                            if(y1 > 0 && !objectGrasped[1]){
                                takeLeft = true;
                                takeRight = false;
                                indexObjectGraspLeft = 0;
                            }
                            else if(y1 > 0 && objectGrasped[1]){
                                takeLeft = false;
                                takeRight = true;
                                indexObjectGraspRight = 0;
                            }
                            if(y1 <= 0 && !objectGrasped[0]){
                                takeLeft = false;
                                takeRight = true;
                                indexObjectGraspRight = 0;
                            }
                            else if(y1 <= 0 && objectGrasped[0]){
                                takeLeft = true;
                                takeRight = false;
                                indexObjectGraspLeft = 0;
                            }
                        }
                        nextState = SM_TAKE_OBJECT;
                    }
                }
                break;

            case SM_FIND_AND_TAKE_OBJECTS:
                {
                    std::cout << stateMachine << "SM_FIND_AND_TAKE_OBJECTS" << std::endl;
                    if (attempsGraspObject < maxAttempsTakeObject && (takeLeft || takeRight)){
                        if(!JustinaTasks::alignWithTable(0.35) && alignWithTable){
                            std::cout << "I can´t align with table   :´(" << std::endl;
                            JustinaNavigation::moveDistAngle(-0.05, 0, 2000);
                            JustinaTasks::alignWithTable(0.35);
                            JustinaTasks::alignWithTable(0.35);
                            alignWithTable = false;
                            break;
                        }
                        else{
                            bool withLeftOrRightArm;
                            justinaSay.str("");
                            if(takeRight){
                                withLeftOrRightArm = false;
                                indexObjectGrasp = indexObjectGraspRight;
                                if(attempsGraspObject == 0)
                                    justinaSay << "I am going to take the " << recoObjForTake[indexObjectGrasp].id << " with my right arm";
                            }else if(takeLeft){
                                withLeftOrRightArm = true;
                                indexObjectGrasp = indexObjectGraspLeft;
                                if(attempsGraspObject == 0)
                                    justinaSay << "I am going to take the " << recoObjForTake[indexObjectGrasp].id << " with my left arm";
                            }
                            
                            std::cout<< justinaSay <<std::endl;
                            JustinaHRI::say(justinaSay.str());
                                
                            std::string idObjectGrasp = recoObjForTake[indexObjectGrasp].id;
                            geometry_msgs::Pose pose = recoObjForTake[indexObjectGrasp].pose;
                            if(idObjectGrasp.compare("unknown") != 0){
                                //if(JustinaTasks::moveActuatorToGrasp(pose.position.x, pose.position.y, pose.position.z, withLeftOrRightArm, idObjectGrasp)){
                                if(JustinaTasks::graspObject(pose.position.x, pose.position.y, pose.position.z, withLeftOrRightArm, idObjectGrasp, true)){
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
                                    /**********************
                                     * This is for skip the grasping object if fail in the first attempt
                                     * This test in TMR 2019
                                     * ********************/
                                    /*if(!withLeftOrRightArm){
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
                                    useLastPoseGrasp = false;*/
                                    
                                    /**********************
                                     * This is for find again a object when fail the first attempt
                                     *********************/
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
                                if(JustinaTasks::graspObject(pose.position.x, pose.position.y, pose.position.z, withLeftOrRightArm, "", true)){
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
                            if(JustinaTasks::graspObject(pose.position.x, pose.position.y, pose.position.z, withLeftOrRightArm, "", true))
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

            case SM_TAKE_OBJECT:
                {
                    std::cout << stateMachine << "SM_TAKE_OBJECT" << std::endl;
                    bool withLeftOrRightArm;
                    justinaSay.str("");
                    if(takeRight){
                        withLeftOrRightArm = false;
                        indexObjectGrasp = indexObjectGraspRight;
                        if(attempsGraspObject == 0)
                            justinaSay << "I am going to take the " << recoObjForTake[indexObjectGrasp].id << " with my right arm";
                    }else if(takeLeft){
                        withLeftOrRightArm = true;
                        indexObjectGrasp = indexObjectGraspLeft;
                        if(attempsGraspObject == 0)
                            justinaSay << "I am going to take the " << recoObjForTake[indexObjectGrasp].id << " with my left arm";
                    }

                    JustinaHRI::say(justinaSay.str());
                    std::string idObjectGrasp = recoObjForTake[indexObjectGrasp].id;
                    std::string catObjectGrasp;
                    if(idObjectGrasp.find("unknown") == std::string::npos)
                        catObjectGrasp = recoObjForTake[indexObjectGrasp].category;
                    else
                        catObjectGrasp = "unknown";
                    geometry_msgs::Pose pose = recoObjForTake[indexObjectGrasp].pose;
                    if(JustinaTasks::graspObject(pose.position.x, pose.position.y, pose.position.z, withLeftOrRightArm, "", true)){
                        if(!withLeftOrRightArm){
                            objectGrasped[0] = true;
                            objectGraspedObj[0] = idObjectGrasp;
                            objectGraspedCat[0] = catObjectGrasp;
                            takeRight = false;
                            nextState = SM_FIND_OBJECTS_ON_TABLE;
                        }
                        else{
                            objectGrasped[1] = true;
                            objectGraspedObj[1] = idObjectGrasp;
                            objectGraspedCat[1] = catObjectGrasp;
                            takeLeft = false;
                            nextState = SM_GOTO_CUPBOARD;
                        }
                        alignWithTable = true;
                        attempsFindObjectsTable = 0;
                    }
                    else{
                        if(!withLeftOrRightArm){
                            objectGrasped[0] = false;
                            takeRight = true;
                            nextState = SM_FIND_OBJECTS_ON_TABLE;
                        }
                        else{
                            attempsGraspLeft++;
                            if(attempsGraspLeft >= maxAttempsGraspLeft){
                                if(objectGrasped[0]){
                                    takeLeft = false;
                                    nextState = SM_GOTO_CUPBOARD;
                                }
                                else
                                    nextState = SM_FIND_OBJECTS_ON_TABLE;
                            }
                            else{
                                objectGrasped[1] = false;
                                takeLeft = true;
                                nextState = SM_FIND_OBJECTS_ON_TABLE;
                            }
                        }
                        alignWithTable = true;
                        attempsFindObjectsTable = 0;
                    }
                }
                break;

            case SM_PUT_OBJECT_ON_CUPBOARD:
                {
                    std::cout << stateMachine << "SM_PUT_OBJECT_ON_CUPBOARD" << std::endl;
                    for(int i=0; i< categories.size(); i++) //----------------
                    {
                    	std::cout<< categories[i]<<" in the level : "<<level[i]<<std::endl;
                    }//----------------------
                    bool withLeftOrRightArm;
                    /********************
                     * This is only for ensure that justina have a object in the hand
                     * *****************/
                    /*if(objectGrasped[0])
                        withLeftOrRightArm = false;
                    else if(objectGrasped[1])
                        withLeftOrRightArm = true;*/
                    if(objectGrasped[0]){
                    	if(JustinaManip::objOnRightHand())
                        	withLeftOrRightArm = false;
                    	else
                    		objectGrasped[0] = false;
                    }else if(objectGrasped[1]){
                    	if(JustinaManip::objOnLeftHand())
                    		withLeftOrRightArm = true;
                    	else
                    		objectGrasped[1] = false;
                    }
                    if(objectGrasped[0] || objectGrasped[1]){
                        if(attempsPlaceObj < maxAttempsPlaceObj){
                            /******  This is for the aligin with the table *****/
                            if(!JustinaTasks::alignWithTable(0.35)){
                                JustinaNavigation::moveDist(-0.10, 3000);
                                if(!JustinaTasks::alignWithTable(0.35)){
                                    JustinaNavigation::moveDist(0.15, 3000);
                                    JustinaTasks::alignWithTable(0.35);
                                }
                            }
                            /*
                            if(JustinaTasks::placeObjectOnShelfHC(withLeftOrRightArm, 2)){
                                if(!withLeftOrRightArm)
                                    objectGrasped[0] = false;
                                else
                                    objectGrasped[1] = false;
                                attempsPlaceObj = 0;
                            }//*/
                            for(int i=0; i < categories.size(); i++)//-----------------
                                std::cout<< categories[i]<<" in the level -> "<<level[i]<<std::endl;
                            
                            for(int i=0; i < categories.size(); i++) 
                            {
                                if(categories[i] == objectGraspedCat[0])
                                {
                                    JustinaTasks::placeObjectOnShelfHC(0,level[i]);
                                }
                                if(categories[i] == objectGraspedCat[1])
                                {
                                    JustinaTasks::placeObjectOnShelfHC(1,level[i]);
                                }
                                nextState = SM_NAVIGATION_TO_TABLE;
                            }//----------------------//*/
                            attempsPlaceObj++;
                        }
                        else{
                            attempsPlaceObj = 1;
                            std::cout << "I can´t placed objects on cupboard whit right Arm" << std::endl;
                            JustinaHRI::say("I can´t found a free place in the cupboard");
                            //if(objectGrasped[0])
                            /********************
                             * This is only for ensure that justina have a object in the hand
                             * *****************/
                            if(objectGrasped[0] && JustinaManip::objOnRightHand())
                                takeRight = false;
                            else
                                takeRight = true;
                            //if(objectGrasped[1])
                            /********************
                             * This is only for ensure that justina have a object in the hand
                             * *****************/
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
            //----------------------------------------------------------------------------------------------------------------------------------
            
            case SM_WAIT_FOR_COMMAND: 
                {
                    std::cout << stateMachine << "---------------------------SM_WAIT_FOR_COMMAND-------------------------"<< std::endl; 				

                    if(objectGrasped[arm])
                    {   
                    	if(categories.size() == 0)       
                    		ask = true;       
						else
						{
	                    	for(int i=0; i< categories.size(); i++)
	                    	{
	                    		if(categories[i] != objectGraspedCat[arm])
	                    		{
	                    			ask = true;
	                    			std::cout<<categories[i] <<" : "<<objectGraspedCat[arm];
	                    		}
	                    		else
	                    		{
	                    			ask = false;
	                    			break;
	                    		}
	                    	} 
	                    }//From else (categories.size != 0)		
                    	if(ask)
                    	{
	                        justinaSay.str("");
                            justinaSay << "Could you tell me at what level to store the " << objectGraspedCat[arm]<< ", For example level one, level two or level three";
	                        std::cout <<  "\nCould you tell me at what level to store the " << objectGraspedCat[arm]<< ", For example level one, level two or level three"<<std::endl;
	                        JustinaHRI::enableSpeechRecognized(false);
                            JustinaHRI::waitAfterSay(justinaSay.str(), 10000);
                            JustinaHRI::enableSpeechRecognized(true);
	                        if(!JustinaHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 10000))
	                        {
	                            nextState = SM_WAIT_FOR_COMMAND;
	                        }
	                        else
	                        {
	                            std::cout << "Parsing word..." << std::endl;
	                            nextState = SM_PARSE_SPOKEN_COMMAND;
	                        }
	                    }
	                    else
	                    {
	                        arm++;
	                    	if(arm > 1)
	                    	{
	                    		arm = 0;
	                    		//nextState = SM_PUT_OBJECT_ON_CUPBOARD;
	                    		nextState = SM_SIMUL;
	                    	}
                        }
                    }//From if (objectGrasped[arm])
                    else
                    {
                        arm++;
                    	if(arm > 1)
                    	{
                    		arm = 0;
                    		//nextState = SM_PUT_OBJECT_ON_CUPBOARD;
                    		nextState = SM_SIMUL;
                    	}
                    }
                }
                break;
            case SM_PARSE_SPOKEN_COMMAND:
                {      
                    std::cout << stateMachine << "SM_PARSE_SPOKEN_COMMAND" << std::endl;
                    
                    justinaSay.str("");       
                    if(lastRecoSpeech.find("level one") != std::string::npos)
                        level_in_[arm] = 1;
                    else if(lastRecoSpeech.find("level two") != std::string::npos)
                        level_in_[arm] = 2;  
                    else if(lastRecoSpeech.find("level three") != std::string::npos)
                        level_in_[arm] = 3;
                    else
                    {
                        JustinaHRI::waitAfterSay(" Sorry, I did not understand you ", 6000);  
                        std::cout << "Sorry, I did not understand you"<< std::endl;                 
                        nextState = SM_WAIT_FOR_COMMAND;
                    }

                    justinaSay << "Ok, I am going to store the " <<objectGraspedCat[arm]<< " on the level "<< level_in_[arm];               
                    std::cout << "Ok, I am going to store the " <<objectGraspedCat[arm]<< " on the level "<< level_in_[arm] << std::endl;
                    JustinaHRI::waitAfterSay(justinaSay.str(), 7000);

                    categories.push_back(objectGraspedCat[arm]);
                    level.push_back(level_in_[arm]);
                    
                    //std::cout<<"arm-----------------------> " <<arm<<std::endl;
                    arm++;
                    if(arm <= 1){
                        nextState = SM_WAIT_FOR_COMMAND;
                    }
                    else
                    {
                        arm = 0;
                        //nextState = SM_PUT_OBJECT_ON_CUPBOARD;
                        nextState = SM_SIMUL;
                    }
            }                
            break;
            
/*            case SM_SIMUL:
            {

                for(int i=0; i < categories.size(); i++)
                    std::cout<< categories[i]<<" in the level -> "<<level[i]<<std::endl;
                
		        JustinaNavigation::moveDist(1.1, 2500);

                for(int i=0; i < categories.size(); i++) //----------------
                {
                    if(categories[i] == objectGraspedCat[0])
                    {
                        JustinaTasks::placeObjectOnShelfHC(0,level[i]);
                    }
                    if(categories[i] == objectGraspedCat[1])
                    {
                        JustinaTasks::placeObjectOnShelfHC(1,level[i]);
                    }
                }

				objectGrasped[0] = true;
				objectGrasped[1] = true; 
				objectGraspedCat[0] = "food";
				objectGraspedCat[1] = "fruits";
				//food
				//fruits
				//containers
				//cuterly
				//tableware
				//cleaning stuff	
		        JustinaNavigation::moveDist(-1.1, 2500);

				nextState = SM_WAIT_FOR_COMMAND;

            }    
            break;//----------------------//*/
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
