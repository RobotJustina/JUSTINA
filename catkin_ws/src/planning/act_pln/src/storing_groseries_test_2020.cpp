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

#define MAX_FIND_PERSON_COUNT 1
#define MAX_FIND_PERSON_RESTART 2
#define MAX_FIND_PERSON_ATTEMPTS 2

#define SM_SAY_WAIT_FOR_DOOR 0
#define SM_WAIT_FOR_DOOR 1
#define SM_INIT 3
#define SM_WAIT_FOR_START_COMMAND 10
#define SM_NAVIGATION_TO_TABLE 20
#define SM_FIND_TABLE 25
#define SM_FIND_HUMAN   27
#define SM_FIND_OBJECTS_ON_TABLE 30
#define SM_INF_TAKE_OBJECT 41
#define SM_TAKE_OBJECT  50
#define SM_FIND_AND_TAKE_OBJECTS  51
#define SM_TAKE_OBJECT_RIGHT 55
#define SM_TAKE_OBJECT_LEFT 60
#define SM_GOTO_CUPBOARD         69
#define SM_TAKE_OBJECT_SIMUL     70
#define SM_WAIT_FOR_COMMAND      71
#define SM_PARSE_SPOKEN_COMMAND  72
#define SM_WAIT_FOR_CONFIRMATION 73
#define SM_OPEN_DOOR 75
#define SM_FIND_OBJECTS_ON_CUPBOARD 80
#define SM_PUT_OBJECT_ON_CUPBOARD 100
#define SM_FINISH_TEST 110
#define SM_PLACE_ARMS 120

std::string stateMachine = "storing_groceries.->";

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
    // ------ This is for not attempt open the cupboard
    bool openDoor = false;
    //////******************************//////
    bool findPerson = false;
    bool skipFindPerson = false;

    bool simul =             false;
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
    
    // This is new for storing groceries test
    std::vector<std::string> idsPerson;
    idsPerson.push_back("chair");
    idsPerson.push_back("person");
    
    std::vector<Eigen::Vector3d> centroids;
    
    int findPersonCount = 0;
    int findPersonAttemps = 0;
    int findPersonRestart = 0;

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
    //std::string name_test = "storingGroseries_1";
    std::string stateMachine = "stroing_groceries.->";
    /////*******************************//////
    std::string simulation = argv[1];

    if(simulation == "simul")
        simul = true;

    std::string lastRecoSpeech;
    std::vector<std::string> validCommands;
    validCommands.push_back("first");
    validCommands.push_back("second");
    validCommands.push_back("third");

    int arm = 0;
    int level_in_[2];
    bool ask;

    std::vector<std::string> categories;
    std::vector<int>              level;
    std::string ss_level;
    
    std::cout<< "SIMULATION:"<<argv[1]<<":"<<simul<<std::endl;

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

    nv_cpb        =  "Navigate to kitchen cabinet.";
    cnt_od        =  "I am search a kitchen cabinets door.";
    ask_hlp       =  "---Ask for help to open the kitchen cabinet´s door.";
    srch_obj_cpb  =  "I am goint to search objects into the kitchen cabinet.";
    ctg_objs_fnd  =  "The categories the objects found are: ";
    fnd_tbl       =  "I am trying to find a nearest table.";
    fnd_objs_tbl  =  "I am going to find objects on the table.";



    while(ros::ok() && !fail && !success){
        switch(nextState){

            case SM_SAY_WAIT_FOR_DOOR:
                {
                    std::cout << stateMachine << "\033[1;33mSM_SAY_WAIT_FOR_DOOR\033[0m\n" << std::endl;
                    JustinaManip::startHdGoTo(0.0, 0.0);
                    JustinaHRI::say("I'm ready for storing groseries test");
                    JustinaHRI::waitAfterSay("I am waiting for the door to be open", 4000);
                    std::cout << "\033[1;34mI am waiting for the door to be open\033[0m\n" << std::endl;
                    nextState = SM_WAIT_FOR_DOOR;
                }
                break;

            case SM_WAIT_FOR_DOOR:
                {
                    std::cout << stateMachine << "\033[1;33mSM_SAY_WAIT_FOR_DOOR\033[0m\n" << std::endl;
                    if (!JustinaNavigation::obstacleInFront())
                        nextState = SM_INIT;
                }
                break;
            
            case SM_INIT:
                {
                    std::cout << stateMachine << "\033[1;33mSM_INIT\033[0m\n" << std::endl;
                    JustinaHRI::waitAfterSay("Now I can see that the door is open",4000);
                    std::cout << "\033[1;34mNow I can see that the door is open\033[0m\n" << std::endl;
                    JustinaNavigation::moveDist(1.0, 4000);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(500));

                    nextState = SM_GOTO_CUPBOARD;

                    attempsNavigation = 0;
                    findPersonAttemps = 0;
                    findObjCupboard = false;
                }
                break;

            case SM_GOTO_CUPBOARD:
                {
                    std::cout << stateMachine << "\033[1;33mSM_GOTO_CUPBOARD\033[0m\n" << std::endl;
                    JustinaManip::startTorsoGoTo(0.1, 0, 0);
                    JustinaHRI::say("I will navigate to the kitchen cabinet");
                    JustinaHRI::say("human, help me please, remove all chairs for me, in the kitchen table");
                    if(!JustinaNavigation::getClose("kitchen_cabinet",200000))
                        if(!JustinaNavigation::getClose("kitchen_cabinet",200000))
                            JustinaNavigation::getClose("kitchen_cabinet",200000);
                    //JustinaHRI::insertAsyncSpeech("I Have reached the kitchen cabinet", 3000);
                    //JustinaHRI::asyncSpeech();
                    if(!findObjCupboard)
                        nextState = SM_OPEN_DOOR;
                    else
                        nextState = SM_WAIT_FOR_COMMAND;
                }
                break;

            case SM_OPEN_DOOR:
                {
                    std::cout << stateMachine << "\033[1;33mSM_OPEN_DOOR\033[0m\n" << std::endl;
                    if(!openDoor){
                        JustinaHRI::waitAfterSay("Human can you open the kitchen cabinet door please", 6000);
                        std::cout << "\033[1;34mHuman can you open the kitchen cabinet door please\033[0m\n" << std::endl;
                        findObjCupboard = true;
                        nextState = SM_NAVIGATION_TO_TABLE;
                    }
                    else{
                        JustinaHRI::say("I'm trying to open the kitchen cabinet door");
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
                    std::cout << stateMachine << "\033[1;33mSM_FIND_OBJECTS_ON_CUPBOARD\033[0m\n" << std::endl;

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
                        std::cout << "I have found " << recoObjList.size() << " objects on the kitchen cabinet" << std::endl;
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
                    justinaSay << "I have found " << countObject << " objects into kitchen cabinet";
                    // JustinaHRI::insertAsyncSpeech(justinaSay.str(), 500);
                    JustinaHRI::say(justinaSay.str());

                    justinaSay.str("");
                    justinaSay << "The objects of the kitchen cabinet belong to categories";
                    // JustinaHRI::insertAsyncSpeech(justinaSay.str(), 500);
                    JustinaHRI::say(justinaSay.str());
                    
                    justinaSay.str("");
                    for(int i = 0; i < categories_cpbr.size(); i++)
                       justinaSay << ", " << categories_cpbr[i];
                    // JustinaHRI::insertAsyncSpeech(justinaSay.str(), 500);
                    JustinaHRI::say(justinaSay.str());
                    nextState = SM_NAVIGATION_TO_TABLE;

                    nmbr_objs_fnd_cpb << "I have found " << itemsOnCupboard << " objects into kitchen cabinet.";


                    findObjCupboard = true;
                    attempsPlaceObj = 1;

                    nextState = SM_PUT_OBJECT_ON_CUPBOARD;
                }
                break;

            case SM_NAVIGATION_TO_TABLE:
                {
                    std::cout << stateMachine << "\033[1;33mSM_NAVIGATION_TO_TABLE\033[0m\n" << std::endl;
                    JustinaManip::startTorsoGoTo(0.1, 0.0, 0.0);
                    
                    JustinaNavigation::startGetClose("table_location");
                    
                    curr = boost::posix_time::second_clock::local_time();
                    prev = curr;

                    nextState = SM_FIND_OBJECTS_ON_TABLE;
                    //attempsNavigation++;
                }
                break;

            case SM_FIND_OBJECTS_ON_TABLE:
                {
                    std::cout << stateMachine << "\033[1;33mSM_FIND_OBJECTS_ON_TABLE\033[0m\n" << std::endl;
                    JustinaManip::startTorsoGoTo(0.0, 0.0, 0.0);
                    if(attempsFindObjectsTable == 0 && alignWithTable){
                        JustinaHRI::say("I am going to look for objects on the table");
                        JustinaTasks::alignWithTable(0.35);
                    }

                    JustinaManip::startHdGoTo(0, -0.79);
                    recoObjForTake = std::vector<vision_msgs::VisionObject>();
                    categories_tabl = std::vector<std::string>();

                    //IF JUSTINA DID NOT DETECT SOMETHING
                    if(!JustinaVision::detectAllObjectsVot(recoObjForTake, image, 5)){
                        if(objectGrasped[0] || objectGrasped[1])
                            nextState = SM_GOTO_CUPBOARD;
                        else{
                            std::cout << "I  cannot detect anything" << std::endl;
                            attempsFindObjectsTable++;
                            if(attempsFindObjectsTable > 3)
                                nextState = SM_FINISH_TEST;
                            else 
                                SM_FIND_OBJECTS_ON_TABLE;
                        }
                    }
                    //IF JUSTINA FOUND SOMETHING
                    else{
                        if(recoObjForTake.size() == 0){
                            if(objectGrasped[0] || objectGrasped[1])
                                nextState = SM_GOTO_CUPBOARD;
                            else{
                                std::cout << "I  cannot reach anything" << std::endl;
                                attempsFindObjectsTable++;
                                nextState = SM_FIND_OBJECTS_ON_TABLE;
                            }
                        }
                        justinaSay.str("");
                        if(recoObjForTake.size() <= 10){
                            int countObject = recoObjForTake.size();
                            justinaSay << "I have found " << countObject << " objects on the table";
                            attempsFindObjectsTable = 0;
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

                            float confidence = 1.0;
                            
                            std::size_t found = recoObjForTake[i].id.find("unknown");
                            countKnownObjects = 0;
                            if(found == std::string::npos){
                                JustinaRepresentation::insertConfidenceAndGetCategory(recoObjForTake[i].id, i, confidence, category, 0);
                                recoObjForTake[i].category = category;
                                countKnownObjects++;
                            }
                        }
                    
                        JustinaTools::getCategoriesFromVisionObject(recoObjForTake, categories_tabl);
                        for(int i = 0; i < categories_tabl.size(); i++)
                            std::cout << "Category_" << i << ":  " << categories_tabl[i] << std::endl;


                        justinaSay.str( std::string() );
                        justinaSay << "The objects of the table belong to categories...";


                        nextState = SM_INF_TAKE_OBJECT;

                    }
                }
                break;

            case SM_INF_TAKE_OBJECT:
                {
                    std::cout << stateMachine << "\033[1;33mSM_INF_TAKE_OBJECT\033[0m\n" << std::endl;
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

                        // Here is to get which of the two objects is optimal to grasp with left or right arm
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
                                if(!simul)
                                    nextState = SM_TAKE_OBJECT;
                                else
                                    nextState = SM_TAKE_OBJECT_SIMUL;
                                
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
                        if(!simul)
                            nextState = SM_TAKE_OBJECT;
                        else
                            nextState = SM_TAKE_OBJECT_SIMUL;
                    }
                }
                break;

            case SM_FIND_AND_TAKE_OBJECTS:
                {
                    std::cout << stateMachine << "\033[1;33mSM_FIND_AND_TAKE_OBJECTS\033[0m\n" << std::endl;
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
                            
                            std::cout << justinaSay.str() <<std::endl;
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
                    std::cout << stateMachine << "\033[1;33mSM_TAKE_OBJECT\033[0m\n" << std::endl;
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
                    std::cout << stateMachine << "\033[1;33mSM_PUT_OBJECT_ON_CUPBOARD\033[0m\n" << std::endl;
                    for(int i=0; i< categories.size(); i++) //----------------
                        std::cout<< categories[i]<<" in the level : "<<level[i]<<std::endl;
                    
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
                            // -------------------------------------------- This change was do by Rey, to algin each to put the object
                            /*if(!JustinaTasks::alignWithTable(0.35)){
                                JustinaNavigation::moveDist(-0.10, 3000);
                                if(!JustinaTasks::alignWithTable(0.35)){
                                    JustinaNavigation::moveDist(0.15, 3000);
                                    JustinaTasks::alignWithTable(0.35);
                                }
                            }*/
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
                                // -------------------------------------------- This change was do by Rey, to algin each to put the object
                                // This add for align with table again
                                if(!JustinaTasks::alignWithTable(0.35)){
                                    JustinaNavigation::moveDist(0.10, 3000);
                                    if(!JustinaTasks::alignWithTable(0.35)){
                                        JustinaNavigation::moveDist(0.10, 3000);
                                        JustinaTasks::alignWithTable(0.35);
                                    }
                                }
                                if(categories[i] == objectGraspedCat[0])
                                {
                                    JustinaTasks::placeObjectOnShelfHC(0,level[i]);
                                    objectGrasped[0] = false;
                                }
                                if(categories[i] == objectGraspedCat[1])
                                {
                                    JustinaTasks::placeObjectOnShelfHC(1,level[i]);
                                    objectGrasped[1] = false;
                                }
                                nextState = SM_NAVIGATION_TO_TABLE;
                            }//----------------------//*/
                            attempsPlaceObj++;
                        }
                        else{
                            attempsPlaceObj = 1;
                            std::cout << "I can´t placed objects on kitchen cabinet whit right Arm" << std::endl;
                            JustinaHRI::say("I can´t found a free place in the kitchen cabinet");
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
            
            case SM_WAIT_FOR_COMMAND: 
                {
                    std::cout << stateMachine << "\033[1;33mSM_WAIT_FOR_COMMAND\033[0m\n"<< std::endl; 				

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
                            justinaSay << "Could you tell me at what level to store the " << objectGraspedCat[arm]<< ", first, second or third";
	                        std::cout <<  "\nCould you tell me at what level to store the " << objectGraspedCat[arm]<< ", first, second or third"<<std::endl;
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
	                    		nextState = SM_PUT_OBJECT_ON_CUPBOARD;
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
                    		nextState = SM_PUT_OBJECT_ON_CUPBOARD;
                    	}
                    }
                }
                break;

            case SM_PARSE_SPOKEN_COMMAND:
                {      
                    std::cout << stateMachine << "\033[1;33mSM_PARSE_SPOKEN_COMMAND\033[0m\n" << std::endl;
                    
                    justinaSay.str("");       
                    if(lastRecoSpeech.find("first") != std::string::npos){
                        level_in_[arm] = 1;
                        ss_level = "first";
                    }
                    else if(lastRecoSpeech.find("second") != std::string::npos){
                        level_in_[arm] = 2;
                        ss_level = "second";  
                    }
                    else if(lastRecoSpeech.find("third") != std::string::npos){
                        level_in_[arm] = 3;
                        ss_level = "third";
                    }
                    else
                    {
                        JustinaHRI::waitAfterSay(" Sorry, I did not understand you ", 6000);  
                        std::cout << "Sorry, I did not understand you"<< std::endl;                 
                        nextState = SM_WAIT_FOR_COMMAND;
                    }

                    justinaSay << "Ok, I am going to store the " <<objectGraspedCat[arm]<< " on the "<< ss_level<< " level ";               
                    std::cout  << "Ok, I am going to store the " <<objectGraspedCat[arm]<< " on the "<< ss_level<< " level " << std::endl;
                    JustinaHRI::waitAfterSay(justinaSay.str(), 7000);

                    categories.push_back(objectGraspedCat[arm]);
                    level.push_back(level_in_[arm]);
                    
                    arm++;
                    if(arm <= 1)
                        nextState = SM_WAIT_FOR_COMMAND;
                    
                    else
                    {
                        arm = 0;
                        nextState = SM_PUT_OBJECT_ON_CUPBOARD;
                    }
            }                
            break;

            case SM_TAKE_OBJECT_SIMUL:
            {
                std::cout << stateMachine << "\033[1;33mSM_TAKE_OBJECT_SIMUL\033[0m\n" << std::endl;

                bool withLeftOrRightArm;
                justinaSay.str("");
                if(takeRight){
                    withLeftOrRightArm = false;
                    indexObjectGrasp = indexObjectGraspRight;
                    if(attempsGraspObject == 0)
                        justinaSay << "I am going to take the " << recoObjForTake[indexObjectGrasp].id << " with my right arm";
                }

                else if(takeLeft){
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
                }
                //food
                //fruits
                //containers
                //cuterly
                //tableware
                //cleaning stuff
                else{
                    if(!withLeftOrRightArm){
                        objectGrasped[0] = true;
                        objectGraspedObj[0] = idObjectGrasp;
                        objectGraspedCat[0] = "food";//catObjectGrasp;
                        takeRight = false;
                        nextState = SM_FIND_OBJECTS_ON_TABLE;
                    }
                    else{
                        attempsGraspLeft++;
                        if(attempsGraspLeft >= maxAttempsGraspLeft){
                            if(objectGrasped[0]){
                                takeLeft = true;
                                nextState = SM_GOTO_CUPBOARD;
                            }
                            else
                                nextState = SM_FIND_OBJECTS_ON_TABLE;
                        }
                        else{
                            objectGrasped[1] = true;
                            objectGraspedObj[1] = idObjectGrasp;
                            objectGraspedCat[1] = "fruit";//catObjectGrasp;
                            takeLeft = true;
                            nextState = SM_GOTO_CUPBOARD;
                        }
                    }
                    alignWithTable = true;
                    attempsFindObjectsTable = 0;
                }


            }
            break;

            case SM_FINISH_TEST:
            {
                std::cout << stateMachine << "\033[1;33mSM_FINISH_TEST.-> I have finish the test\033[0m\n" << std::endl;
                JustinaHRI::say("I have finished the test");
                fail = false;
                success = true;
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
