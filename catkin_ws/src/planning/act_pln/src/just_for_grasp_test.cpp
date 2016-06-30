#include <iostream>
#include "justina_tools/JustinaTasks.h"

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING A TEST JUST FOR TEST BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "act_pln");
    ros::NodeHandle n;
    JustinaTasks::setNodeHandle(&n);
    ros::Rate loop(10);

    int nextState = 1;
    bool fail = false; 
    bool success = false;
    bool isAlign;
    geometry_msgs::Pose pose;

    std::vector<vision_msgs::VisionObject> recognizedObjects;
    bool found;
    int indexFound = 0;
    std::string idObject = "pringles";

    while(ros::ok() && !fail && !success){
        switch(nextState){
        case 1:
            isAlign = JustinaTasks::alignWithTable(0.35);
            std::cout << "Align With table " << std::endl;
            if(!isAlign){
                std::cout << "Can not align with table." << std::endl;
                nextState = 1;
            }
            else
                nextState = 2;
            break;
        case 2:
            if(!JustinaManip::hdGoTo(0, -0.9, 5000))
                JustinaManip::hdGoTo(0, -0.9, 5000);

            found = JustinaVision::detectObjects(recognizedObjects);
            indexFound = 0;
            if(found){
                found = false;
                for(int i = 0; i < recognizedObjects.size(); i++){
                    vision_msgs::VisionObject vObject = recognizedObjects[i];
                    if(vObject.id.compare(idObject) == 0){
                        found = true;
                        indexFound = i;
                        break;
                    }
                }
            }
            if(!found || recognizedObjects.size() == 0){
                std::cout << "Not found a object" << std::endl;
                nextState = 2;
            }
            else{
                std::cout << "Found a object" << std::endl;
                pose = recognizedObjects[indexFound].pose;
                nextState = 3;
            }
            break;
        case 3:
            JustinaTasks::graspObject(pose.position.x, pose.position.y, pose.position.z, false);
            nextState = -1;
            break;
        default:
            std::cout << "NavigTest.->Somebody very stupid programmed this shit. " << std::endl;
            fail = true;
            break;
        }
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}
