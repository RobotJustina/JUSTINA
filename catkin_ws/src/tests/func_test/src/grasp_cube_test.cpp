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

    //geometry_msgs::Pose pose;
    
    bool found;
    int indexFound = 0;
    vision_msgs::VisionObjectList objects;
    vision_msgs::VisionObject object;
    //vision_msgs::CubesSegmented cubes;
    //vision_msgs::Cube cube_aux;
    bool withLeftOrRightArm;
    object.id = "green_2";
    objects.ObjectList.push_back(object);

    while(ros::ok() && !fail && !success){
        switch(nextState){
            case 1:
                isAlign = JustinaTasks::alignWithTable(0.42);
                std::cout << "Align With table " << std::endl;
                if(!isAlign){
                    std::cout << "Can not align with table." << std::endl;
                    nextState = 1;
                }
                else
                    nextState = 2;
                break;
            case 2:
                found = JustinaVision::getObjectSeg(objects);
                withLeftOrRightArm = true; 
                if(!found){
                    std::cout << "Not found a object" << std::endl;
                    nextState = 2;
                }
                else{
                    object = objects.ObjectList[0];
                    if(object.pose.position.y > 0)
                        withLeftOrRightArm = true;
                    else
                        withLeftOrRightArm = false;
                    std::cout << "Found a object" << std::endl;
                    nextState = 3;
                }
                break;
            case 3:
                //JustinaTasks::moveActuatorToGrasp(pose.position.x, pose.position.y, pose.position.z, withLeftOrRightArm, idObject, true);
                //JustinaTasks::graspObjectFeedback(pose.position.x, pose.position.y, pose.position.z, withLeftOrRightArm, idObject, true);
                JustinaTasks::graspObjectColorFeedback(object, withLeftOrRightArm, object.id, true);
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
