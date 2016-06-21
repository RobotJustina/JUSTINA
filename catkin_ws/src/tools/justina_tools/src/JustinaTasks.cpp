#include "justina_tools/JustinaTasks.h"

bool JustinaTasks::is_node_set = false;

bool JustinaTasks::setNodeHandle(ros::NodeHandle* nh)
{
    if(JustinaTasks::is_node_set)
        return true;
    if(nh == 0)
        return false;

    std::cout << "JustinaTasks.->Setting ros node..." << std::endl;
    JustinaHardware::setNodeHandle(nh);
    JustinaHRI::setNodeHandle(nh);
    JustinaManip::setNodeHandle(nh);
    JustinaNavigation::setNodeHandle(nh);
    JustinaVision::setNodeHandle(nh);
    JustinaTools::setNodeHandle(nh);

    JustinaTasks::is_node_set = true;
    return true;
}

bool JustinaTasks::alignWithTable()
{
    std::cout << "JustinaTasks.->Aligning with table. Moving head to 0 -0.9" << std::endl;
    if(!JustinaManip::hdGoTo(0, -0.9, 5000))
        JustinaManip::hdGoTo(0, -0.9, 5000);
    std::cout << "JustinaTasks.->Requesting line to line_finder" << std::endl;
    float x1, y1, z1, x2, y2, z2;
    if(!JustinaVision::findLine(x1, y1, z1, x2, y2, z2))
    {
        std::cout << "JustinaTasks.->Cannot find line. " << std::endl;
        return false;
    }
    if(fabs(z1 - z2) > 0.3)
    {
        std::cout << "JustinaTasks.->Found line is not confident. " << std::endl;
        return false;
    }
    float robotX, robotY, robotTheta;
    std::cout << "JustinaTasks.->Getting robot position.." << std::endl;
    JustinaNavigation::getRobotPose(robotX, robotY, robotTheta);
    float A = y1 - y2;
    float B = x2 - x1;
    float C = -(A*x1 + B*y1);
    //The robot center should be 0.4 m away of the table
    float distance = fabs(A*robotX + B*robotY + C)/sqrt(A*A + B*B) - 0.3;
    float angle = atan2(y2 - y1, x2 - x1) - M_PI/2;
    if(angle < 0)
        angle += M_PI;
    std::cout << "JustinaTasks.->Moving base: dist=" << distance << "  angle=" << angle << std::endl;
    JustinaNavigation::moveDistAngle(distance, angle, 10000);
    return true;
}

bool JustinaTasks::graspNearestObjectLeftArm()
{
    std::cout << "JustinaTasks.->Moving to a good-pose for grasping objects" << std::endl;
    if(!JustinaManip::hdGoTo(0, -0.9, 5000))
        JustinaManip::hdGoTo(0, -0.9, 5000);
    std::cout << "JustinaTasks.->Trying to detect objects..." << std::endl;
    std::vector<vision_msgs::VisionObject> recoObjList;
    if(!JustinaVision::detectObjects(recoObjList))
    {
        std::cout << "JustinaTasks.->Cannot dectect objects :'( " << std::endl;
        return false;
    }

    float idealX = 0.4;
    float idealY = 0.235; //It is the distance from the center of the robot, to the center of the arm
    float idealZ = 0.618; //It is the ideal height for taking an object when torso is at zero height.
    float torsoSpine, torsoWaist, torsoShoulders;
    JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist, torsoShoulders);
    idealZ += torsoSpine;

    float minDist = 1000000;
    int nearestObj = -1;
    for(size_t i=0; i< recoObjList.size(); i++)
    {
        float objX = recoObjList[i].pose.position.x;
        float objY = recoObjList[i].pose.position.y;
        float objZ = recoObjList[i].pose.position.z;
        float temp =  sqrt((objX-idealX)*(objX-idealX) + (objY-idealY)*(objY-idealY) + (objZ-idealZ)*(objZ-idealZ));
        if(temp < minDist)
        {
            minDist = temp;
            nearestObj = i;
        }
    }

    float objToGraspX = recoObjList[nearestObj].pose.position.x;
    float objToGraspY = recoObjList[nearestObj].pose.position.y;
    float objToGraspZ = recoObjList[nearestObj].pose.position.z;
    float movFrontal = idealX -  objToGraspX;
    float movLateral = idealY -  objToGraspY;
    float movVertical = idealZ - objToGraspZ;
    float goalTorso = torsoSpine + movVertical;
    if(goalTorso < 0)
        goalTorso = 0;
    if(goalTorso > 0.45)
        goalTorso = 0.45;

    std::cout << "JustinaTasks.->Adjusting  with frontal=" << movFrontal << "  lateral=" << movLateral << "  and vertical=" << movVertical << std::endl;
    float lastRobotX, lastRobotY, lastRobotTheta;
    JustinaNavigation::getRobotPose(lastRobotX, lastRobotY, lastRobotTheta);
    JustinaManip::startTorsoGoTo(goalTorso, 0, 0);
    JustinaNavigation::moveLateral(movLateral, 5000);
    JustinaNavigation::moveDist(movFrontal, 5000);
    JustinaManip::waitForTorsoGoalReached(30000);
    float robotX, robotY, robotTheta;
    JustinaNavigation::getRobotPose(robotX, robotY, robotTheta);
    //Adjust the object position according to the new robot pose
    //I don't request again the object position due to the possibility of not recognizing it again
    objToGraspX -= (robotX - lastRobotX);
    objToGraspY -= (robotY - lastRobotY);
    //The position it is adjusted and converted to coords wrt to left arm
    if(!JustinaTools::transformPoint("base_link", objToGraspX, objToGraspY, objToGraspZ, "left_arm_link1", objToGraspX, objToGraspY, objToGraspZ))
    {
        std::cout << "JustinaTasks.->Cannot transform point. " << std::endl;
        return false;
    }
    std::cout << "JustinaTasks.->Moving left arm to " << objToGraspX << "  " << objToGraspY << "  " << objToGraspZ << std::endl;
}
