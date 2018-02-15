#include "justina_tools/JustinaTasks.h"

bool JustinaTasks::is_node_set = false;

bool JustinaTasks::setNodeHandle(ros::NodeHandle* nh) {
    if (JustinaTasks::is_node_set)
        return true;
    if (nh == 0)
        return false;

    std::cout << "JustinaTasks.->Setting ros node..." << std::endl;
    JustinaHardware::setNodeHandle(nh);
    JustinaHRI::setNodeHandle(nh);
    JustinaManip::setNodeHandle(nh);
    JustinaNavigation::setNodeHandle(nh);
    JustinaVision::setNodeHandle(nh);
    JustinaTools::setNodeHandle(nh);
    JustinaKnowledge::setNodeHandle(nh);

    JustinaTasks::is_node_set = true;
    return true;
}

bool JustinaTasks::alignWithTable() {
    return JustinaTasks::alignWithTable(0.4);
}

bool JustinaTasks::alignWithTable(float distToTable) {
    std::cout << "JustinaTasks.->Aligning with table. Moving head to 0 -0.9"
        << std::endl;
    if (!JustinaManip::hdGoTo(0, -0.9, 5000))
        JustinaManip::hdGoTo(0, -0.9, 5000);
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    std::cout << "JustinaTasks.->Requesting line to line_finder" << std::endl;
    float x1, y1, z1, x2, y2, z2;
    if (!JustinaVision::findLine(x1, y1, z1, x2, y2, z2)) {
        std::cout << "JustinaTasks.->Cannot find line. " << std::endl;
        return false;
    }
    if (fabs(z1 - z2) > 0.3) {
        std::cout << "JustinaTasks.->Found line is not confident. "
            << std::endl;
        return false;
    }

    if(x1 == x2 && y1 == y2 && z1 == z2)
        return false;

    float robotX = 0, robotY = 0, robotTheta = 0;
    //std::cout << "JustinaTasks.->Getting robot position.." << std::endl;
    //JustinaNavigation::getRobotPose(robotX, robotY, robotTheta);
    //Since line is give wrt robot, we can consider that robot is at zero
    float A = y1 - y2;
    float B = x2 - x1;
    float C = -(A * x1 + B * y1);
    //The robot center should be 0.4 m away of the table
    float distance = fabs(A * robotX + B * robotY + C) / sqrt(A * A + B * B)
        - distToTable;
    float angle = atan2(y2 - y1, x2 - x1) - M_PI / 2;
    if (angle < 0)
        angle += M_PI;
    std::cout << "JustinaTasks.->Moving base: dist=" << distance << "  angle="
        << angle << std::endl;
    JustinaNavigation::moveDistAngle(distance, angle, 10000);
    return true;
}

bool JustinaTasks::graspNearestObject(bool withLeftArm) {
    std::cout
        << "JustinaTasks.->Trying to detect objects for manipulating with ";
    if (withLeftArm)
        std::cout << "left arm." << std::endl;
    else
        std::cout << "right arm." << std::endl;
    if (!JustinaManip::hdGoTo(0, -0.9, 5000))
        JustinaManip::hdGoTo(0, -0.9, 5000);
    ros::Rate loop(10);
    int delays = 10;
    //w
    std::cout << "JustinaTasks.->Trying to detect objects..." << std::endl;
    std::vector<vision_msgs::VisionObject> recoObjList;
    if (!JustinaVision::detectObjects(recoObjList)) {
        std::cout << "JustinaTasks.->Cannot dectect objects :'(" << std::endl;
        return false;
    }
    return JustinaTasks::graspNearestObject(recoObjList, withLeftArm);
}

bool JustinaTasks::graspNearestObject(
        std::vector<vision_msgs::VisionObject>& recoObjList, bool withLeftArm) {
    std::cout
        << "JustinaTasks.->Moving to a good-pose for grasping objects with ";
    if (withLeftArm)
        std::cout << "left arm" << std::endl;
    else
        std::cout << "right arm" << std::endl;

    float idealX = 0.475;
    float idealY = withLeftArm ? 0.235 : -0.235; //It is the distance from the center of the robot, to the center of the arm
    float idealZ = 0.618; //It is the ideal height for taking an object when torso is at zero height.
    float torsoSpine, torsoWaist, torsoShoulders;
    JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist,
            torsoShoulders);
    idealZ += torsoSpine;

    float minDist = 1000000;
    int nearestObj = -1;
    for (size_t i = 0; i < recoObjList.size(); i++) {
        float objX = recoObjList[i].pose.position.x;
        float objY = recoObjList[i].pose.position.y;
        float objZ = recoObjList[i].pose.position.z;
        float temp = sqrt(
                (objX - idealX) * (objX - idealX)
                + (objY - idealY) * (objY - idealY)
                + (objZ - idealZ) * (objZ - idealZ));
        if (temp < minDist) {
            minDist = temp;
            nearestObj = i;
        }
    }

    std::string id = recoObjList[nearestObj].id;
    float objToGraspX = recoObjList[nearestObj].pose.position.x;
    float objToGraspY = recoObjList[nearestObj].pose.position.y;
    float objToGraspZ = recoObjList[nearestObj].pose.position.z;
    std::cout << "JustinaTasks.->ObjToGrasp: " << id << "  " << objToGraspX
        << "  " << objToGraspY << "  " << objToGraspZ << std::endl;
    float movFrontal = -(idealX - objToGraspX);
    float movLateral = -(idealY - objToGraspY);
    float movVertical = -(idealZ - objToGraspZ);
    float goalTorso = torsoSpine + movVertical;
    if (goalTorso < 0.2)
        goalTorso = 0.2;
    if (goalTorso > 0.5)
        goalTorso = 0.5;

    std::cout << "JustinaTasks.->Adjusting with frontal=" << movFrontal
        << " lateral=" << movLateral << " and vertical=" << movVertical
        << std::endl;
    float lastRobotX, lastRobotY, lastRobotTheta;
    JustinaNavigation::getRobotPose(lastRobotX, lastRobotY, lastRobotTheta);
    JustinaManip::startTorsoGoTo(goalTorso, 0, 0);
    JustinaNavigation::moveLateral(movLateral, 3000);
    JustinaNavigation::moveDist(movFrontal, 3000);
    int waitTime = (int) (30000 * movFrontal + 2000);
    JustinaManip::waitForTorsoGoalReached(waitTime);
    float robotX, robotY, robotTheta;
    JustinaNavigation::getRobotPose(robotX, robotY, robotTheta);
    //Adjust the object position according to the new robot pose
    //I don't request again the object position due to the possibility of not recognizing it again
    objToGraspX -= (robotX - lastRobotX);
    objToGraspY -= (robotY - lastRobotY);
    //The position it is adjusted and converted to coords wrt to the corresponding arm
    std::string destFrame = withLeftArm ? "left_arm_link1" : "right_arm_link1";
    if (!JustinaTools::transformPoint("base_link", objToGraspX, objToGraspY,
                objToGraspZ, destFrame, objToGraspX, objToGraspY, objToGraspZ)) {
        std::cout << "JustinaTasks.->Cannot transform point. " << std::endl;
        return false;
    }
    std::cout << "JustinaTasks.->Moving ";

    if (withLeftArm)
        std::cout << "left arm";
    else
        std::cout << "right arm";

    std::cout << " to " << objToGraspX << "  " << objToGraspY << "  "
        << objToGraspZ << std::endl;

    if (withLeftArm) {
        JustinaManip::startLaOpenGripper(0.6);
        JustinaManip::laGoTo("navigation", 5000);
        JustinaManip::laGoToCartesian(objToGraspX - 0.03, objToGraspY - 0.04,
                objToGraspZ, 0, 0, 1.5708, 0, 5000);
        JustinaManip::startLaCloseGripper(0.4);
        ros::Rate loop(10);
        int attempts = 20;
        while (ros::ok() && --attempts > 0)
            loop.sleep();
        JustinaManip::startTorsoGoTo(goalTorso + 0.03, 0, 0);
        JustinaManip::waitForTorsoGoalReached(3000);
        JustinaNavigation::moveDist(-0.15, 3000);
        JustinaManip::laGoTo("navigation", 5000);
    } else {
        JustinaManip::startRaOpenGripper(0.6);
        JustinaManip::raGoTo("navigation", 5000);
        JustinaManip::raGoToCartesian(objToGraspX - 0.03, objToGraspY - 0.04,
                objToGraspZ, 0, 0, 1.5708, 0, 5000);
        JustinaManip::startRaCloseGripper(0.4);
        ros::Rate loop(10);
        int attempts = 20;
        while (ros::ok() && --attempts > 0)
            loop.sleep();
        JustinaManip::startTorsoGoTo(goalTorso + 0.03, 0, 0);
        JustinaManip::waitForTorsoGoalReached(3000);
        JustinaNavigation::moveDist(-0.15, 3000);
        JustinaManip::raGoTo("navigation", 5000);
    }
    return true;
}

bool JustinaTasks::graspObject(float x, float y, float z, bool withLeftArm,
        std::string idObject, bool usingTorse) {
    std::cout
        << "JustinaTasks.->Moving to a good-pose for grasping objects with ";
    if (withLeftArm)
        std::cout << "left arm" << std::endl;
    else
        std::cout << "right arm" << std::endl;

    bool objectInHand = false;
    float idealX = 0.475;
    float idealY = withLeftArm ? 0.234 : -0.235; //It is the distance from the center of the robot, to the center of the arm
    float idealZ = 0.52; //It is the ideal height for taking an object when torso is at zero height.

    float torsoSpine, torsoWaist, torsoShoulders;
    JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist,
            torsoShoulders);
    std::cout << "JustinaTasks.->torsoSpine:" << torsoSpine << std::endl;

    tf::StampedTransform transform;
    tf::TransformListener* tf_listener = new tf::TransformListener();
    tf_listener->waitForTransform("base_link", "map", ros::Time(0), ros::Duration(10.0));
    tf_listener->lookupTransform("base_link", "map",ros::Time(0), transform);
    tf::Vector3 p(x , y , z);

    float objToGraspX = x;
    float objToGraspY = y;
    float objToGraspZ = z;
    float movTorsoFromCurrPos;

    if(idObject.compare("simul") == 0){
        p = transform * p;
        objToGraspX = p.getX();
        objToGraspY = p.getY();
        objToGraspZ = p.getZ();
    }

    std::cout << "JustinaTasks.->ObjToGrasp: " << "  " << objToGraspX << "  "
        << objToGraspY << "  " << objToGraspZ << std::endl;
    float movFrontal = -(idealX - objToGraspX);
    float movLateral = -(idealY - objToGraspY);
    float movVertical = objToGraspZ - idealZ - torsoSpine;
    float goalTorso = torsoSpine + movVertical;
    std::cout << "JustinaTasks.->goalTorso:" << goalTorso << std::endl;
    int waitTime;
    if (goalTorso < 0.2)
        goalTorso = 0.2;
    if (goalTorso > 0.5)
        goalTorso = 0.5;

    movTorsoFromCurrPos = goalTorso - torsoSpine;
    waitTime = (int) (30000 * fabs(movTorsoFromCurrPos) / 0.3 + 3000);
    std::cout << "JustinaTasks.->movTorsoFromCurrPos:" << movTorsoFromCurrPos << std::endl;
    std::cout << "JustinaTasks.->waitTime:" << waitTime << std::endl;

    std::cout << "JustinaTasks.->Adjusting with frontal=" << movFrontal
        << " lateral=" << movLateral << " and vertical=" << movVertical
        << std::endl;
    float lastRobotX, lastRobotY, lastRobotTheta;
    JustinaNavigation::getRobotPose(lastRobotX, lastRobotY, lastRobotTheta);
    if(usingTorse)
        JustinaManip::startTorsoGoTo(goalTorso, 0, 0);
    JustinaNavigation::moveLateral(movLateral, 6000);
    JustinaNavigation::moveDist(movFrontal, 6000);
    if(usingTorse)
        JustinaManip::waitForTorsoGoalReached(waitTime);

    bool found = false;
    std::vector<vision_msgs::VisionObject> recognizedObjects;
    int indexFound = 0;
    if (idObject.compare("") != 0 && idObject.compare("simul") != 0) {
        JustinaManip::startHdGoTo(0, -0.9);
        JustinaManip::waitForHdGoalReached(5000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        found = JustinaVision::detectObjects(recognizedObjects);
        if (found) {
            found = false;
            for (int i = 0; i < recognizedObjects.size(); i++) {
                vision_msgs::VisionObject vObject = recognizedObjects[i];
                if (vObject.id.compare(idObject) == 0) {
                    found = true;
                    indexFound = i;
                    break;
                }
            }
        }
    }

    if (found) {
        std::cout << "The object was found again, update the new coordinates."
            << std::endl;
        objToGraspX = recognizedObjects[indexFound].pose.position.x;
        objToGraspY = recognizedObjects[indexFound].pose.position.y;
    } else if (!found && idObject.compare("") == 0) {
        std::cout
            << "The object was not found again, update new coordinates with the motion of robot."
            << std::endl;
        float robotX, robotY, robotTheta;
        //JustinaNavigation::getRobotPose(robotX, robotY, robotTheta);
        JustinaNavigation::getRobotPose(robotX, robotY, robotTheta);
        //Adjust the object position according to the new robot pose
        //I don't request again the object position due to the possibility of not recognizing it again
        float dxa = (robotX - lastRobotX);
        float dya = (robotY - lastRobotY);
        float dxr = dxa * cos(robotTheta) + dya * sin(robotTheta);
        float dyr = -dxa * sin(robotTheta) + dya * cos(robotTheta);

        objToGraspX -= dxr;
        objToGraspY -= dyr;
        std::cout << "lastRobotX:" << lastRobotX << ",lastRobotY:" << lastRobotY
            << ",lastRobotTheta:" << lastRobotTheta << std::endl;
        std::cout << "robotX:" << robotX << ",robotY:" << robotY
            << ",robotTheta:" << robotTheta << std::endl;
        std::cout << "objToGraspX:" << objToGraspX << ",objToGraspY:"
            << objToGraspY << ",objToGraspZ:" << objToGraspZ << std::endl;
    } else if(!found && idObject.compare("simul") == 0){
        tf_listener->waitForTransform("base_link", "map", ros::Time(0), ros::Duration(10.0));
        tf_listener->lookupTransform("base_link", "map", ros::Time(0), transform);
        tf::Vector3 pos(x, y ,z);
        pos = transform *pos;
        objToGraspX = pos.getX();
        objToGraspY = pos.getY();

    }
    else if (!found && idObject.compare("") != 0 && idObject.compare("simul") != 0) {
        JustinaNavigation::moveDist(-0.2, 3000);
        return false;
    }

    //The position it is adjusted and converted to coords wrt to the corresponding arm
    std::string destFrame = withLeftArm ? "left_arm_link0" : "right_arm_link0";
    if (!JustinaTools::transformPoint("base_link", objToGraspX, objToGraspY,
                objToGraspZ, destFrame, objToGraspX, objToGraspY, objToGraspZ)) {
        std::cout << "JustinaTasks.->Cannot transform point. " << std::endl;
        return false;
    }
    std::cout << "JustinaTasks.->Moving ";
    if (withLeftArm)
        std::cout << "left arm";
    else
        std::cout << "right arm";
    std::cout << " to " << objToGraspX << "  " << objToGraspY << "  "
        << objToGraspZ << std::endl;

    if (withLeftArm) {

        if(!JustinaManip::isLaInPredefPos("navigation"))
            JustinaManip::laGoTo("navigation", 7000);
        else
            std::cout << "JustinaTasks.->The left arm already has in the navigation pose" << std::endl;

        JustinaManip::startLaOpenGripper(0.8);
        //Move the manipulator to objectOB

        JustinaManip::laGoToCartesian(objToGraspX - 0.04, objToGraspY - 0.25,
                objToGraspZ - 0.04, 3000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));

        JustinaManip::laGoToCartesian(objToGraspX - 0.04, objToGraspY - 0.15,
                objToGraspZ - 0.04, 3000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));

        JustinaManip::laGoToCartesian(objToGraspX + 0.035, objToGraspY - 0.10,
                objToGraspZ - 0.06, 2000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));

        JustinaNavigation::moveDist(0.08, 3000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1500));


        JustinaManip::startLaCloseGripper(0.5);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
        ros::spinOnce();
        if (JustinaManip::objOnLeftHand()) {
            if(usingTorse){
                JustinaManip::startTorsoGoTo(goalTorso + 0.03, 0, 0);
                JustinaManip::waitForTorsoGoalReached(5000);
            }else
                JustinaManip::laGoToCartesian(objToGraspX - 0.13, objToGraspY + 0.04, objToGraspZ, 0, 0, 1.5708, 0, 5000);
            JustinaNavigation::moveDist(-0.35, 3000);
            JustinaManip::laGoTo("navigation", 5000);
            std::cout
                << "The object was grasp with the left arm in the first test"
                << std::endl;
            return true;
        }
        JustinaNavigation::moveDist(-0.2, 3000);
        if(!JustinaManip::isLaInPredefPos("navigation"))
            JustinaManip::laGoTo("navigation", 5000);
        else
            std::cout << "JustinaTasks.->The left arm already has in the navigation pose" << std::endl;
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        if (JustinaManip::objOnLeftHand()) {
            std::cout
                << "The object was grasp with the left arm in the second test"
                << std::endl;
            return true;
        }
        std::cout << "The object was not grasp with the left arm" << std::endl;
        return false;
    } else {
        if(!JustinaManip::isRaInPredefPos("navigation"))
            JustinaManip::raGoTo("navigation", 10000);
        else
            std::cout << "JustinaTasks.->The right arm already has in the navigation pose" << std::endl;

        JustinaManip::startRaOpenGripper(0.8);
        //Move the manipulator to object


        JustinaManip::raGoToCartesian(objToGraspX - 0.06, objToGraspY - 0.25,
                objToGraspZ, 3000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));

        JustinaManip::raGoToCartesian(objToGraspX - 0.06, objToGraspY - 0.15,
                objToGraspZ, 3000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));

        JustinaManip::raGoToCartesian(objToGraspX + 0.035, objToGraspY - 0.05,
                objToGraspZ, 3000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));

        JustinaNavigation::moveDist(0.08, 3000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1500));

        JustinaManip::startRaCloseGripper(0.5);
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
        ros::spinOnce();
        if (JustinaManip::objOnRightHand()) {
            if(usingTorse){
                JustinaManip::startTorsoGoTo(goalTorso + 0.03, 0, 0);
                JustinaManip::waitForTorsoGoalReached(6000);
            }else
                JustinaManip::raGoToCartesian(objToGraspX - 0.1, objToGraspY + 0.04, objToGraspZ, 0, 0, 1.5708, 0, 5000);
            JustinaNavigation::moveDist(-0.35, 3000);
            JustinaManip::raGoTo("navigation", 5000);
            std::cout
                << "The object was grasp with the right arm in the first test"
                << std::endl;
            return true;
        }
        JustinaNavigation::moveDist(-0.2, 3000);
        if(!JustinaManip::isRaInPredefPos("navigation"))
            JustinaManip::raGoTo("navigation", 5000);
        else
            std::cout << "JustinaTasks.->The right arm already has in the navigation pose" << std::endl;
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        if (JustinaManip::objOnRightHand()) {
            std::cout
                << "The object was grasp with the right arm in the second test"
                << std::endl;
            return true;
        }
        std::cout << "The object was not grasp with the right arm" << std::endl;
        return false;
    }
    return false;


}

bool JustinaTasks::graspObjectFeedback(float x, float y, float z, bool withLeftArm,
        std::string idObject, bool usingTorse) {
    std::cout
        << "JustinaTasks.->Moving to a good-pose for Feedback-grasping objects with ";
    if (withLeftArm)
        std::cout << "left arm" << std::endl;
    else
        std::cout << "right arm" << std::endl;

    geometry_msgs::Point gripperPos;
    float idealY = withLeftArm ? 0.234 : -0.235;    //It is the distance from the center of the robot, to the center of the arm
    bool objectInHand = false;
    float idealX = 0.47;
    float idealZ = 0.55;                           //It is the ideal height for taking an object when torso is at zero height.

    float torsoSpine, torsoWaist, torsoShoulders;
    float lastRobotX, lastRobotY, lastRobotTheta;
    float robotX, robotY, robotTheta;
    float gripper_x, gripper_y, gripper_z;
    float dxa, dya, dxr, dyr;
    float objToGraspX = x;
    float objToGraspY = y;
    float objToGraspZ = z;
    float movTorsoFromCurrPos;
    float stepX, stepY, stepZ;
    float dy, dz;
    float movFrontal;
    float movLateral;
    float movVertical;
    float goalTorso;
    int waitTime;

    //Get the torso position
    JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist, torsoShoulders);

    std::cout << "JustinaTasks.->ObjToGrasp: " << "  " << objToGraspX << "  "  << objToGraspY
        << "  " << objToGraspZ << std::endl;

    std::cout << "JustinaTasks.->torsoSpine:" << torsoSpine << std::endl;
    std::cout << "JustinaTasks.->idealZ:" << idealZ << std::endl;

    movFrontal = -(idealX - objToGraspX);
    movLateral = -(idealY - objToGraspY);
    movVertical = objToGraspZ - idealZ - torsoSpine;

    goalTorso = torsoSpine + movVertical;


    if (goalTorso < 0.2)
        goalTorso = 0.2;
    if (goalTorso > 0.5)
        goalTorso = 0.5;

    movTorsoFromCurrPos = goalTorso - torsoSpine;  
    waitTime = (int) (30000 * fabs(movTorsoFromCurrPos) / 0.3 + 3000);

    std::cout << "JustinaTasks.->Adjusting with frontal=" << movFrontal
        << " lateral=" << movLateral << " and vertical=" << movVertical
        << std::endl;

    std::cout << "JustinaTasks.->waitTime:" << waitTime << std::endl;


    std::cout << "JustinaTasks.->Trying to reach torso goalPos: " << goalTorso << std::endl;

    JustinaNavigation::getRobotPose(lastRobotX, lastRobotY, lastRobotTheta);

    if(usingTorse)
        JustinaManip::startTorsoGoTo(goalTorso, 0, 0);
    JustinaNavigation::moveLateral(movLateral, 6000);
    JustinaNavigation::moveDist(movFrontal, 6000);
    if(usingTorse)
        JustinaManip::waitForTorsoGoalReached(waitTime);

    bool found = false;
    std::vector<vision_msgs::VisionObject> recognizedObjects;
    int indexFound = 0;
    if (idObject.compare("") != 0)
    {
        JustinaManip::startHdGoTo(0, -0.9);
        JustinaManip::waitForHdGoalReached(5000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        found = JustinaVision::detectObjects(recognizedObjects);
        if (found)
        {
            found = false;
            for (int i = 0; i < recognizedObjects.size(); i++)
            {
                vision_msgs::VisionObject vObject = recognizedObjects[i];
                if (vObject.id.compare(idObject) == 0)
                {
                    found = true;
                    indexFound = i;
                    break;
                }
            }
        }
    }

    //Update the coordinates to grasp object.
    if (found)
    {
        std::cout << "The object was found again, update the new coordinates." << std::endl;
        objToGraspX = recognizedObjects[indexFound].pose.position.x;
        objToGraspY = recognizedObjects[indexFound].pose.position.y;
    }
    else if (!found && idObject.compare("") == 0)
    {
        std::cout
            << "The object was not found again, update new coordinates with the motion of robot."
            << std::endl;


        JustinaNavigation::getRobotPose(robotX, robotY, robotTheta);
        //Adjust the object position according to the new robot pose
        //I don't request again the object position due to the possibility of not recognizing it again
        dxa = (robotX - lastRobotX);
        dya = (robotY - lastRobotY);
        dxr = dxa * cos(robotTheta) + dya * sin(robotTheta);
        dyr = -dxa * sin(robotTheta) + dya * cos(robotTheta);

        objToGraspX -= dxr;
        objToGraspY -= dyr;
        std::cout << "lastRobotX:" << lastRobotX << ",lastRobotY:" << lastRobotY
            << ",lastRobotTheta:" << lastRobotTheta << std::endl;
        std::cout << "robotX:" << robotX << ",robotY:" << robotY
            << ",robotTheta:" << robotTheta << std::endl;
        std::cout << "objToGraspX:" << objToGraspX << ",objToGraspY:"
            << objToGraspY << ",objToGraspZ:" << objToGraspZ << std::endl;
    } else if (!found && idObject.compare("") != 0) {
        JustinaNavigation::moveDist(-0.2, 3000);
        return false;
    }

    //The position it is adjusted and converted to coords wrt to the corresponding arm
    std::string destFrame = withLeftArm ? "left_arm_link0" : "right_arm_link0";
    if (!JustinaTools::transformPoint("base_link", objToGraspX, objToGraspY,
                objToGraspZ, destFrame, objToGraspX, objToGraspY, objToGraspZ)) {
        std::cout << "JustinaTasks.->Cannot transform point. " << std::endl;
        return false;
    }
    std::cout << "JustinaTasks.->Moving ";
    if (withLeftArm)
        std::cout << "left arm";
    else
        std::cout << "right arm";

    std::cout << " to " << objToGraspX << "  " << objToGraspY << "  "
        << objToGraspZ << std::endl;

    if (withLeftArm) {
        JustinaManip::laGoTo("navigation", 7000);
        JustinaManip::startLaOpenGripper(0.6);

        //Move the manipulator to objectOB
        stepX = objToGraspX - 0.06;
        stepY = objToGraspY - 0.25;
        stepZ = objToGraspZ;
        JustinaManip::laGoToCartesian(stepX, stepY, stepZ, 0, 0, 1.5708, 0, 3000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(200));

        stepX = objToGraspX - 0.06;
        stepY = objToGraspY - 0.15;
        stepZ = objToGraspZ;
        JustinaManip::laGoToCartesian(stepX, stepY, stepZ, 0, 0, 1.5708, 0, 3000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(200));

        stepX = objToGraspX - 0.04;
        stepY = objToGraspY - 0.10;
        stepZ = objToGraspZ;
        JustinaManip::laGoToCartesian(stepX, stepY, stepZ, 0, 0, 1.5708, 0, 3000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(200));

        stepX = objToGraspX - 0.02;
        stepY = objToGraspY - 0.10;
        stepZ = objToGraspZ;
        JustinaManip::laGoToCartesian(stepX, stepY, stepZ, 0, 0, 1.5708, 0, 3000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(3000));


        // Verify if the robot can get the GripperPos
        if(!JustinaVision::getGripperPos(gripperPos))
        {
            stepX = objToGraspX + 0.02;
            stepY = objToGraspY - 0.05;
            stepZ = objToGraspZ;
            JustinaManip::laGoToCartesian(stepX, stepY, stepZ, 0, 0, 1.5708, 0, 3000);
        }
        else
        {
            //std::cout << "Gripper_ pos (BT):  " << std::endl;
            //std::cout << gripperPos << std::endl;
            //std::cout << "" ;
            gripper_x = gripperPos.x;
            gripper_y = gripperPos.y;
            gripper_z = gripperPos.z;
            if (!JustinaTools::transformPoint("base_link", gripper_x, gripper_y, gripper_z, destFrame, 
                        gripper_x, gripper_y, gripper_z)) 
                std::cout << "JustinaTasks.->Cannot transform point. " << std::endl;

            //std::cout << "Gripper_ pos (AT):  " << std::endl;
            //std::cout << "x: " << gripper_x << std::endl;
            //std::cout << "y: " << gripper_y << std::endl;
            //std::cout << "z: " << gripper_z << std::endl;


            //std::cout << "Object pos:   " << std::endl;
            //std::cout << "x: " << objToGraspX << std::endl;
            //std::cout << "y: " << objToGraspY << std::endl;
            //std::cout << "z: " << objToGraspZ << std::endl;


            dy = objToGraspY - gripper_y;
            dz = objToGraspZ - gripper_z;

            std::cout << "Correct gripper_coordinates (diff): " << std::endl;
            std::cout << "      dy: "<< dy << std::endl;
            std::cout << "      dz: "<< dz - 0.10 << std::endl;

            stepX = objToGraspX + 0.04;
            stepY = objToGraspY + dy - 0.10;
            stepZ = objToGraspZ + dz;

            std::cout << "Final coordinates: " << std::endl;
            std::cout << "      x: "<< stepX << std::endl;
            std::cout << "      y: "<< stepY << std::endl;
            std::cout << "      z: "<< stepZ << std::endl;

            JustinaManip::laGoToCartesian(stepX, stepY, stepZ, 0.0, 0.0, 1.5708, 0, 3000);
            boost::this_thread::sleep(boost::posix_time::milliseconds(200));
        }

        JustinaNavigation::moveDist(0.05, 3000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

        JustinaManip::startLaCloseGripper(0.5);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        ros::spinOnce();
        if (JustinaManip::objOnLeftHand()) {
            if(usingTorse){
                JustinaManip::startTorsoGoTo(goalTorso + 0.03, 0, 0);
                JustinaManip::waitForTorsoGoalReached(5000);
            }else
                JustinaManip::laGoToCartesian(objToGraspX - 0.13, objToGraspY + 0.04, objToGraspZ, 0, 0, 1.5708, 0, 5000);
            JustinaNavigation::moveDist(-0.35, 3000);
            JustinaManip::laGoTo("navigation", 5000);
            std::cout
                << "The object was grasp with the left arm in the first test"
                << std::endl;
            return true;
        }

        JustinaNavigation::moveDist(-0.2, 3000);
        JustinaManip::laGoTo("navigation", 5000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        if (JustinaManip::objOnLeftHand()) {
            std::cout
                << "The object was grasp with the left arm in the second test"
                << std::endl;
            return true;
        }
        std::cout << "The object was not grasp with the left arm" << std::endl;
        return false;
    } 
    else 
    {
        JustinaManip::raGoTo("navigation", 10000);
        JustinaManip::startRaOpenGripper(0.8);

        //Move the manipulator to object
        stepX = objToGraspX - 0.06;
        stepY = objToGraspY - 0.25;
        stepZ = objToGraspZ;
        JustinaManip::raGoToCartesian(stepX, stepY, stepZ, 0, 0, 1.5708, 0, 3000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(200));

        stepX = objToGraspX - 0.06;
        stepY = objToGraspY - 0.15;
        stepZ = objToGraspZ;
        JustinaManip::raGoToCartesian(stepX, stepY, stepZ, 0, 0, 1.5708, 0, 3000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(200));

        stepX = objToGraspX - 0.04;
        stepY = objToGraspY - 0.10;
        stepZ = objToGraspZ;
        JustinaManip::raGoToCartesian(stepX, stepY, stepZ, 0, 0, 1.5708, 0, 3000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(200));

        stepX = objToGraspX - 0.02;
        stepY = objToGraspY - 0.10;
        stepZ = objToGraspZ;
        JustinaManip::raGoToCartesian(stepX, stepY, stepZ, 0, 0, 1.5708, 0, 3000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

        // Verify if the robot can get the GripperPos
        if(!JustinaVision::getGripperPos(gripperPos))
        {
            stepX = objToGraspX + 0.02;
            stepY = objToGraspY - 0.05;
            stepZ = objToGraspZ;
            JustinaManip::raGoToCartesian(stepX, stepY, stepZ, 0, 0, 1.5708, 0, 3000);
        }
        else
        {
            std::cout << "Gripper_ pos (BT):  " << std::endl;
            std::cout << gripperPos << std::endl;
            std::cout << "" ;

            gripper_x = gripperPos.x;
            gripper_y = gripperPos.y;
            gripper_z = gripperPos.z;
            if (!JustinaTools::transformPoint("base_link", gripper_x, gripper_y, gripper_z, destFrame, 
                        gripper_x, gripper_y, gripper_z)) 
                std::cout << "JustinaTasks.->Cannot transform point. " << std::endl;

            //std::cout << "Gripper_ pos (AT):  " << std::endl;
            //std::cout << "x: " << gripper_x << std::endl;
            //std::cout << "y: " << gripper_y << std::endl;
            //std::cout << "z: " << gripper_z << std::endl;



            //std::cout << "Object pos:   " << std::endl;
            //std::cout << "x: " << objToGraspX << std::endl;
            //std::cout << "y: " << objToGraspY << std::endl;
            //std::cout << "z: " << objToGraspZ << std::endl;


            dy = objToGraspY - gripper_y;
            dz = objToGraspZ - gripper_z;

            std::cout << "Correct gripper_coordinates (diff): " << std::endl;
            std::cout << "      dy: "<< dy << std::endl;
            std::cout << "      dz: "<< dz - 0.10 << std::endl;

            stepX = objToGraspX + 0.04;
            stepY = objToGraspY + dy - 0.10;
            stepZ = objToGraspZ + dz;

            std::cout << "Final coordinates: " << std::endl;
            std::cout << "      x: "<< stepX << std::endl;
            std::cout << "      y: "<< stepY << std::endl;
            std::cout << "      z: "<< stepZ << std::endl;

            JustinaManip::raGoToCartesian(stepX, stepY, stepZ, 0.2, 0.0, 1.5708, 0, 3000);
            boost::this_thread::sleep(boost::posix_time::milliseconds(200));
        }

        JustinaNavigation::moveDist(0.05, 3000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

        JustinaManip::startRaCloseGripper(0.5);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        ros::spinOnce();
        if (JustinaManip::objOnRightHand()) {
            if(usingTorse){
                JustinaManip::startTorsoGoTo(goalTorso + 0.03, 0, 0);
                JustinaManip::waitForTorsoGoalReached(6000);
            }else
                JustinaManip::raGoToCartesian(objToGraspX - 0.1, objToGraspY + 0.04, objToGraspZ, 0, 0, 1.5708, 0, 5000);
            JustinaNavigation::moveDist(-0.35, 3000);
            JustinaManip::raGoTo("navigation", 5000);
            std::cout
                << "The object was grasp with the right arm in the first test"
                << std::endl;
            return true;
        }
        JustinaNavigation::moveDist(-0.2, 3000);
        if(!JustinaManip::isRaInPredefPos("navigation"))
            JustinaManip::raGoTo("navigation", 5000);
        else
            std::cout << "JustinaTasks.->The right arm already has in the navigation pose" << std::endl;
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        if (JustinaManip::objOnRightHand()) {
            std::cout
                << "The object was grasp with the right arm in the second test"
                << std::endl;
            return true;
        }
        std::cout << "The object was not grasp with the right arm" << std::endl;
        return false;
    }
    return false;


}



void JustinaTasks::sayAndAsyncNavigateToLoc(std::string location, bool say) {
    std::stringstream ss;
    std::cout << "Navigation to " << location << std::endl;
    ss << "I will navigate to the " << location;
    if (say)
        JustinaHRI::say(ss.str());
    JustinaNavigation::startGetClose(location);
}

bool JustinaTasks::sayAndSyncNavigateToLoc(std::string location, int timeout,
        bool say) {
    std::stringstream ss;
    std::cout << "Navigation to " << location << std::endl;
    ss << "I will navigate to the " << location;
    if (say)
        JustinaHRI::say(ss.str());
    bool reachedLocation = JustinaNavigation::getClose(location, timeout);
    ss.str("");
    if (reachedLocation) {
        ss << "I have reached the " << location;
        if (say)
            JustinaHRI::waitAfterSay(ss.str(), 4000);
    } else {
        ss.str("");
        ss << "I cannot reached the " << location;
        if (say)
            JustinaHRI::waitAfterSay(ss.str(), 4000);
    }
    return reachedLocation;
}

bool JustinaTasks::waitRecognizedFace(
        float timeout, std::string id, int gender, POSE pose, std::vector<vision_msgs::VisionFaceObject> &facesRecog) {
    boost::posix_time::ptime curr;
    boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
    boost::posix_time::time_duration diff;
    bool recognized;
    std::vector<vision_msgs::VisionFaceObject> lastRecognizedFaces;
    do {
        if (id.compare("") == 0)
            JustinaVision::facRecognize();
        else
            JustinaVision::facRecognize(id);
        boost::this_thread::sleep(boost::posix_time::milliseconds(200));
        ros::spinOnce();
        JustinaVision::getLastRecognizedFaces(lastRecognizedFaces);
        curr = boost::posix_time::second_clock::local_time();
    } while (ros::ok() && (curr - prev).total_milliseconds() < timeout
            && lastRecognizedFaces.size() == 0);

    if(pose != NONE){
        for(int i = 0; i < lastRecognizedFaces.size(); i++){
            if(pose == STANDING && lastRecognizedFaces[i].face_centroid.z > 1.2)
                facesRecog.push_back(lastRecognizedFaces[i]);  
            else if(pose == SITTING && lastRecognizedFaces[i].face_centroid.z > 0.8 && lastRecognizedFaces[i].face_centroid.z <= 1.2)
                facesRecog.push_back(lastRecognizedFaces[i]);  
            else if(pose == LYING && lastRecognizedFaces[i].face_centroid.z > 0.1 && lastRecognizedFaces[i].face_centroid.z <= 0.8)
                facesRecog.push_back(lastRecognizedFaces[i]);
        }
        lastRecognizedFaces = facesRecog;
        facesRecog.clear();
    }

    if(gender != -1){
        for(int i = 0; i < lastRecognizedFaces.size(); i++){
            if(lastRecognizedFaces[i].gender == gender)
                facesRecog.push_back(lastRecognizedFaces[i]);
        }
    }
    else
        facesRecog = lastRecognizedFaces;

    if (facesRecog.size() > 0)
        recognized = true;
    else
        recognized = false;
    std::cout << "recognized:" << recognized << std::endl;
    return recognized;
}

bool JustinaTasks::waitRecognizedGesture(std::vector<vision_msgs::GestureSkeleton> &gestures, float timeout){
    boost::posix_time::ptime curr;
    boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
    boost::posix_time::time_duration diff;
    bool recognized;
    do {
        boost::this_thread::sleep(boost::posix_time::milliseconds(200));
        ros::spinOnce();
        JustinaVision::getLastGesturesRecognize(gestures);
        curr = boost::posix_time::second_clock::local_time();
    } while (ros::ok() && (curr - prev).total_milliseconds() < timeout
            && gestures.size() == 0);

    if (gestures.size() > 0)
        recognized = true;
    else
        recognized = false;
    std::cout << "recognized:" << recognized << std::endl;
    return recognized;
}

bool JustinaTasks::getNearestRecognizedFace(std::vector<vision_msgs::VisionFaceObject> facesObject, float distanceMax, Eigen::Vector3d &faceCentroid, int &genderRecog) {
    int indexMin;
    float distanceMin = 99999999.0;
    faceCentroid = Eigen::Vector3d::Zero();
    bool found = false;
    for (int i = 0; i < facesObject.size(); i++) {
        vision_msgs::VisionFaceObject vro = facesObject[i];
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        centroid(0, 0) = vro.face_centroid.x;
        centroid(1, 0) = vro.face_centroid.y;
        centroid(2, 0) = vro.face_centroid.z;
        float dist = centroid.norm();
        if (dist < distanceMax && dist < distanceMin) {
            indexMin = i;
            distanceMin = dist;
            found = true;
        }
    }
    if (found) {
        std::cout << "I found the centroid nearest to robot" << std::endl;
        faceCentroid(0, 0) = facesObject[indexMin].face_centroid.x;
        faceCentroid(1, 0) = facesObject[indexMin].face_centroid.y;
        faceCentroid(2, 0) = facesObject[indexMin].face_centroid.z;
        genderRecog = facesObject[indexMin].gender;
    }
    std::cout << "Face centroid:" << faceCentroid(0, 0) << ","
        << faceCentroid(1, 0) << "," << faceCentroid(2, 0);
    std::cout << std::endl;
    return found;
}

bool JustinaTasks::turnAndRecognizeFace(std::string id, int gender, POSE pose, float initAngPan, float incAngPan, float maxAngPan, float initAngTil, float incAngTil, float maxAngTil, float incAngleTurn, float maxAngleTurn, Eigen::Vector3d &centroidFace, int &genderRecog) {

    bool recog = false;
    bool moveBase = false;
    float initTil = initAngTil;
    float incTil = incAngTil;
    bool direction = false;
    centroidFace = Eigen::Vector3d::Zero();

    if(pose == STANDING)
        maxAngTil = initAngTil;

    for(float baseTurn = incAngleTurn; ros::ok() && baseTurn <= maxAngleTurn && !recog; baseTurn+=incAngleTurn){
        for(float headPanTurn = initAngPan; ros::ok() && headPanTurn <= maxAngPan && !recog; headPanTurn+=incAngPan){
            float currTil;
            for (float headTilTurn = initTil; ros::ok() && ((!direction && headTilTurn >= maxAngTil) || (direction && headTilTurn <= initAngTil)) && !recog; headTilTurn+=incTil){
                currTil = headTilTurn;
                JustinaManip::startHdGoTo(headPanTurn, headTilTurn);
                if(moveBase){
                    JustinaNavigation::moveDistAngle(0.0, incAngleTurn, 4000);
                    moveBase = false;
                }
                JustinaManip::waitForHdGoalReached(3000);
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                std::vector<vision_msgs::VisionFaceObject> facesObject;
                recog = waitRecognizedFace(2000, id, gender, pose, facesObject);
                if(recog)
                    recog = getNearestRecognizedFace(facesObject, 4.0, centroidFace, genderRecog);
            }
            initTil = currTil;
            direction ^= true;
            incTil *= -1; 
        }
        moveBase = true;
    }
    return recog;
}

bool JustinaTasks::getNearestRecognizedGesture(std::string typeGesture, std::vector<vision_msgs::GestureSkeleton> gestures, float distanceMax, Eigen::Vector3d &nearestGesture){
    int indexMin;
    float distanceMin = 99999999.0;
    bool found = false;
    for (int i = 0; i < gestures.size(); i++) {
        vision_msgs::GestureSkeleton g = gestures[i];

        if(typeGesture.compare("waving") != 0){
            if(g.gesture.compare(typeGesture) != 0)
                continue;
        }
        else{
            if(!(g.gesture.compare("left_hand_rised") || g.gesture.compare("right_hand_rised")))
                continue;
        }
        Eigen::Vector3d pos = Eigen::Vector3d::Zero();
        pos(0, 0) = g.gesture_centroid.x;
        pos(1, 0) = g.gesture_centroid.y;
        pos(2, 0) = g.gesture_centroid.z;
        float dist = pos.norm();
        if (dist < distanceMax && dist < distanceMin) {
            indexMin = i;
            distanceMin = dist;
            found = true;
        }
    }
    if(!found)
        return false;
    std::cout << "I found the gesture nearest to robot" << std::endl;
    nearestGesture(0, 0) = gestures[indexMin].gesture_centroid.x;
    nearestGesture(1, 0) = gestures[indexMin].gesture_centroid.y;
    nearestGesture(2, 0) = gestures[indexMin].gesture_centroid.z;
    std::cout << "Gesture centroid:" << nearestGesture(0, 0) << "," << nearestGesture(1, 0) << "," << nearestGesture(2, 0);
    std::cout << std::endl;
    return true;
}

bool JustinaTasks::turnAndRecognizeGesture(std::string typeGesture, float initAngPan, float incAngPan, float maxAngPan, float initAngTil, float incAngTil, float maxAngTil, float incAngleTurn, float maxAngleTurn, Eigen::Vector3d &gesturePos){

    bool recog = false;
    bool moveBase = false;
    float initTil = initAngTil;
    float incTil = incAngTil;
    bool direction = false;
    Eigen::Vector3d centroidGesture = Eigen::Vector3d::Zero();

    for(float baseTurn = incAngleTurn; ros::ok() && baseTurn <= maxAngleTurn && !recog; baseTurn+=incAngleTurn){
        for(float headPanTurn = initAngPan; ros::ok() && headPanTurn <= maxAngPan && !recog; headPanTurn+=incAngPan){
            float currTil;
            for (float headTilTurn = initTil; ros::ok() && ((!direction && headTilTurn >= maxAngTil) || (direction && headTilTurn <= initAngTil)) && !recog; headTilTurn+=incTil){
                currTil = headTilTurn;
                JustinaManip::startHdGoTo(headPanTurn, headTilTurn);
                if(moveBase){
                    JustinaNavigation::moveDistAngle(0.0, incAngleTurn, 4000);
                    moveBase = false;
                }
                JustinaManip::waitForHdGoalReached(3000);
                std::vector<vision_msgs::GestureSkeleton> gestures;
                recog = waitRecognizedGesture(gestures, 2000);
                if(recog)
                    recog = getNearestRecognizedGesture(typeGesture, gestures, 3.0, centroidGesture);
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
            }
            initTil = currTil;
            direction ^= true;
            incTil *= -1; 
        }
        moveBase = true;
    }
    if(recog)
        gesturePos = centroidGesture;
    return recog;
}

bool JustinaTasks::findPerson(std::string person, int gender, POSE pose, bool recogByID) {

    std::vector<int> facesDistances;
    std::stringstream ss;
    std::string personID = "";

    JustinaVision::startFaceRecognitionOld();

    JustinaManip::startHdGoTo(0, 0.0);
    JustinaManip::waitForHdGoalReached(5000);

    std::cout << "Find a person " << person << std::endl;

    ss << person << ", I am going to find you";
    //JustinaHRI::waitAfterSay(ss.str(), 2000);
    JustinaHRI::insertAsyncSpeech(ss.str(), 500);
    JustinaHRI::asyncSpeech();

    Eigen::Vector3d centroidFace;
    int genderRecog;
    if (recogByID) personID = person;
    bool recog = turnAndRecognizeFace(personID, gender, pose, -M_PI_4, M_PI_4 / 2.0, M_PI_4, 0, -M_PI_4, -M_PI_4, M_PI_2, 2 * M_PI, centroidFace, genderRecog);
    std::cout << "Centroid Face in coordinates of robot:" << centroidFace(0, 0)
        << "," << centroidFace(1, 0) << "," << centroidFace(2, 0) << ")";
    std::cout << std::endl;
    //personLocation.clear();
    JustinaVision::stopFaceRecognition();

    ss.str("");
    if (!recog) {
        std::cout << "I have not found a person " << person << std::endl;
        (recogByID) ? ss << "I did not find you " << person : ss << "I did not find a person";
        //JustinaHRI::waitAfterSay(ss.str(), 2000);
        JustinaHRI::insertAsyncSpeech(ss.str(), 500);
        JustinaHRI::asyncSpeech();
        return false;
    }

    std::cout << "I have found a person " << person << std::endl;
    //ss << person << ", I found you";
    (recogByID) ? ss << person << ", I found you" : ss << ", I find a person";
    //JustinaHRI::waitAfterSay(ss.str(), 2000);
    JustinaHRI::insertAsyncSpeech(ss.str(), 500);
    JustinaHRI::insertAsyncSpeech("I am getting close to you", 500);
    JustinaHRI::asyncSpeech();

    float cx, cy, cz;
    cx = centroidFace(0, 0);
    cy = centroidFace(1, 0);
    cz = centroidFace(2, 0);
    float dis = sqrt( pow(cx, 2) + pow(cy, 2) );
    JustinaTools::transformPoint("/base_link", cx, cy, cz, "/map", cx, cy, cz);
    tf::Vector3 worldFaceCentroid(cx, cy, cz);

    int waitToClose = (int) (dis * 10000);
    std::cout << "JustinaTasks.->dis:" << dis << std::endl;
    std::cout << "JustinaTasks.->waitToClose:" << waitToClose << std::endl;

    //JustinaHRI::waitAfterSay("I am getting close to you", 2000);
    closeToGoalWithDistanceTHR(worldFaceCentroid.x(), worldFaceCentroid.y(), 1.0, waitToClose);

    return true;
}

bool JustinaTasks::findGesturePerson(std::string gesture){
    //std::vector<int> facesDistances;
    std::stringstream ss;
    std::string gestureSpeech;

    JustinaVision::startSkeletonFinding();

    JustinaManip::startHdGoTo(0, 0.0);
    JustinaManip::waitForHdGoalReached(5000);

    if(gesture.compare("pointing_left") == 0)
        gestureSpeech = "pointing left";
    if(gesture.compare("pointing_right") == 0)
        gestureSpeech = "pointing left";
    if(gesture.compare("right_hand_rised") == 0)
        gestureSpeech = "right hand rised";
    if(gesture.compare("left_hand_rised") == 0)
        gestureSpeech = "left hand rised";

    std::cout << "Find a " << gestureSpeech << " person" << std::endl;

    ss << "I am going to find you";
    JustinaHRI::insertAsyncSpeech(ss.str(), 500);
    JustinaHRI::asyncSpeech();
    //JustinaHRI::waitAfterSay(ss.str(), 2000);

    Eigen::Vector3d centroidGesture;
    bool recog = JustinaTasks::turnAndRecognizeGesture(gesture, -M_PI_4, M_PI_4 / 2.0, M_PI_4, -0.3, -0.2, -0.5, M_PI_2, 2 * M_PI, centroidGesture);
    std::cout << "Centroid Gesture in coordinates of robot:" << centroidGesture(0, 0) << "," << centroidGesture(1, 0) << "," << centroidGesture(2, 0) << ")";
    std::cout << std::endl;
    JustinaVision::stopSkeletonFinding();

    ss.str("");
    if (!recog) {
        std::cout << "I have not found a person" << std::endl;
        ss << "I did not find the person ";
        JustinaHRI::insertAsyncSpeech(ss.str(), 500);
        JustinaHRI::asyncSpeech();
        //JustinaHRI::waitAfterSay(ss.str(), 2000);
        return false;
    }

    std::cout << "I have found a " << gestureSpeech << " person" << std::endl;
    ss << "I found you";
    JustinaHRI::insertAsyncSpeech(ss.str(), 500);
    JustinaHRI::insertAsyncSpeech("I am getting close to you", 500);
    //JustinaHRI::waitAfterSay(ss.str(), 2000);
    JustinaHRI::asyncSpeech();

    float cx, cy, cz;
    cx = centroidGesture(0, 0);
    cy = centroidGesture(1, 0);
    cz = centroidGesture(2, 0);
    float dis = sqrt( pow(cx, 2) + pow(cy, 2) );
    JustinaTools::transformPoint("/base_link", cx, cy, cz, "/map", cx, cy, cz);
    tf::Vector3 wgc(cx, cy, cz);

    int waitToClose = (int) (dis * 10000);
    std::cout << "JustinaTasks.->dis:" << dis << std::endl;
    std::cout << "JustinaTasks.->waitToClose:" << waitToClose << std::endl;

    //JustinaHRI::waitAfterSay("I am getting close to you", 2000);
    closeToGoalWithDistanceTHR(wgc.x(), wgc.y(), 1.0, waitToClose);

    return true;
}

void JustinaTasks::closeToGoalWithDistanceTHR(float goalX, float goalY, float thr, float timeout){
    float currx, curry, currtheta;
    bool finishReachedPerson = false;

    float distanceToGoal;
    JustinaNavigation::getRobotPose(currx, curry, currtheta);
    distanceToGoal = sqrt(
            pow(goalX - currx, 2)
            + pow(goalY - curry, 2));
    if (distanceToGoal > thr) {
        JustinaNavigation::startGetClose(goalX, goalY);
        boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
        boost::posix_time::ptime curr = prev;
        do {
            JustinaNavigation::getRobotPose(currx, curry, currtheta);
            distanceToGoal = sqrt(pow(goalX - currx, 2) + pow(goalY - curry, 2));
            if ((JustinaNavigation::obstacleInFront() && distanceToGoal < thr) || distanceToGoal < thr)
                finishReachedPerson = true;
            else
                boost::this_thread::sleep(boost::posix_time::milliseconds(100));
            ros::spinOnce();
            curr = boost::posix_time::second_clock::local_time();
        } while (ros::ok() && !finishReachedPerson && ((curr - prev).total_milliseconds() < timeout || timeout == 0));
        std::cout << "JustinaTasks.->I have the reached position." << std::endl;
        JustinaHardware::stopRobot();
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        ros::spinOnce();
    } else
        std::cout << "JustinaTasks.->Robot dont need to move." << std::endl;

    float thetaToGoal = atan2(goalY - curry, goalX - currx);
    if (thetaToGoal < 0.0f)
        thetaToGoal = 2 * M_PI + thetaToGoal;
    float theta = thetaToGoal - currtheta;
    std::cout << "JustinaTasks.->Turn in direction of robot:" << theta << std::endl;
    JustinaNavigation::moveDistAngle(0, theta, 2000);

    JustinaManip::startHdGoTo(0, 0.0);
    JustinaManip::waitForHdGoalReached(5000);
}

bool JustinaTasks::findAndFollowPersonToLoc(std::string goalLocation) {
    //bool found = findPerson();
    //if (!found)
    //    return false;
    std::stringstream ss;
    ss << "I am going to follow you to the " << goalLocation;
    std::cout << "Follow to the " << goalLocation << std::endl;
    JustinaHRI::say(ss.str());

    ss.str("");
    ss << "Please put in front of me";
    JustinaHRI::say(ss.str()); 

    JustinaHRI::enableLegFinder(true);

    while (ros::ok() && !JustinaHRI::frontalLegsFound()) {
        std::cout << "Not found a legs try to found." << std::endl;
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        ros::spinOnce();
    }

    JustinaHRI::startFollowHuman();

    float currx, curry, currtheta;
    float errorx, errory;
    float dis;

    std::map<std::string, std::vector<float> > locations;
    JustinaKnowledge::getKnownLocations(locations);
    std::vector<float> location = locations.find(goalLocation)->second;
    do {
        JustinaNavigation::getRobotPose(currx, curry, currtheta);
        errorx = currx - location[0];
        errory = curry - location[1];
        dis = sqrt(pow(errorx, 2) + pow(errory, 2));
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        ros::spinOnce();
    } while (ros::ok() && dis > 1.6);

    std::cout << "I have reach a location to follow a person in the "
        << goalLocation << std::endl;
    ss.str("");
    ss << "I have finish follow a person ";
    JustinaHRI::say(ss.str());

    JustinaHRI::stopFollowHuman();

    JustinaHRI::enableLegFinder(false);

    return true;
}

bool JustinaTasks::tellGenderPerson(std::string &gender){
    std::stringstream ss;

    JustinaVision::startFaceRecognitionOld();

    JustinaManip::startHdGoTo(0, 0.0);
    JustinaManip::waitForHdGoalReached(5000);

    std::cout << "Find a person " << std::endl;

    ss<< " I am going to find you";
    //JustinaHRI::waitAfterSay(ss.str(), 2000);
    JustinaHRI::insertAsyncSpeech(ss.str(), 500);
    JustinaHRI::asyncSpeech();

    Eigen::Vector3d centroidFace;
    int genderRecog;
    // The second parametter is -1 for all gender person
    bool recog = turnAndRecognizeFace("", -1, NONE,-M_PI_4, M_PI_4 / 2.0, M_PI_4, 0, -M_PI_4, -M_PI_4, M_PI_2, 2 * M_PI, centroidFace, genderRecog);
    std::cout << "Centroid Face in coordinates of robot:" << centroidFace(0, 0)
        << "," << centroidFace(1, 0) << "," << centroidFace(2, 0) << ")";
    std::cout << std::endl;
    //personLocation.clear();
    JustinaVision::stopFaceRecognition();

    ss.str("");
    if (!recog) {
        std::cout << "I have not found a person " << std::endl;
        ss << "I did not find the person ";
        //JustinaHRI::waitAfterSay(ss.str(), 2000);
        JustinaHRI::insertAsyncSpeech(ss.str(), 500);
        JustinaHRI::asyncSpeech();
        return false;
    }

    std::cout << "I have found a person " << std::endl;
    ss << "I found you";
    //JustinaHRI::waitAfterSay(ss.str(), 2000);
    JustinaHRI::insertAsyncSpeech(ss.str(), 500);
    JustinaHRI::insertAsyncSpeech("I am getting close to you", 500);
    JustinaHRI::insertAsyncSpeech("I have verified the information", 2000);
    JustinaHRI::asyncSpeech();

    float cx, cy, cz;
    cx = centroidFace(0, 0);
    cy = centroidFace(1, 0);
    cz = centroidFace(2, 0);
    float dis = sqrt( pow(cx, 2) + pow(cy, 2) );
    JustinaTools::transformPoint("/base_link", cx, cy, cz, "/map", cx, cy, cz);
    tf::Vector3 worldFaceCentroid(cx, cy, cz);

    int waitToClose = (int) (dis * 10000);
    std::cout << "JustinaTasks.->dis:" << dis << std::endl;
    std::cout << "JustinaTasks.->waitToClose:" << waitToClose << std::endl;

    //JustinaHRI::waitAfterSay("I am getting close to you", 2000);
    closeToGoalWithDistanceTHR(worldFaceCentroid.x(), worldFaceCentroid.y(), 1.0, waitToClose);

    JustinaVision::startFaceRecognitionOld();
    //JustinaHRI::waitAfterSay("I have verified the information", 4000);
    std::vector<vision_msgs::VisionFaceObject> facesObject;
    // -1 is for all gender person
    recog = waitRecognizedFace(2000, "", -1, NONE, facesObject);
    JustinaVision::stopFaceRecognition();
    if (recog){
        int genderRecogConfirm;
        Eigen::Vector3d centroidFaceConfirm;
        recog = getNearestRecognizedFace(facesObject, 3.0, centroidFaceConfirm, genderRecogConfirm);
        if(genderRecog == genderRecogConfirm){
            if(genderRecog == 0)
                gender = "female";
            else
                gender =  "male";
        }
        if(genderRecogConfirm == 0)
            gender = "female";
        else
            gender =  "male";
    }
    if(genderRecog == 0)
        gender = "female";
    else
        gender = "male";
    return true;
}

bool JustinaTasks::getPanoramic(float initAngTil, float incAngTil, float maxAngTil, float initAngPan, float incAngPan, float maxAngPan, sensor_msgs::Image &image, float timeout){
    bool genPano = false;
    float initTil = initAngTil;
    float incTil = incAngTil;
    bool direction = false;
    ros::Rate rate(20);
    boost::posix_time::ptime curr;
    boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
    JustinaVision::clearPano();
    for(float headPanTurn = initAngPan; ros::ok() && headPanTurn <= maxAngPan; headPanTurn+=incAngPan){
        float currTil;
        for (float headTilTurn = initTil; ros::ok() && ((!direction && headTilTurn >= maxAngTil) || (direction && headTilTurn <= initAngTil)); headTilTurn+=incTil){
            currTil = headTilTurn;
            JustinaManip::startHdGoTo(headPanTurn, headTilTurn);
            JustinaManip::waitForHdGoalReached(3000);
            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
            JustinaVision::takePano();
            boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        }
        initTil = currTil;
        direction ^= true;
        incTil *= -1; 
    } 

    JustinaManip::startHdGoTo(0.0, 0.0);
    JustinaManip::waitForHdGoalReached(3000);
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    JustinaVision::makePano();
    do{
        rate.sleep();
        ros::spinOnce();
        curr = boost::posix_time::second_clock::local_time();
        genPano = JustinaVision::isPanoImageRecived();
    }while(ros::ok() && (curr - prev).time_duration::total_milliseconds() <= timeout && !genPano);
    if(genPano)
        image = JustinaVision::getLastPanoImage();
    return genPano;
}

bool JustinaTasks::findObject(std::string idObject, geometry_msgs::Pose & pose,
        bool & withLeftOrRightArm) {
    std::vector<vision_msgs::VisionObject> recognizedObjects;
    std::stringstream ss;
    std::string toSpeech = idObject;

    //boost::replace_all(idObject, "_", "-");
    //boost::replace_all(toSpeech, "_", " ");

    std::cout << "Find a object " << idObject << std::endl;

    JustinaManip::startHdGoTo(0, -0.785);
    JustinaManip::waitForHdGoalReached(5000);

    bool found = JustinaVision::detectObjects(recognizedObjects);
    int indexFound = 0;
    if (found) {
        found = false;
        for (int i = 0; i < recognizedObjects.size(); i++) {
            vision_msgs::VisionObject vObject = recognizedObjects[i];
            if (vObject.id.compare(idObject) == 0) {
                found = true;
                indexFound = i;
                break;
            }
        }
    }

    ss.str("");
    if (!found || recognizedObjects.size() == 0) {
        ss << "I can not find the " << toSpeech;
        //JustinaHRI::waitAfterSay(ss.str(), 2000);
        JustinaHRI::insertAsyncSpeech(ss.str(), 500);
        JustinaHRI::asyncSpeech();
        return false;
    }

    ss << "I found the " << toSpeech;
    //JustinaHRI::waitAfterSay(ss.str(), 2000);
    JustinaHRI::insertAsyncSpeech(ss.str(), 500);
    JustinaHRI::asyncSpeech();

    pose = recognizedObjects[indexFound].pose;
    std::cout << "Position:" << pose.position.x << "," << pose.position.y << ","
        << pose.position.z << std::endl;
    std::cout << "Orientation:" << pose.orientation.x << ","
        << pose.orientation.y << "," << pose.orientation.z << ","
        << pose.orientation.w << std::endl;

    if (pose.position.y <= 0)
        withLeftOrRightArm = false;
    else
        withLeftOrRightArm = true;

    std::cout << "JustinaTask.->withLeftOrRightArm:" << withLeftOrRightArm
        << std::endl;

    return true;
}

bool JustinaTasks::moveActuatorToGrasp(float x, float y, float z,
        bool withLeftArm, std::string id, bool usingTorse) {
    std::cout << "Move actuator " << id << std::endl;
    std::stringstream ss;

    if (withLeftArm)
        std::cout << "left arm" << std::endl;
    else
        std::cout << "right arm" << std::endl;

    if(id == "")
        ss << "I am going to take a  Unknown  object.";
    else
        ss << "I am going to take the " << id;

    //JustinaHRI::waitAfterSay(ss.str(), 2000);
    JustinaHRI::insertAsyncSpeech(ss.str(), 500);
    JustinaHRI::asyncSpeech();

    float xf = x, yf = y, zf = z;

    int maxAttemps = 3;
    bool isGrasp = false, isFind = true;
    for (int attemps = 0; attemps < maxAttemps && !isGrasp; attemps++) 
    {
        if (attemps > 0) 
        {
            int attempsToFind = 0, indexFound;
            geometry_msgs::Pose pose;
            std::vector<vision_msgs::VisionObject> recognizedObjects;

            boost::this_thread::sleep(boost::posix_time::milliseconds(500));
            JustinaTasks::alignWithTable(0.42);
            JustinaManip::startHdGoTo(0, -0.85);
            JustinaManip::waitForHdGoalReached(5000);

            isFind = false;
            for (int findAttemps = 0; findAttemps < maxAttemps && !isFind;
                    findAttemps++) {
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                isFind = JustinaVision::detectObjects(recognizedObjects);
                if (isFind) {
                    isFind = false;
                    for (int i = 0; i < recognizedObjects.size(); i++) {
                        vision_msgs::VisionObject vObject = recognizedObjects[i];
                        if (vObject.id.compare(id) == 0) {
                            isFind = true;
                            indexFound = i;
                            break;
                        }
                    }
                }
                if (isFind)
                    pose = recognizedObjects[indexFound].pose;
            }
            if (isFind) {
                xf = pose.position.x;
                yf = pose.position.y;
                zf = pose.position.z;
            }
        }
        if (isFind)
            isGrasp = JustinaTasks::graspObject(xf, yf, zf, withLeftArm,
                    attemps < maxAttemps - 1 ? id : "", usingTorse);
    }

    //JustinaManip::laGoTo("home", 10000);
    return isGrasp;

}

bool JustinaTasks::dropObject(std::string id, bool withLeftOrRightArm, int timeout) {
    float x, y, z;
    geometry_msgs::Point gripperPose;

    boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
    boost::posix_time::ptime curr = prev;

    JustinaManip::hdGoTo(0, 0.0, 5000);
    if (id.compare("") == 0)
        JustinaHRI::waitAfterSay("I am going to give it to you", 2000);
    else {
        std::stringstream ss;
        ss << "I am going to give you the " << id;
        JustinaHRI::waitAfterSay(ss.str(), 2000);
    }
    JustinaHRI::waitAfterSay("please wait", 2000);
    JustinaManip::hdGoTo(0, -0.9, 5000);

    // If withLeftOrRightArm is false the arm to use is the right and else the arm to use is the left.
    if(!withLeftOrRightArm){
        JustinaManip::raGoTo("take", 10000);
        //This is for get gripper with the pose of servos
        //JustinaManip::getRightHandPosition(x, y, z);
    }
    else{
        JustinaManip::laGoTo("take", 10000);
        //This is for get gripper with the pose of servos
        //JustinaManip::getLeftHandPosition(x, y, z);
    }
    if(JustinaVision::getGripperPos(gripperPose)){
        x = gripperPose.x;
        y = gripperPose.y;
        z = gripperPose.z;
    }
    else{
        if(!withLeftOrRightArm)
            JustinaManip::getRightHandPosition(x, y, z);
        else
            JustinaManip::getLeftHandPosition(x, y, z);
    }

    JustinaVision::startHandFrontDetectBB(x, y, z);
    JustinaHRI::waitAfterSay("please put your hand", 2000);

    boost::this_thread::sleep(boost::posix_time::milliseconds(200));
    //JustinaVision::startHandDetectBB(0.50, -0.15, 0.95);
    ros::Rate rate(10);
    while (ros::ok() && !JustinaVision::getDetectionHandFrontBB() && (curr - prev).total_milliseconds() < timeout) {
        rate.sleep();
        ros::spinOnce();
        curr = boost::posix_time::second_clock::local_time();
    }
    JustinaVision::stopHandFrontDetectBB();
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    JustinaHRI::waitAfterSay("I am going hand over the object", 2000);

    if(!withLeftOrRightArm){
        JustinaManip::startRaOpenGripper(0.6);
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
        JustinaManip::startRaOpenGripper(0.0);
        JustinaNavigation::moveDist(-0.25, 2000);
        JustinaManip::raGoTo("navigation", 10000);
        JustinaManip::raGoTo("home", 10000);
    }
    else{
        JustinaManip::startLaOpenGripper(0.6);
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
        JustinaManip::startLaOpenGripper(0.0);
        JustinaNavigation::moveDist(-0.25, 2000);
        JustinaManip::laGoTo("navigation", 10000);
        JustinaManip::laGoTo("home", 10000);
    }
    return true;
}

bool JustinaTasks::detectBagInFront(bool withLeftOrRightArm, int timeout){
    float x, y, z;
    geometry_msgs::Point gripperPose;
    ros::Rate rate(10);
    boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
    boost::posix_time::ptime curr = prev;
    JustinaHRI::waitAfterSay("Please, wait", 4500);
    if(withLeftOrRightArm){
        JustinaManip::laGoTo("take", 4000);
        JustinaManip::startLaOpenGripper(0.6);
    }
    else{
        JustinaManip::raGoTo("take", 4000);
        JustinaManip::startRaOpenGripper(0.6);
    }
    JustinaManip::hdGoTo(0, -0.9, 3000);
    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
    if(JustinaVision::getGripperPos(gripperPose)){
        x = gripperPose.x;
        y = gripperPose.y;
        z = gripperPose.z;
    }
    else{
        if(!withLeftOrRightArm)
            JustinaManip::getRightHandPosition(x, y, z);
        else
            JustinaManip::getLeftHandPosition(x, y, z);
    }
    boost::this_thread::sleep(boost::posix_time::milliseconds(400));
    std::cout << "helMeCarry.->Point(" << x << "," << y << "," << z << ")" << std::endl;
    JustinaVision::startHandFrontDetectBB(x, y, z);
    prev = boost::posix_time::second_clock::local_time();
    curr = prev;
    JustinaHRI::waitAfterSay("Please put the bag in my hand", 3000);
    while(ros::ok() && !JustinaVision::getDetectionHandFrontBB() && (curr - prev).total_milliseconds() < timeout){
        rate.sleep();
        ros::spinOnce();
        curr = boost::posix_time::second_clock::local_time();
    }
    JustinaVision::stopHandFrontDetectBB();
    JustinaHRI::waitAfterSay("Thank you", 1500);
    if(withLeftOrRightArm)
        JustinaManip::startLaCloseGripper(0.4);
    else
        JustinaManip::startRaCloseGripper(0.4);
    boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
    if(withLeftOrRightArm)
        JustinaManip::laGoTo("navigation", 10000);
    else
        JustinaManip::raGoTo("navigation", 10000);
    return true;
}

bool JustinaTasks::dropObjectInBox(std::string id, bool withLeftOrRightArm, int posId) {
    float x, y, z;

    // If withLeftOrRightArm is false the arm to use is the right and else the arm to use is the left.
    if(!withLeftOrRightArm){
        JustinaManip::raGoTo("box", 10000);
        JustinaManip::getRightHandPosition(x, y, z);
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
    }
    else{
        JustinaManip::laGoTo("box", 10000);
        JustinaManip::getLeftHandPosition(x, y, z);
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
    }

    if(!withLeftOrRightArm){
        JustinaManip::startRaOpenGripper(0.6);
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
        JustinaManip::startRaOpenGripper(0.0);
        JustinaManip::raGoTo("navigation", 10000);
    }
    else{
        JustinaManip::startLaOpenGripper(0.6);
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
        JustinaManip::startLaOpenGripper(0.0);
        JustinaManip::laGoTo("navigation", 10000);
    }
    return true;
}

bool JustinaTasks::placeObject(bool withLeftArm, float h, bool placeBag) {
    std::cout << "JustinaTasks::placeObject..." << std::endl;
    std::vector<float> vacantPlane;
    std::vector<int> inliers;
    std::vector<int> inliersRight;
    std::vector<int> inliersLeft;
    std::vector<float> xRight;
    std::vector<float> yRight;
    std::vector<float> zRight;
    std::vector<float> xLeft;
    std::vector<float> yLeft;
    std::vector<float> zLeft;
    std::vector<float> distance;
    float maximunInliers = 0;
    float objToGraspX;
    float objToGraspY;
    float objToGraspZ;
    float lateral;

    int maxInliersIndex;

    int aux =  0;
    int contLeft = 0;
    int contRight = 0;

    JustinaManip::hdGoTo(0, -0.7, 5000);
    if(!JustinaTasks::alignWithTable(0.32))
        JustinaTasks::alignWithTable(0.32);

    if(!JustinaVision::findVacantPlane(vacantPlane, inliers))
    {
        JustinaNavigation::moveDist(0.04, 1000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
        if(!JustinaVision::findVacantPlane(vacantPlane, inliers))
        {
            JustinaNavigation::moveDist(-0.06, 1000);
            boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
            if(!JustinaTasks::alignWithTable(0.32))
            {
                if(!JustinaVision::findVacantPlane(vacantPlane, inliers))
                    return false;
            }else
            {
                if(!JustinaVision::findVacantPlane(vacantPlane, inliers))
                    return false;
            }
        }
    }

    for(int i = 0; i < (vacantPlane.size()) ; i=i+3)
    {
        if (vacantPlane[i+1]>=0.0){
            xLeft.push_back( vacantPlane[ i ] );
            yLeft.push_back( vacantPlane[i+1] );
            zLeft.push_back( vacantPlane[i+2] );
            inliersLeft.push_back(inliers[aux]);
            contLeft ++;
            std::cout << "Justina::Tasks->PlaceObject plano lado izquierdo " << std::endl;
        }
        else{
            xRight.push_back( vacantPlane[ i ] );
            yRight.push_back( vacantPlane[i+1] );
            zRight.push_back( vacantPlane[i+2] );
            inliersRight.push_back(inliers[aux]);
            contRight ++;
            std::cout << "Justina::Tasks->PlaceObject plano lado derecho " << std::endl;
        }

        aux++;
    }


    if(contLeft == 0 && contRight > 0){
        std::cout << "Justina::Tasks->PlaceObject: No hay planos libres del lado izquierdo, se usaran los del derecho " << std::endl;
        for(int i = 0; i < xRight.size() ; i++)
        {
           xLeft.push_back( xRight[i] );
           yLeft.push_back( yRight[i] );
           zLeft.push_back( zRight[i] ); 
           inliersLeft.push_back(inliersRight[i]);
        }
    }
    else if(contRight == 0 && contLeft > 0){
        std::cout << "Justina::Tasks->PlaceObject: No hay planos libres del lado derecho, se usaran los del izquierdo " << std::endl;
        for(int i = 0; i < xLeft.size() ; i++)
        {
           xRight.push_back( xLeft[i] );
           yRight.push_back( yLeft[i] );
           zRight.push_back( zLeft[i] ); 
           inliersRight.push_back(inliersLeft[i]);
        }
    }
    else if(contLeft == 0 && contRight ==0)
        std::cout << "Justina::Tasks->PlaceObject: No hay planos libres" << std::endl;
    else
        std::cout << "Justina::Tasks->PlaceObject: Planos libres en el lado izquierdo y derecho" << std::endl;

    if(withLeftArm){
        for(int i = 0; i < xLeft.size();i++)
        {
            if(inliersLeft[i] > maximunInliers)
            {
                maximunInliers = inliersLeft[i];
                maxInliersIndex = i;
            }
        }
        std::cout << "Justina::Tasks->PlaceObject  P_max[" << maxInliersIndex << "]:  (" << xLeft[maxInliersIndex] << ", " << yLeft[maxInliersIndex] << ", "  << zLeft[maxInliersIndex] << " + " << h << ")" << std::endl;
        std::cout << "Justina::Tasks->PlaceObject  inliers_max[" << maxInliersIndex << "]:  " << inliersLeft[maxInliersIndex] << std::endl;
    }
    else
    {
        for(int i = 0; i < xRight.size();i++)
        {
            if(inliersRight[i] > maximunInliers)
            {
                maximunInliers = inliersRight[i];
                maxInliersIndex = i;
            }
        }
        std::cout << "Justina::Tasks->PlaceObject  P_max[" << maxInliersIndex << "]:  (" << xRight[maxInliersIndex] << ", " << yRight[maxInliersIndex] << ", "  << zRight[maxInliersIndex] << " + " << h << ")" << std::endl;
        std::cout << "Justina::Tasks->PlaceObject  inliers_max[" << maxInliersIndex << "]:  " << inliersRight[maxInliersIndex] << std::endl;
    }
    
    std::string destFrame = withLeftArm ? "left_arm_link0" : "right_arm_link0";

    if(withLeftArm)
    {
        lateral = yLeft[maxInliersIndex]-0.225;
        JustinaNavigation::moveLateral(lateral, 3000);
        yLeft[maxInliersIndex] = 0.22;
        if (!JustinaTools::transformPoint("base_link", xLeft[maxInliersIndex], yLeft[maxInliersIndex],
                    zLeft[maxInliersIndex]+ (zLeft[maxInliersIndex]*0.05) + h, destFrame, objToGraspX, objToGraspY, objToGraspZ))
        {
            std::cout << "JustinaTasks.->Cannot transform point. " << std::endl;
            return false;
        }
        std::cout << "Moving left arm to P[wrtr]:  (" << xLeft[maxInliersIndex] << ", " << yLeft[maxInliersIndex] << ", "  << zLeft[maxInliersIndex]+ (zLeft[maxInliersIndex]*0.05) + h << ")" << std::endl;
        if(!JustinaManip::isLaInPredefPos("navigation"))
        {
            std::cout << "Left Arm is not already on navigation position" << std::endl;
            JustinaManip::laGoTo("navigation", 7000);
        }

        // Verify if the height of plane is longer than 1.2 if not calculate the
        // inverse kinematic.
        if(zLeft[maxInliersIndex] > 1.2)
        {
            JustinaManip::laGoTo("shelf_1", 7000);
            JustinaNavigation::moveDist(0.05, 1000);
            JustinaManip::laGoTo("shelf_2", 7000);
            boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            JustinaManip::startLaOpenGripper(0.3);
            JustinaManip::laGoTo("shelf_1", 7000);
            boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            JustinaNavigation::moveDist(-0.15, 5000);
            JustinaManip::laGoTo("navigation", 7000);
            JustinaManip::startLaOpenGripper(0.0);
            JustinaManip::laGoTo("home", 7000);
            JustinaManip::hdGoTo(0, 0.0, 5000);

        }
        else
        {
            JustinaManip::laGoTo("put1", 6000);
            JustinaManip::laGoToCartesian(objToGraspX, objToGraspY, objToGraspZ, 0, 0, 1.5708, 0, 5000);
            std::cout << "Moving left arm to P[wrta]:  (" << objToGraspX << ", " << objToGraspY << ", "  << objToGraspZ << ")" << std::endl;
            if(placeBag)
            {
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                std::vector<float> currPose;
                JustinaManip::getLaCurrentPos(currPose);
                if(currPose.size() == 7)
                {
                    currPose[5] = -0.785;
                    currPose[6] = -1.5708;
                    JustinaManip::startLaGoToArticular(currPose);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                }
                //JustinaNavigation::moveDist(0.05, 1000);
                JustinaManip::startLaOpenGripper(0.7);
            }
            else{
                //JustinaNavigation::moveDist(0.05, 1000);
                JustinaManip::startLaOpenGripper(0.3);
            }
            boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            JustinaNavigation::moveDist(-0.2, 5000);
            JustinaManip::laGoTo("navigation", 5000);
            JustinaManip::startLaOpenGripper(0.0);
            //JustinaManip::startLaGoTo("home");
            JustinaManip::startHdGoTo(0.0, 0.0);
        }
        if(!placeBag)
            JustinaNavigation::moveLateral(-lateral, 3000);


    }
    else
    {
        lateral = yRight[maxInliersIndex]+0.225;
        JustinaNavigation::moveLateral(lateral, 3000);
        yRight[maxInliersIndex] = -0.22;
        if (!JustinaTools::transformPoint("base_link", xRight[maxInliersIndex], yRight[maxInliersIndex],
                    zRight[maxInliersIndex] + (zRight[maxInliersIndex]*0.05) +h, destFrame, objToGraspX, objToGraspY, objToGraspZ))
        {
            std::cout << "JustinaTasks.->Cannot transform point. " << std::endl;
            return false;
        }
        std::cout << "Moving right arm to P[wrtr]:  (" << xRight[maxInliersIndex] << ", " << yRight[maxInliersIndex] << ", "  << zRight[maxInliersIndex]+ (zRight[maxInliersIndex]*0.05) + h << ")" << std::endl;
        if(!JustinaManip::isRaInPredefPos("navigation"))
        {
            std::cout << "Right Arm is not already on navigation position" << std::endl;
            JustinaManip::raGoTo("navigation", 7000);
        }

        if(zRight[maxInliersIndex] > 1.2)
        {
            JustinaManip::raGoTo("shelf_1", 7000);
            JustinaNavigation::moveDist(0.05, 1000);
            JustinaManip::raGoTo("shelf_2", 7000);
            boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            JustinaManip::startRaOpenGripper(0.3);
            JustinaManip::raGoTo("shelf_1", 7000);
            boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            JustinaNavigation::moveDist(-0.15, 5000);
            JustinaManip::raGoTo("navigation", 7000);
            JustinaManip::startRaOpenGripper(0.0);
            JustinaManip::raGoTo("home", 7000);
            JustinaManip::hdGoTo(0, 0.0, 5000);
        }
        else
        {
            JustinaManip::raGoTo("put1", 6000);
            JustinaManip::raGoToCartesian(objToGraspX, objToGraspY, objToGraspZ, 0, 0, 1.5708, 0, 5000) ;
            std::cout << "Moving right arm to P[wrta]:  (" << objToGraspX << ", " << objToGraspY << ", "  << objToGraspZ << ")" << std::endl;
            if(placeBag)
            {
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                std::vector<float> currPose;
                JustinaManip::getRaCurrentPos(currPose);
                if(currPose.size() == 7)
                {
                    currPose[5] = -0.785;
                    currPose[6] = -1.5708;
                    JustinaManip::startRaGoToArticular(currPose);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                }
                //JustinaNavigation::moveDist(0.05, 1000);
                JustinaManip::startRaOpenGripper(0.7);
            }
            else{
                //JustinaNavigation::moveDist(0.05, 1000);
                JustinaManip::startRaOpenGripper(0.3);
            }
            boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            JustinaNavigation::moveDist(-0.2, 5000);
            JustinaManip::raGoTo("navigation", 5000);
            JustinaManip::startRaOpenGripper(0.0);
            //JustinaManip::startRaGoTo("home");
            JustinaManip::startHdGoTo(0.0, 0.0);

        }
        if(!placeBag)
            JustinaNavigation::moveLateral(-lateral, 3000);
    }

    return true;
}

bool JustinaTasks::placeObjectOnShelf(bool withLeftArm, float h)
{
    std::cout << "-- JustinaTasks::placeObjectOnShelf..." << std::endl;
    std::vector<float> vacantPlane;
    std::vector<int> inliers;
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> z;
    std::vector<float> distance;
    float maximunInliers = 0;
    float XtoPlace;
    float YtoPlace;
    float ZtoPlace;

    bool isFreeSpace = false;

    int maxInliersIndex;

    JustinaManip::hdGoTo(0, -0.7, 5000);
    //JustinaHardware::goalTorso(0.45, 4000);
    if(!JustinaTasks::alignWithTable(0.35))
        if(!JustinaTasks::alignWithTable(0.35))
            if(!JustinaTasks::alignWithTable(0.35))
                if(!JustinaTasks::alignWithTable(0.35))
                    JustinaTasks::alignWithTable(0.35);

    if(!JustinaVision::findVacantPlane(vacantPlane, inliers))
    {
        JustinaNavigation::moveDist(0.04, 1000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
        if(!JustinaVision::findVacantPlane(vacantPlane, inliers))
        {
            JustinaNavigation::moveDist(-0.06, 1000);
            boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
            if(!JustinaTasks::alignWithTable(0.32))
            {
                if(!JustinaVision::findVacantPlane(vacantPlane, inliers))
                    return false;
            }else
            {
                if(!JustinaVision::findVacantPlane(vacantPlane, inliers))
                    return false;
            }
        }
    }



    for(int i = 0; i < (vacantPlane.size()) ; i=i+3)
    {
        x.push_back( vacantPlane[ i ] );
        y.push_back( vacantPlane[i+1] );
        z.push_back( vacantPlane[i+2] );
    }

    for(int i = 0; i < x.size();i++)
    {
        //std::cout << "P[" << i << "]:  (" << x[i] << ", " << y[i] << ", "  << z[i] << ")" << std::endl;
        //std::cout << "inliers[" << i << "]:  " << inliers[i] << std::endl;
        if(z[i] < 1.60)
        {
            if(inliers[i] > maximunInliers)
            {
                maximunInliers = inliers[i];
                maxInliersIndex = i;
            }
            isFreeSpace = true;
        }
    }

    if(!isFreeSpace)
        return false;

    std::cout << "Justina::Tasks->PlaceObject  P_max[" << maxInliersIndex << "]:  (" << x[maxInliersIndex] << ", " << y[maxInliersIndex] << ", "  << z[maxInliersIndex] << " + " << h << ")" << std::endl;
    std::cout << "Justina::Tasks->PlaceObject  inliers_max[" << maxInliersIndex << "]:  " << inliers[maxInliersIndex] << std::endl;

    std::string destFrame = withLeftArm ? "left_arm_link0" : "right_arm_link0";

    if(withLeftArm)
    {
        JustinaNavigation::moveLateral(y[maxInliersIndex]-0.24, 3000);
        y[maxInliersIndex] = 0.22;
        if (!JustinaTools::transformPoint("base_link", x[maxInliersIndex], y[maxInliersIndex],
                    z[maxInliersIndex]+ h, destFrame, XtoPlace, YtoPlace, ZtoPlace))
        {
            std::cout << "JustinaTasks.->Cannot transform point. " << std::endl;
            return false;
        }
        std::cout << "Moving left arm to P[wrtr]:  (" << x[maxInliersIndex] << ", " << y[maxInliersIndex] << ", "  << z[maxInliersIndex]+ (z[maxInliersIndex]*0.05) + h << ")" << std::endl;

        // Verify if the height of plane is longer than 1.2 if not calculate the
        // inverse kinematic.

        JustinaManip::laGoTo("navigation", 6000);

        //boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
        JustinaNavigation::moveDist(-0.10, 5000);

        if(z[maxInliersIndex] > 1.10)
        {
            JustinaNavigation::moveDist(-0.25, 5000);
            JustinaManip::laGoToCartesian(XtoPlace-0.15, YtoPlace+0.05, ZtoPlace, 0, 0, 1.5708, 0, 5000);
            boost::this_thread::sleep(boost::posix_time::milliseconds(3500));
        }
        else
        {
            JustinaManip::laGoToCartesian(XtoPlace-0.05, YtoPlace+0.05, ZtoPlace, 0, 0, 1.5708, 0, 5000);
            boost::this_thread::sleep(boost::posix_time::milliseconds(2500));
        }


        if(z[maxInliersIndex] > 1.10)
        {            
            JustinaNavigation::moveDist(0.15, 5000);
            boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
            JustinaManip::laGoToCartesian(XtoPlace-0.05, YtoPlace+0.05, ZtoPlace, 0, 0, 1.5708, 0, 5000);
            boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
        }

        JustinaNavigation::moveDist(0.08, 5000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
        JustinaManip::startLaOpenGripper(0.3);

        JustinaManip::laGoToCartesian(XtoPlace-0.08, YtoPlace-0.05, ZtoPlace, 0, 0, 1.5708, 0, 5000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1500));

        JustinaNavigation::moveDist(-0.10, 5000);
        if(z[maxInliersIndex] > 1.10)
            JustinaNavigation::moveDist(-0.15, 5000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1500));

        JustinaManip::startLaGoTo("navigation");
        JustinaManip::startHdGoTo(0.0, 0.0);


    }
    else
    {
        JustinaNavigation::moveLateral(y[maxInliersIndex]+0.24, 3000);
        y[maxInliersIndex] = -0.22;
        if (!JustinaTools::transformPoint("base_link", x[maxInliersIndex], y[maxInliersIndex],
                    z[maxInliersIndex] + h, destFrame, XtoPlace, YtoPlace, ZtoPlace))
        {
            std::cout << "JustinaTasks.->Cannot transform point. " << std::endl;
            return false;
        }
        std::cout << "Moving right arm to P[wrtr]:  (" << x[maxInliersIndex] << ", " << y[maxInliersIndex] << ", "  << z[maxInliersIndex]+ (z[maxInliersIndex]*0.05) + h << ")" << std::endl;


        JustinaManip::raGoTo("navigation", 6000);
        JustinaNavigation::moveDist(-0.10, 5000);

        //boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
        if(z[maxInliersIndex] > 1.10)
        {
            JustinaNavigation::moveDist(-0.25, 5000);
            JustinaManip::raGoToCartesian(XtoPlace-0.15, YtoPlace+0.05, ZtoPlace, 0, 0, 1.5708, 0, 5000);
            boost::this_thread::sleep(boost::posix_time::milliseconds(3500));
        }
        else
        {
            JustinaManip::raGoToCartesian(XtoPlace-0.08, YtoPlace+0.05, ZtoPlace, 0, 0, 1.5708, 0, 5000);
            boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
        }


        if(z[maxInliersIndex] > 1.10)
        {

            JustinaNavigation::moveDist(0.15, 5000);
            boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
            JustinaManip::raGoToCartesian(XtoPlace-0.03, YtoPlace+0.05, ZtoPlace, 0, 0, 1.5708, 0, 5000);
            boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
        }

        JustinaNavigation::moveDist(0.08, 5000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
        JustinaManip::startRaOpenGripper(0.3);

        JustinaManip::raGoToCartesian(XtoPlace-0.08, YtoPlace-0.05, ZtoPlace, 0, 0, 1.5708, 0, 5000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1500));

        JustinaNavigation::moveDist(-0.10, 5000);
        if(z[maxInliersIndex] > 1.10)
            JustinaNavigation::moveDist(-0.15, 5000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1500));

        JustinaManip::startRaGoTo("navigation");
        JustinaManip::startHdGoTo(0.0, 0.0);

    }
    return true;

}


bool JustinaTasks::guideAPerson(std::string loc, int timeout){

    STATE nextState = SM_GUIDING_MEMORIZING_OPERATOR_SAY;
    std::stringstream ss;
    std::vector<std::string> tokens;
    bool hokuyoRear;
    bool success = false;
    ros::Rate rate(10);

    boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
    boost::posix_time::ptime curr = prev;

    while(ros::ok() && !success && ((curr - prev).total_milliseconds() < timeout || timeout == 0)){
        switch(nextState){
            case SM_GUIDING_MEMORIZING_OPERATOR_SAY:
                std::cout << "State machine: SM_GUIDING_MEMORIZING_OPERATOR_SAY" << std::endl;
                ss.str("");
                ss << "I will guide you to the ";
                boost::algorithm::split(tokens, loc, boost::algorithm::is_any_of("_"));
                for(int i = 0; i < tokens.size(); i++)
                    ss << tokens[i] << " ";
                JustinaHRI::waitAfterSay(ss.str(), 4000);
                nextState = SM_GUIDING_MEMORIZING_OPERATOR_ELF;
                break;
            case SM_GUIDING_MEMORIZING_OPERATOR_ELF:
                std::cout << "State machine: SM_GUIDING_MEMORIZING_OPERATOR_ELF" << std::endl;
                JustinaHRI::enableLegFinderRear(true);
                nextState = SM_GUIDING_MEMORIZING_OPERATOR;
                break;
            case SM_GUIDING_MEMORIZING_OPERATOR:
                std::cout << "State machine: SM_GUIDING_MEMORIZING_OPERATOR" << std::endl;
                hokuyoRear = JustinaHRI::rearLegsFound();
                if(hokuyoRear){
                    JustinaHRI::waitAfterSay("Ok, let us go", 2500);
                    JustinaNavigation::startGetClose(loc);
                    nextState = SM_GUIDING_PHASE;
                }
                else
                    JustinaHRI::waitAfterSay("Human, stand behind me", 3000);
                break;
            case SM_GUIDING_PHASE:
                std::cout << "State machine: SM_GUIDING_PHASE" << std::endl;
                hokuyoRear = JustinaHRI::rearLegsFound();
                if(!hokuyoRear)
                    nextState=SM_GUIDING_STOP;
                if(JustinaNavigation::isGlobalGoalReached())
                    nextState=SM_GUIDING_FINISHED;
                break;
            case SM_GUIDING_STOP:
                std::cout << "State machine: SM_GUIDING_STOP" << std::endl;
                JustinaHardware::stopRobot();
                ros::spinOnce();
                JustinaHRI::waitAfterSay("I lost you", 1500);
                JustinaHRI::enableLegFinderRear(false);
                nextState=SM_GUIDING_MEMORIZING_OPERATOR_ELF;
                break;
            case SM_GUIDING_FINISHED:
                std::cout << "State machine: SM_GUIDING_FINISHED" << std::endl;
                ss.str("");
                ss << "Her is the ";
                boost::algorithm::split(tokens, loc, boost::algorithm::is_any_of("_"));
                for(int i = 0; i < tokens.size(); i++)
                    ss << tokens[i] << " ";
                JustinaHRI::waitAfterSay(ss.str(), 2500);
                JustinaHRI::enableLegFinderRear(false);
                success = true;
                break;
        }
        rate.sleep();
        ros::spinOnce();
        curr = boost::posix_time::second_clock::local_time();
    }
    if(!success && timeout != 0){
        ss.str("");
        ss << "I cannot guide you to the  ";
        boost::algorithm::split(tokens, loc, boost::algorithm::is_any_of("_"));
        for(int i = 0; i < tokens.size(); i++)
            ss << tokens[i] << " ";
        JustinaHRI::waitAfterSay(ss.str(), 2500);
        JustinaHardware::stopRobot();
    }
    return success;
}

bool JustinaTasks::followAPersonAndRecogStop(std::string stopRecog){
    STATE nextState = SM_WAIT_FOR_OPERATOR;
    bool success = false;
    ros::Rate rate(10);
    std::string lastRecoSpeech;
    std::vector<std::string> validCommandsStop;
    validCommandsStop.push_back(stopRecog);

    while(ros::ok() && !success){

        switch(nextState){
            case SM_WAIT_FOR_OPERATOR:
                std::cout << "State machine: SM_WAIT_FOR_OPERATOR" << std::endl;
                JustinaHRI::waitAfterSay("Please, tell me, follow me for start following you", 3000);
                if(JustinaHRI::waitForSpecificSentence("follow me" , 15000))
                    nextState = SM_MEMORIZING_OPERATOR;
                else
                    nextState = SM_WAIT_FOR_OPERATOR;    		
                break;
            case SM_MEMORIZING_OPERATOR:
                std::cout << "State machine: SM_MEMORIZING_OPERATOR" << std::endl;
                JustinaHRI::waitAfterSay("Human, please put in front of me", 2500);
                JustinaHRI::enableLegFinder(true);
                nextState=SM_WAIT_FOR_LEGS_FOUND;	    
                break;
            case SM_WAIT_FOR_LEGS_FOUND:
                std::cout << "State machine: SM_WAIT_FOR_LEGS_FOUND" << std::endl;
                if(JustinaHRI::frontalLegsFound()){
                    std::cout << "NavigTest.->Frontal legs found!" << std::endl;
                    JustinaHRI::startFollowHuman();
                    JustinaHRI::waitAfterSay("I found you, i will start to follow you human, please walk and tell me, stop follow me, when we reached the goal location", 10000);
                    nextState = SM_FOLLOWING_PHASE;
                }
                break;
            case SM_FOLLOWING_PHASE:
                std::cout << "State machine: SM_FOLLOWING_PHASE" << std::endl;
                if(JustinaHRI::waitForSpecificSentence(validCommandsStop, lastRecoSpeech, 7000)){
                    if(lastRecoSpeech.find(stopRecog) != std::string::npos){
                        JustinaHRI::stopFollowHuman();
                        JustinaHRI::enableLegFinder(false);
                        JustinaHRI::waitAfterSay("I stopped", 1500);
                        nextState = SM_FOLLOWING_FINISHED;
                        break;
                    }
                }
                if(!JustinaHRI::frontalLegsFound()){
                    std::cout << "State machine: SM_FOLLOWING_PHASE -> Lost human!" << std::endl;
                    JustinaHRI::waitAfterSay("I lost you", 1500);
                }        
                break;
            case SM_FOLLOWING_FINISHED:
                std::cout << "State machine: SM_FOLLOWING_FINISHED" << std::endl;
                JustinaHRI::waitAfterSay("I have finished following you", 3000);
                success = true;
                break;
        }

        rate.sleep();
        ros::spinOnce();
    }
    return success;
}

bool JustinaTasks::findTable(std::string &ss)
{
    std::cout << "JustinaTask::findTable" << std::endl;

    //JustinaHRI::waitAfterSay("I am going to search the closes table", 2500);
    JustinaHRI::insertAsyncSpeech("I am going to search the closes table", 500);
    JustinaHRI::asyncSpeech();
    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

    JustinaManip::hdGoTo(0.0, -0.7, 4000);
    boost::this_thread::sleep(boost::posix_time::milliseconds(500));
    //JustinaHRI::waitAfterSay("I am serching table in front of me", 1500);   
    if(JustinaVision::findPlane())
    {
        JustinaHRI::insertAsyncSpeech("I found a table", 500);
        JustinaHRI::asyncSpeech();
        //JustinaHRI::waitAfterSay("I have found a table", 1500);
        ss = "center";
        return true;
    }

    //Turn head to left	
    JustinaManip::hdGoTo(0.9, -0.7, 4000);
    boost::this_thread::sleep(boost::posix_time::milliseconds(500));
    //JustinaHRI::waitAfterSay("I am serching table on my left side", 2500);
    if(JustinaVision::findPlane())
    {
        JustinaHRI::insertAsyncSpeech("I found a table", 500);
        JustinaHRI::asyncSpeech();
        //JustinaHRI::waitAfterSay("I have found a table", 1500);
        JustinaNavigation::startMoveDistAngle(0.0, M_PI_4 / 2.0);
        JustinaManip::hdGoTo(0.0, -0.7, 4000);
        ss = "left";
        return true;
    }

    //Turn head to right	
    JustinaManip::hdGoTo(-0.9, -0.7, 4000);
    boost::this_thread::sleep(boost::posix_time::milliseconds(500));
    //JustinaHRI::waitAfterSay("I am serching table on my right side", 1500);
    if(JustinaVision::findPlane())
    {
        JustinaHRI::insertAsyncSpeech("I found a table", 500);
        JustinaHRI::asyncSpeech();
        //JustinaHRI::waitAfterSay("I have found a table", 1500);
        JustinaNavigation::startMoveDistAngle(0.0, -M_PI_4 / 2.0);
        JustinaManip::hdGoTo(0.0, -0.7, 4000);
        ss = "right";
        return true;
    }

    return false;

}

bool JustinaTasks::findAndAlignTable()
{
    std::cout << "JustinaTask::findAndAlignTable" << std::endl;

    float norm = 0.0;
    float angle = 0.0;
    std::vector<float> point;

    std::string table_loc = "";
    if(JustinaTasks::findTable(table_loc))
    {

        if (JustinaVision::findTable(point) )
        {
            std::cout << "The nearest point to plane is:  " << std::endl;
            std::cout << "p_x:  " << point[0] << std::endl;
            std::cout << "p_y:  " << point[1] << std::endl;
            std::cout << "p_z:  " << point[2] << std::endl;

            norm = sqrt(point[0]*point[0] + point[1]*point[1]);
            angle = atan(point[1]/point[0]);

            std::cout << "Correct angle: " << angle << std::endl;
            JustinaNavigation::moveDistAngle(0.0, angle, 3000);

            if(norm > 1.5)
            {
                JustinaNavigation::moveDist(norm - 0.5, 3000);
                std::cout << "Correct dist: " << norm -0.5 << std::endl;
            }
        }
        else
            std::cout << "I cannot find the nearest point... " << std::endl;

        JustinaHRI::insertAsyncSpeech("I am searching the line of the table", 500);
        JustinaHRI::asyncSpeech();    
        //JustinaHRI::waitAfterSay("I am searching the line of the table", 3000);
        //JustinaNavigation::moveDist(-0.15, 3000);
        for(int i = 0; i < 3; i++)
        {
            if( JustinaTasks::alignWithTable(0.35) )
            {
                std::cout << "I have found the table" << std::endl;
                JustinaHRI::insertAsyncSpeech("I found the table", 500);
                JustinaHRI::asyncSpeech();
                //JustinaHRI::waitAfterSay("I found the table", 3000);
                if(table_loc == "left")
                {
                    JustinaNavigation::moveLateral(-0.50, 2000);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
                    JustinaNavigation::moveDist(-0.05, 3000);
                }
                else if(table_loc == "rigth")
                {
                    JustinaNavigation::moveLateral(0.40, 2000);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
                    JustinaNavigation::moveDist(0.05, 3000);
                }

                return true;
            }
            else
            {

                if(table_loc == "left")
                {
                    JustinaNavigation::moveDistAngle(0.0, -M_PI_4, 3000);
                    JustinaNavigation::moveLateral(-0.20, 2000);
                    return true;
                }
                else if(table_loc == "rigth")
                {
                    JustinaNavigation::moveDistAngle(0.0, M_PI_4, 3000);
                    JustinaNavigation::moveLateral(0.20, 2000);
                    return true;
                }
            }
        }

        return false;

    }  
    else
    {
        return false;
    }
}

std::vector<vision_msgs::VisionFaceObject> JustinaTasks::recognizeAllFaces(float timeOut, bool &recognized)
{
    JustinaVision::startFaceRecognition();
    recognized = false;
    int previousSize = 20;
    int sameValue = 0;
    boost::posix_time::ptime curr;
    boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
    boost::posix_time::time_duration diff;
    std::vector<vision_msgs::VisionFaceObject> lastRecognizedFaces;

    do
    {
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        JustinaVision::facRecognize();
        JustinaVision::getLastRecognizedFaces(lastRecognizedFaces);
        ros::Duration(1.0).sleep();

        if(lastRecognizedFaces.size() == previousSize && lastRecognizedFaces.size() > 0)
            sameValue ++;
        if(sameValue == 3)
            recognized = true;
        else
        {
            previousSize = lastRecognizedFaces.size();
            recognized = false;
        }
        curr = boost::posix_time::second_clock::local_time();
        ros::spinOnce();
    }while(ros::ok() && (curr - prev).total_milliseconds()< timeOut && !recognized);

    std::cout << "recognized:" << recognized << std::endl;
    return lastRecognizedFaces;
}


bool JustinaTasks::findCrowd(int &men, int &women, int &sitting, int &standing, int &lying) {

    std::vector<int> facesDistances;
    std::stringstream ss;
    std::string personID = "";

    JustinaVision::startFaceRecognitionOld();

    JustinaManip::startHdGoTo(0, 0.0);
    JustinaManip::waitForHdGoalReached(5000);

    std::cout << "Find the crowd " << std::endl;

    ss << ", I am going to find the crowd";
    JustinaHRI::insertAsyncSpeech(ss.str(), 500);
    JustinaHRI::asyncSpeech();
    //JustinaHRI::waitAfterSay(ss.str(), 2000);

    Eigen::Vector3d centroidFace;
    int genderRecog;

    bool recog = turnAndRecognizeFace(personID, -1, NONE, -M_PI_4, M_PI_4 / 2.0, M_PI_4, 0, -M_PI_4, -M_PI_4, M_PI_2, 2 * M_PI, centroidFace, genderRecog);
    std::cout << "Centroid Face in coordinates of robot:" << centroidFace(0, 0)
        << "," << centroidFace(1, 0) << "," << centroidFace(2, 0) << ")";
    std::cout << std::endl;
    //personLocation.clear();
    JustinaVision::stopFaceRecognition();

    ss.str("");
    if (!recog) {
        std::cout << "I have not found the crowd "<< std::endl;
        ss << "I did not find a the crowd";
        JustinaHRI::insertAsyncSpeech(ss.str(), 500);
        JustinaHRI::asyncSpeech();
        //JustinaHRI::waitAfterSay(ss.str(), 2000);
        return false;
    }

    std::cout << "I found the crowd " << std::endl;
    //ss << person << ", I found you";
    ss << ", I find a person";
    JustinaHRI::insertAsyncSpeech(ss.str(), 500);
    JustinaHRI::insertAsyncSpeech("please do not move, I am going to count the number of people", 500);
    JustinaHRI::asyncSpeech();
    //JustinaHRI::waitAfterSay(ss.str(), 2000);

    float cx, cy, cz;
    cx = centroidFace(0, 0);
    cy = centroidFace(1, 0);
    cz = centroidFace(2, 0);
    JustinaTools::transformPoint("/base_link", cx, cy, cz, "/map", cx, cy, cz);
    tf::Vector3 worldFaceCentroid(cx, cy, cz);

    //JustinaHRI::waitAfterSay("I am getting close to you", 2000);
    //closeToGoalWithDistanceTHR(worldFaceCentroid.x(), worldFaceCentroid.y(), 1.0, 40000);
    float currx, curry, currtheta;

    JustinaNavigation::getRobotPose(currx, curry, currtheta);

    float thetaToGoal = atan2(worldFaceCentroid.y() - curry, worldFaceCentroid.x() - currx);
    if (thetaToGoal < 0.0f)
        thetaToGoal = 2 * M_PI + thetaToGoal;
    float theta = thetaToGoal - currtheta;
    std::cout << "JustinaTasks.->Turn in direction of robot:" << theta << std::endl;
    JustinaNavigation::moveDistAngle(0, theta, 2000);

    JustinaManip::startHdGoTo(0, 0.0);
    JustinaManip::waitForHdGoalReached(5000);

    int contChances = 0;
    std::vector<vision_msgs::VisionFaceObject> dFaces;
    recog = false;

    //JustinaHRI::say("please do not move, I am going to count the number of people");
    ros::Duration(1.5).sleep();
    while(!recog && contChances < 2){
        dFaces = recognizeAllFaces(10000,recog);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        JustinaVision::stopFaceRecognition();
        contChances++;
    }

    std::cout << "size of array is: " << dFaces.size() << std::endl;

    for(int i=0; i<dFaces.size(); i++)
    {
        if(dFaces[i].face_centroid.z < 0.8){
            lying++;
        }
        if(dFaces[i].face_centroid.z >= 0.8 && dFaces[i].face_centroid.z <1.20){
            sitting++;
        }
        if(dFaces[i].face_centroid.z >= 1.20){
            standing++;
        }
        if(dFaces[i].gender==0){
            women++;
        }
        if(dFaces[i].gender==1){
            men++;
        }
    }


    return true;
}

bool JustinaTasks::findWaving(float initPan, float incPan, float maxPan, float initTil, float incTil, float maxTil, int timeToFind, vision_msgs::VisionRect &rectWav){
    bool direction = false;
    bool find = false;
    float firstTil = initTil;
    for(float headPan = initPan; ros::ok() && headPan <= maxPan && !find; headPan+=incPan){
        float currTil;
        for (float headTil = initTil; ros::ok() && ((!direction && headTil >= maxTil) || (direction && headTil <= firstTil)) && !find; headTil+=incTil){
            currTil = headTil;
            JustinaManip::startHdGoTo(headPan, headTil);
            JustinaManip::waitForHdGoalReached(3000);
            //boost::this_thread::sleep(boost::posix_time::milliseconds(timeToFind));
            std::vector<vision_msgs::VisionRect> wavingDetect = JustinaVision::detectWaving();
            if(wavingDetect.size() > 0){
                //JustinaNavigation::moveDistAngle(0.0, -headPan, 3000);
                float minx;
                int indexMin;
                for(int i = 0; i < wavingDetect.size(); i++){
                    if(i == 0){
                        minx = wavingDetect[i].x;
                        indexMin = 0;
                    }
                    else if(wavingDetect[i].x < minx){
                        minx = wavingDetect[i].x;
                        indexMin = i;
                    }
                }
                rectWav = wavingDetect[indexMin];
                find = true;
            }
        }
        initTil = currTil;
        direction ^= true;
        incTil *= -1; 
    }
    return find;
}

bool JustinaTasks::alignWithWaving(vision_msgs::VisionRect rectWav){
    int width = 1920, height = 1080 ;
    float c_turn = 0.001;
    int maxOffsetx = 200, numAttempts = 5;

    int offsetx = int(width/2.0 - rectWav.x);
    int offsety = int(height/2.0 - rectWav.y);
    /*int offsetx = 99999;
      int offsety = 99999;*/
    std::cout << "JustinaTasks.->Trying to alignWithWaving offsetx:" << offsetx << std::endl;
    //JustinaManip::hdGoTo(0.0, 0.0, 4000);
    if(fabs(offsetx) <= maxOffsetx)
        return true;
    JustinaNavigation::moveDistAngle(0.0, c_turn * offsetx, 3000);

    for(int attempts = 0; attempts < numAttempts; attempts++){
        if(fabs(offsetx) <= maxOffsetx)
            return true;
        std::vector<vision_msgs::VisionRect> wavDetec = JustinaVision::detectWaving();
        if(wavDetec.size() > 0){
            float minx;
            int indexMin;
            for(int i = 0; i < wavDetec.size(); i++){
                if(i == 0){
                    minx = wavDetec[i].x;
                    indexMin = 0;
                }
                else if(wavDetec[i].x < minx){
                    minx = wavDetec[i].x;
                    indexMin = i;
                }
            }
            offsetx = int(width/2.0 -wavDetec[indexMin].x); 
            offsety = int(height/2.0 -wavDetec[indexMin].y);
            JustinaNavigation::moveDistAngle(0.0, c_turn * offsetx, 3000);
        }
        //boost::this_thread::sleep(boost::posix_time::milliseconds(500));
    }

    /*if(offsetx > 0)
      JustinaNavigation::moveDistAngle(0.0, c_turn, 3000);
      else if(offsetx < 0)
      JustinaNavigation::moveDistAngle(0.0, -c_turn, 3000);
      boost::this_thread::sleep(boost::posix_time::milliseconds(500));
      for(int attempts = 0; attempts < numAttempts; attempts++){
      std::cout << "JustinaTasks.->Attemp of new detect waving:" << attempts << std::endl;
      std::vector<vision_msgs::VisionRect> wavDetec = JustinaVision::detectWaving();
      if(wavDetec.size() > 0){
      std::cout << "JustinaTasks.->Detect new waving" << std::endl; 
      for(int i = 0; i < wavDetec.size(); i++){
      int newoffsetx = int(width/2.0 - wavDetec[i].x);
      int newoffsety = int(height/2.0 - wavDetec[i].y);
      if(fabs(newoffsetx) > maxOffsetx)
      continue;    
      if(fabs(newoffsetx <= maxOffsetx))
      return true;
      }
      }
      }*/
    return false;
}


bool JustinaTasks::openDoor(bool withLeftArm)
{
    std::cout << "JustinaTasks.->Trying to open the cupboard door" << std::endl;

    if(withLeftArm)
    {
        JustinaManip::laGoTo("navigation", 4000);
        JustinaManip::laGoTo("door_1", 4000);
        JustinaManip::laGoTo("door_2", 4000);

        JustinaNavigation::moveDist(-0.05, 2000);
        JustinaNavigation::moveDist(-0.05, 2000);
        JustinaNavigation::moveDist(-0.05, 2000);
        JustinaNavigation::moveDist(-0.05, 2000);

        JustinaNavigation::moveDistAngle(0.0, -M_PI_4, 3000);
        JustinaNavigation::moveDistAngle(0.0, M_PI_4, 3000);

        JustinaManip::laGoTo("navigation", 4000);
    }
    else
    {
        JustinaManip::raGoTo("navigation", 4000);
        JustinaManip::raGoTo("door_1", 4000);
        JustinaManip::raGoTo("door_2", 4000);

        JustinaNavigation::moveDist(-0.05, 2000);
        JustinaNavigation::moveDist(-0.05, 2000);
        JustinaNavigation::moveDist(-0.05, 2000);
        JustinaNavigation::moveDist(-0.05, 2000);

        JustinaNavigation::moveDistAngle(0.0, M_PI_4, 3000);
        JustinaNavigation::moveDistAngle(0.0, -M_PI_4, 3000);

        JustinaManip::raGoTo("navigation", 4000);
    }

    return true;
}


bool JustinaTasks::cubeSortByY (vision_msgs::Cube &i,vision_msgs::Cube &j) { 
    return i.cube_centroid.y < j.cube_centroid.y; 
}

bool JustinaTasks::cubeSortByZ (vision_msgs::Cube &i,vision_msgs::Cube &j) { 
    return i.cube_centroid.z < j.cube_centroid.z; 
}


bool JustinaTasks::sortCubes(vision_msgs::CubesSegmented cubes, std::vector<vision_msgs::CubesSegmented> &Stacks)
{
    vision_msgs::CubesSegmented StackCube1;
    vision_msgs::CubesSegmented StackCube2;
    float dif= 0.0;
    //std::vector<vision_msgs::CubesSegmented> Stacks;


    if(cubes.recog_cubes.size() > 0) {
        //std::cout << "cube size = " << cubes.recog_cubes.size() << std::endl;
        std::sort (cubes.recog_cubes.begin(), cubes.recog_cubes.end(), cubeSortByY);
    }
    else
        return false;
    /*for(int j=0; j<cubes.recog_cubes.size(); j++)
      {
      std::cout<< "color: " << cubes.recog_cubes[j].color<< std::endl;
      }*/

    StackCube1.recog_cubes.push_back(cubes.recog_cubes[0]);

    for(int i=1; i<cubes.recog_cubes.size();i++)
    {
        vision_msgs::Cube cube1=cubes.recog_cubes[0];
        vision_msgs::Cube cube2=cubes.recog_cubes[i];
        /*std::cout<< "cube0 y: " << cube1.cube_centroid.y<< std::endl;
          std::cout<< "cubei y: " << cube2.cube_centroid.y<< std::endl;*/

        dif = fabs(fabs(cube1.cube_centroid.y)-fabs(cube2.cube_centroid.y));
        std::cout<<"dif: "<<dif<<std::endl;
        if(dif<0.03 && ((cube1.cube_centroid.y<0 &&cube2.cube_centroid.y<0)
                    ||(cube1.cube_centroid.y>0 &&cube2.cube_centroid.y>0)))
            StackCube1.recog_cubes.push_back(cube2);

        else
            StackCube2.recog_cubes.push_back(cube2); 
    }

    if(StackCube1.recog_cubes.size() > 0)
    { 
        std::sort (StackCube1.recog_cubes.begin(), StackCube1.recog_cubes.end(), cubeSortByZ);
        Stacks[0]=StackCube1;
    }

    if(StackCube2.recog_cubes.size() > 0) 
    {
        std::sort (StackCube2.recog_cubes.begin(), StackCube2.recog_cubes.end(), cubeSortByZ);
        Stacks[1]=StackCube2;
    }

    /*std::cout<< "stackcube1 size: " << StackCube1.recog_cubes.size()<< std::endl;
      std::cout<< "stackcube2 size: " << StackCube2.recog_cubes.size()<< std::endl;
      std::cout<< "stacks size: " << Stacks.size()<< std::endl;
      std::cout<< "stacks at 0: "<<Stacks[0].recog_cubes.size()<<std::endl;*/


    return true;
}

bool JustinaTasks::graspBlock(float x, float y, float z, bool withLeftArm,
        std::string idBlock, bool usingTorse) {
    std::cout
        << "JustinaTasks.->Moving to a good-pose for grasping objects with ";
    if (withLeftArm)
        std::cout << "left arm" << std::endl;
    else
        std::cout << "right arm" << std::endl;

    bool objectInHand = false;
    float idealX = 0.475;
    float idealY = withLeftArm ? 0.234 : -0.235; //It is the distance from the center of the robot, to the center of the arm
    float idealZ = 0.52; //It is the ideal height for taking an object when torso is at zero height.

    float torsoSpine, torsoWaist, torsoShoulders;
    JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist,
            torsoShoulders);
    std::cout << "JustinaTasks.->torsoSpine:" << torsoSpine << std::endl;

    float objToGraspX = x;
    float objToGraspY = y;
    float objToGraspZ = z;
    float movTorsoFromCurrPos;
    std::cout << "JustinaTasks.->ObjToGrasp: " << "  " << objToGraspX << "  "
        << objToGraspY << "  " << objToGraspZ << std::endl;
    float movFrontal = -(idealX - objToGraspX);
    float movLateral = -(idealY - objToGraspY);
    float movVertical = objToGraspZ - idealZ - torsoSpine;
    float goalTorso = torsoSpine + movVertical;
    std::cout << "JustinaTasks.->goalTorso:" << goalTorso << std::endl;
    int waitTime;
    if (goalTorso < 0.2)
        goalTorso = 0.2;
    if (goalTorso > 0.5)
        goalTorso = 0.5;

    movTorsoFromCurrPos = goalTorso - torsoSpine;
    waitTime = (int) (30000 * fabs(movTorsoFromCurrPos) / 0.3 + 3000);
    std::cout << "JustinaTasks.->movTorsoFromCurrPos:" << movTorsoFromCurrPos << std::endl;
    std::cout << "JustinaTasks.->waitTime:" << waitTime << std::endl;

    std::cout << "JustinaTasks.->Adjusting with frontal=" << movFrontal
        << " lateral=" << movLateral << " and vertical=" << movVertical
        << std::endl;
    float lastRobotX, lastRobotY, lastRobotTheta;
    JustinaNavigation::getRobotPose(lastRobotX, lastRobotY, lastRobotTheta);
    if(usingTorse)
        JustinaManip::startTorsoGoTo(goalTorso, 0, 0);
    JustinaNavigation::moveLateral(movLateral, 6000);
    JustinaNavigation::moveDist(movFrontal, 6000);
    if(usingTorse)
        JustinaManip::waitForTorsoGoalReached(waitTime);

    bool found = false;
    vision_msgs::CubesSegmented cubes;
    vision_msgs::Cube cube_aux;
    std::vector<vision_msgs::VisionObject> recognizedObjects;
    int indexFound = 0;
    if (idBlock.compare("") != 0) {
        JustinaManip::startHdGoTo(0, -0.9);
        JustinaManip::waitForHdGoalReached(5000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        cube_aux.color = idBlock;
        cubes.recog_cubes.push_back(cube_aux);
        found = JustinaVision::getCubesSeg(cubes);
        std::cout << "GET CUBES: " << found << std::endl;
    }

    if (found) {
        std::cout << "The object was found again, update the new coordinates."
            << std::endl;
        objToGraspX = cubes.recog_cubes.at(0).cube_centroid.x;
        objToGraspY = cubes.recog_cubes.at(0).cube_centroid.y;
    } else if (!found && idBlock.compare("") == 0) {
        std::cout
            << "The object was not found again, update new coordinates with the motion of robot."
            << std::endl;
        float robotX, robotY, robotTheta;
        //JustinaNavigation::getRobotPose(robotX, robotY, robotTheta);
        JustinaNavigation::getRobotPose(robotX, robotY, robotTheta);
        //Adjust the object position according to the new robot pose
        //I don't request again the object position due to the possibility of not recognizing it again
        float dxa = (robotX - lastRobotX);
        float dya = (robotY - lastRobotY);
        float dxr = dxa * cos(robotTheta) + dya * sin(robotTheta);
        float dyr = -dxa * sin(robotTheta) + dya * cos(robotTheta);

        objToGraspX -= dxr;
        objToGraspY -= dyr;
        std::cout << "lastRobotX:" << lastRobotX << ",lastRobotY:" << lastRobotY
            << ",lastRobotTheta:" << lastRobotTheta << std::endl;
        std::cout << "robotX:" << robotX << ",robotY:" << robotY
            << ",robotTheta:" << robotTheta << std::endl;
        std::cout << "objToGraspX:" << objToGraspX << ",objToGraspY:"
            << objToGraspY << ",objToGraspZ:" << objToGraspZ << std::endl;
    } else if (!found && idBlock.compare("") != 0) {
        JustinaNavigation::moveDist(-0.2, 3000);
        return false;
    }

    //The position it is adjusted and converted to coords wrt to the corresponding arm
    std::string destFrame = withLeftArm ? "left_arm_link0" : "right_arm_link0";
    if (!JustinaTools::transformPoint("base_link", objToGraspX, objToGraspY,
                objToGraspZ, destFrame, objToGraspX, objToGraspY, objToGraspZ)) {
        std::cout << "JustinaTasks.->Cannot transform point. " << std::endl;
        return false;
    }
    std::cout << "JustinaTasks.->Moving ";
    if (withLeftArm)
        std::cout << "left arm";
    else
        std::cout << "right arm";
    std::cout << " to " << objToGraspX << "  " << objToGraspY << "  "
        << objToGraspZ << std::endl;

    if (withLeftArm) {

        if(!JustinaManip::isLaInPredefPos("navigation"))
            JustinaManip::laGoTo("navigation", 7000);
        else
            std::cout << "JustinaTasks.->The left arm already has in the navigation pose" << std::endl;

        JustinaManip::startLaOpenGripper(0.8);
        //Move the manipulator to objectOB

        JustinaManip::laGoToCartesian(objToGraspX - 0.04, objToGraspY - 0.25,
                objToGraspZ - 0.04, 3000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));

        JustinaManip::laGoToCartesian(objToGraspX - 0.04, objToGraspY - 0.15,
                objToGraspZ - 0.04, 3000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));

        JustinaManip::laGoToCartesian(objToGraspX + 0.035, objToGraspY - 0.10,
                objToGraspZ - 0.06, 2000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));

        JustinaNavigation::moveDist(0.08, 3000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1500));


        JustinaManip::startLaCloseGripper(0.5);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
        ros::spinOnce();
        if (JustinaManip::objOnLeftHand()) {
            if(usingTorse){
                JustinaManip::startTorsoGoTo(goalTorso + 0.03, 0, 0);
                JustinaManip::waitForTorsoGoalReached(5000);
            }else
                JustinaManip::laGoToCartesian(objToGraspX - 0.13, objToGraspY + 0.04, objToGraspZ, 0, 0, 1.5708, 0, 5000);
            JustinaNavigation::moveDist(-0.35, 3000);
            JustinaManip::laGoTo("navigation", 5000);
            std::cout
                << "The object was grasp with the left arm in the first test"
                << std::endl;
            return true;
        }
        JustinaNavigation::moveDist(-0.2, 3000);
        if(!JustinaManip::isLaInPredefPos("navigation"))
            JustinaManip::laGoTo("navigation", 5000);
        else
            std::cout << "JustinaTasks.->The left arm already has in the navigation pose" << std::endl;
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        if (JustinaManip::objOnLeftHand()) {
            std::cout
                << "The object was grasp with the left arm in the second test"
                << std::endl;
            return true;
        }
        std::cout << "The object was not grasp with the left arm" << std::endl;
        return false;
    } else {
        if(!JustinaManip::isRaInPredefPos("navigation"))
            JustinaManip::raGoTo("navigation", 10000);
        else
            std::cout << "JustinaTasks.->The right arm already has in the navigation pose" << std::endl;

        JustinaManip::startRaOpenGripper(0.8);
        //Move the manipulator to object


        JustinaManip::raGoToCartesian(objToGraspX - 0.06, objToGraspY - 0.25,
                objToGraspZ, 3000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));

        JustinaManip::raGoToCartesian(objToGraspX - 0.06, objToGraspY - 0.15,
                objToGraspZ, 3000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));

        JustinaManip::raGoToCartesian(objToGraspX + 0.035, objToGraspY - 0.05,
                objToGraspZ, 3000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));

        JustinaNavigation::moveDist(0.08, 3000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1500));

        JustinaManip::startRaCloseGripper(0.5);
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
        ros::spinOnce();
        if (JustinaManip::objOnRightHand()) {
            if(usingTorse){
                JustinaManip::startTorsoGoTo(goalTorso + 0.03, 0, 0);
                JustinaManip::waitForTorsoGoalReached(6000);
            }else
                JustinaManip::raGoToCartesian(objToGraspX - 0.1, objToGraspY + 0.04, objToGraspZ, 0, 0, 1.5708, 0, 5000);
            JustinaNavigation::moveDist(-0.35, 3000);
            JustinaManip::raGoTo("navigation", 5000);
            std::cout
                << "The object was grasp with the right arm in the first test"
                << std::endl;
            return true;
        }
        JustinaNavigation::moveDist(-0.2, 3000);
        if(!JustinaManip::isRaInPredefPos("navigation"))
            JustinaManip::raGoTo("navigation", 5000);
        else
            std::cout << "JustinaTasks.->The right arm already has in the navigation pose" << std::endl;
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        if (JustinaManip::objOnRightHand()) {
            std::cout
                << "The object was grasp with the right arm in the second test"
                << std::endl;
            return true;
        }
        std::cout << "The object was not grasp with the right arm" << std::endl;
        return false;
    }
    return false;
}

bool JustinaTasks::graspBlockFeedback(float x, float y, float z, bool withLeftArm,
        std::string idBlock, bool usingTorse) {
    std::cout
        << "JustinaTasks.->Moving to a good-pose for grasping objects with ";
    if (withLeftArm)
        std::cout << "left arm" << std::endl;
    else
        std::cout << "right arm" << std::endl;

    std::stringstream ss;
    ss.str("");
    
    if(idBlock == "")
        ss << "I am going to take a  Unknown  object.";
    else
        ss << "I am going to take the " << idBlock << " block";

    //JustinaHRI::waitAfterSay(ss.str(), 2000);
    JustinaHRI::insertAsyncSpeech(ss.str(), 500);
    JustinaHRI::asyncSpeech();

    bool objectInHand = false;
    float idealX = 0.475;
    float idealY = withLeftArm ? 0.225 : -0.255; //It is the distance from the center of the robot, to the center of the arm
    float idealZ = 0.62; //It is the ideal height for taking an object when torso is at zero height.

    float torsoSpine, torsoWaist, torsoShoulders;
    JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist,
            torsoShoulders);
    std::cout << "JustinaTasks.->torsoSpine:" << torsoSpine << std::endl;

    float objToGraspX = x;
    float objToGraspY = y;
    float objToGraspZ = z;
    float movTorsoFromCurrPos;
    std::cout << "JustinaTasks.->ObjToGrasp: " << "  " << objToGraspX << "  "
        << objToGraspY << "  " << objToGraspZ << std::endl;
    float movFrontal = -(idealX - objToGraspX);
    float movLateral = -(idealY - objToGraspY);
    float movVertical = objToGraspZ - idealZ - torsoSpine;
    float goalTorso = torsoSpine + movVertical;
    std::cout << "JustinaTasks.->goalTorso:" << goalTorso << std::endl;
    int waitTime;
    if (goalTorso < 0.2)
        goalTorso = 0.2;
    if (goalTorso > 0.5)
        goalTorso = 0.5;

    movTorsoFromCurrPos = goalTorso - torsoSpine;
    waitTime = (int) (30000 * fabs(movTorsoFromCurrPos) / 0.3 + 3000);
    std::cout << "JustinaTasks.->movTorsoFromCurrPos:" << movTorsoFromCurrPos << std::endl;
    std::cout << "JustinaTasks.->goalTorso:" << goalTorso << std::endl;
    std::cout << "JustinaTasks.->waitTime:" << waitTime << std::endl;

    std::cout << "JustinaTasks.->Adjusting with frontal=" << movFrontal
        << " lateral=" << movLateral << " and vertical=" << movVertical
        << std::endl;
    float lastRobotX, lastRobotY, lastRobotTheta;
    JustinaNavigation::getRobotPose(lastRobotX, lastRobotY, lastRobotTheta);
    if(usingTorse)
        JustinaManip::startTorsoGoTo(goalTorso, 0, 0);
    JustinaNavigation::moveLateral(movLateral, 6000);
    JustinaNavigation::moveDist(movFrontal, 6000);
    if(usingTorse)
        JustinaManip::waitForTorsoGoalReached(waitTime);

    bool found = false;
    vision_msgs::CubesSegmented cubes;
    vision_msgs::Cube cube_aux;
    std::vector<vision_msgs::VisionObject> recognizedObjects;
    int indexFound = 0;
    if (idBlock.compare("") != 0) {
        JustinaManip::startHdGoTo(0, -0.9);
        JustinaManip::waitForHdGoalReached(5000);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        cube_aux.color = idBlock;
        cubes.recog_cubes.push_back(cube_aux);
        found = JustinaVision::getCubesSeg(cubes);
        std::cout << "GET CUBES: " << found << std::endl;
    }

    if (found && cubes.recog_cubes[0].detected_cube) {
        std::cout << "The object was found again, update the new coordinates."
            << std::endl;
        objToGraspX = cubes.recog_cubes.at(0).cube_centroid.x;
        objToGraspY = cubes.recog_cubes.at(0).cube_centroid.y;
        //objToGraspZ = (cubes.recog_cubes.at(0).cube_centroid.z + cubes.recog_cubes.at(0).maxPoint.z) / 2.0f;
        //objToGraspZ = cubes.recog_cubes.at(0).cube_centroid.z;
        objToGraspZ = cubes.recog_cubes.at(0).maxPoint.z;
        std::cout << "MaxPoint en z:" << objToGraspZ << std::endl;
    } else if (!found && idBlock.compare("") == 0) {
        std::cout
            << "The object was not found again, update new coordinates with the motion of robot."
            << std::endl;
        float robotX, robotY, robotTheta;
        //JustinaNavigation::getRobotPose(robotX, robotY, robotTheta);
        JustinaNavigation::getRobotPose(robotX, robotY, robotTheta);
        //Adjust the object position according to the new robot pose
        //I don't request again the object position due to the possibility of not recognizing it again
        float dxa = (robotX - lastRobotX);
        float dya = (robotY - lastRobotY);
        float dxr = dxa * cos(robotTheta) + dya * sin(robotTheta);
        float dyr = -dxa * sin(robotTheta) + dya * cos(robotTheta);

        objToGraspX -= dxr;
        objToGraspY -= dyr;
        std::cout << "lastRobotX:" << lastRobotX << ",lastRobotY:" << lastRobotY
            << ",lastRobotTheta:" << lastRobotTheta << std::endl;
        std::cout << "robotX:" << robotX << ",robotY:" << robotY
            << ",robotTheta:" << robotTheta << std::endl;
        std::cout << "objToGraspX:" << objToGraspX << ",objToGraspY:"
            << objToGraspY << ",objToGraspZ:" << objToGraspZ << std::endl;
    } else if (!found && idBlock.compare("") != 0 || !cubes.recog_cubes[0].detected_cube) {
        JustinaNavigation::moveDist(-0.2, 3000);
        return false;
    }

    //The position it is adjusted and converted to coords wrt to the corresponding arm
    std::string destFrame = withLeftArm ? "left_arm_link0" : "right_arm_link0";
    if (!JustinaTools::transformPoint("base_link", objToGraspX, objToGraspY,
                objToGraspZ, destFrame, objToGraspX, objToGraspY, objToGraspZ)) {
        std::cout << "JustinaTasks.->Cannot transform point. " << std::endl;
        return false;
    }
    std::cout << "JustinaTasks.->Moving ";
    if (withLeftArm)
        std::cout << "left arm";
    else
        std::cout << "right arm";
    std::cout << " to " << objToGraspX << "  " << objToGraspY << "  "
        << objToGraspZ << std::endl;

    if (withLeftArm) {

        if(!JustinaManip::isLaInPredefPos("navigation"))
            JustinaManip::laGoTo("navigation", 7000);
        else
            std::cout << "JustinaTasks.->The left arm already has in the navigation pose" << std::endl;

        JustinaManip::startLaOpenGripper(0.8);
        //Move the manipulator to objectOB

        JustinaManip::laGoToCartesianTraj(objToGraspX, objToGraspY, objToGraspZ, 20000);
        JustinaManip::laStopGoToCartesian();
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        ros::spinOnce();
        JustinaManip::startLaCloseGripper(0.5);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
        for(int i = 0; i < 3; i++){
            if (JustinaManip::objOnLeftHand()) {
                if(usingTorse){
                    JustinaManip::startTorsoGoTo(goalTorso + 0.05, 0, 0);
                    JustinaManip::waitForTorsoGoalReached(8000);
                }else
                    JustinaManip::laGoToCartesian(objToGraspX - 0.13, objToGraspY + 0.04, objToGraspZ, 0, 0, 1.5708, 0, 5000);
                JustinaNavigation::moveDist(-0.35, 3000);
                JustinaManip::laGoTo("navigation", 5000);
                std::cout
                    << "The object was grasp with the left arm in the first test"
                    << std::endl;
                return true;
            }
            boost::this_thread::sleep(boost::posix_time::milliseconds(500));
            ros::spinOnce();
        }
        JustinaNavigation::moveDist(-0.2, 3000);
        if(!JustinaManip::isLaInPredefPos("navigation"))
            JustinaManip::laGoTo("navigation", 5000);
        else
            std::cout << "JustinaTasks.->The left arm already has in the navigation pose" << std::endl;
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        if (JustinaManip::objOnLeftHand()) {
            std::cout
                << "The object was grasp with the left arm in the second test"
                << std::endl;
            return true;
        }
        std::cout << "The object was not grasp with the left arm" << std::endl;
        return false;
    } else {
        if(!JustinaManip::isRaInPredefPos("navigation"))
            JustinaManip::raGoTo("navigation", 10000);
        else
            std::cout << "JustinaTasks.->The right arm already has in the navigation pose" << std::endl;

        JustinaManip::startRaOpenGripper(0.8);
        //Move the manipulator to object

        JustinaManip::raGoToCartesianTraj(objToGraspX, objToGraspY, objToGraspZ, 20000);
        JustinaManip::raStopGoToCartesian();
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        ros::spinOnce();
        JustinaManip::startRaCloseGripper(0.5);
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
        for(int i = 0; i < 3; i++){
            if (JustinaManip::objOnRightHand()) {
                if(usingTorse){
                    JustinaManip::startTorsoGoTo(goalTorso + 0.05, 0, 0);
                    JustinaManip::waitForTorsoGoalReached(8000);
                }else
                    JustinaManip::raGoToCartesian(objToGraspX - 0.1, objToGraspY + 0.04, objToGraspZ, 0, 0, 1.5708, 0, 5000);
                JustinaNavigation::moveDist(-0.35, 3000);
                JustinaManip::raGoTo("navigation", 5000);
                std::cout
                    << "The object was grasp with the right arm in the first test"
                    << std::endl;
                return true;
            }
            ros::spinOnce();
            boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        }
        JustinaNavigation::moveDist(-0.2, 3000);
        if(!JustinaManip::isRaInPredefPos("navigation"))
            JustinaManip::raGoTo("navigation", 5000);
        else
            std::cout << "JustinaTasks.->The right arm already has in the navigation pose" << std::endl;
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        if (JustinaManip::objOnRightHand()) {
            std::cout
                << "The object was grasp with the right arm in the second test"
                << std::endl;
            return true;
        }
        std::cout << "The object was not grasp with the right arm" << std::endl;
        return false;
    }
    return false;
}

bool JustinaTasks::placeBlockOnBlock(float h, bool withLeftArm,  std::string idBlock, bool usingTorse, float X, float Y, float Z) {
    std::cout << "JustinaTasks::placeBlockOnBlock..." << std::endl;
    float x, y, z;
    if(!JustinaTasks::alignWithTable(0.32))
        JustinaTasks::alignWithTable(0.32);

    bool finishMotion = false;
    float moves[3] = {0.3, -0.6, 0.0};
    for(int i = 0; i < sizeof(moves) / sizeof(*moves) && !finishMotion && idBlock.compare("simul") != 0 ; i++){
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        vision_msgs::CubesSegmented cubes;
        vision_msgs::Cube cube_aux;
        cube_aux.color = idBlock;
        cubes.recog_cubes.push_back(cube_aux);

        bool fcubes = JustinaVision::getCubesSeg(cubes);

        if(fcubes){
            if(cubes.recog_cubes[0].detected_cube){
                finishMotion = true;
                x = cubes.recog_cubes[0].cube_centroid.x;
                y = cubes.recog_cubes[0].cube_centroid.y;
                z = cubes.recog_cubes[0].maxPoint.z;
            }
            else
                JustinaNavigation::moveLateral(moves[i], 4000);
        }
    }

    tf::StampedTransform transform;
    tf::TransformListener* tf_listener= new tf::TransformListener();
    tf::Vector3 p(X, Y, Z);

    if(idBlock.compare("simul") == 0){
        tf_listener->waitForTransform("base_link", "map", ros::Time(0), ros::Duration(10.0));
        tf_listener->lookupTransform("base_link", "map", ros::Time(0), transform);
        p = transform * p;

        x = p.getX();
        y = p.getY();
        z = p.getZ();
        std::cout << "Coordenadas Simul: " << x << " " << y << " " << z << std::endl;
    }


    std::cout << "JustinaTasks.->Moving to a good-pose for grasping objects with ";
    if (withLeftArm)
        std::cout << "left arm" << std::endl;
    else
        std::cout << "right arm" << std::endl;

    float idealX = 0.475;
    float idealY = withLeftArm ? 0.225 : -0.225; //It is the distance from the center of the robot, to the center of the arm
    float idealZ = 0.52; //It is the ideal height for taking an object when torso is at zero height.

    float torsoSpine, torsoWaist, torsoShoulders;
    JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist,
            torsoShoulders);
    std::cout << "JustinaTasks.->torsoSpine:" << torsoSpine << std::endl;

    float toPlaceCubeX = x;
    float toPlaceCubeY = y;
    float toPlaceCubeZ = z;
    float movTorsoFromCurrPos;
    std::cout << "JustinaTasks.->toPlaceCube: " << "  " << toPlaceCubeX << ", " << toPlaceCubeY << ", " << toPlaceCubeZ << std::endl;
    float movFrontal = -(idealX - toPlaceCubeX);
    float movLateral = -(idealY - toPlaceCubeY);
    float movVertical = toPlaceCubeZ - idealZ - torsoSpine;
    float goalTorso = torsoSpine + movVertical;
    std::cout << "JustinaTasks.->goalTorso:" << goalTorso << std::endl;
    int waitTime;
    if (goalTorso < 0.2)
        goalTorso = 0.2;
    if (goalTorso > 0.5)
        goalTorso = 0.5;

    movTorsoFromCurrPos = goalTorso - torsoSpine;
    waitTime = (int) (30000 * fabs(movTorsoFromCurrPos) / 0.3 + 3000);
    std::cout << "JustinaTasks.->movTorsoFromCurrPos:" << movTorsoFromCurrPos << std::endl;
    std::cout << "JustinaTasks.->waitTime:" << waitTime << std::endl;

    std::cout << "JustinaTasks.->Adjusting with frontal=" << movFrontal << " lateral=" << movLateral << " and vertical=" << movVertical << std::endl;

    float lastRobotX, lastRobotY, lastRobotTheta;
    JustinaNavigation::getRobotPose(lastRobotX, lastRobotY, lastRobotTheta);
    if(usingTorse)
        JustinaManip::startTorsoGoTo(goalTorso, 0, 0);
    JustinaNavigation::moveLateral(movLateral, 6000);
    JustinaNavigation::moveDist(movFrontal, 6000);
    if(usingTorse)
        JustinaManip::waitForTorsoGoalReached(waitTime);

    vision_msgs::CubesSegmented cubes;
    JustinaManip::startHdGoTo(0, -0.9);
    JustinaManip::waitForHdGoalReached(5000);
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

    float armGoalX, armGoalY, armGoalZ;
    
    if(idBlock.compare("simul") != 0){
        vision_msgs::Cube cube_aux;
        cube_aux.color = idBlock;
        cubes.recog_cubes.push_back(cube_aux);
        bool found = JustinaVision::getCubesSeg(cubes);
        if(!found)
            return false;
        else
            if(!cubes.recog_cubes[0].detected_cube)
                return false;

        //float armGoalX = cubes.recog_cubes[0].cube_centroid.x;
        armGoalX = cubes.recog_cubes[0].minPoint.x;
        armGoalY = cubes.recog_cubes[0].cube_centroid.y;
        armGoalZ = cubes.recog_cubes[0].maxPoint.z + h;
    }
    else{
        tf_listener->waitForTransform("base_link", "map", ros::Time(0), ros::Duration(10.0));
        tf_listener->lookupTransform("base_link", "map", ros::Time(0), transform);
        tf::Vector3 pos(X, Y, Z);
        pos = transform * pos;
        armGoalX = pos.getX();
        armGoalY = pos.getY();
        armGoalZ = pos.getZ() + h;
    }

    //The position it is adjusted and converted to coords wrt to the corresponding arm
    std::string destFrame = withLeftArm ? "left_arm_link0" : "right_arm_link0";
    if (!JustinaTools::transformPoint("base_link", armGoalX, armGoalY, armGoalZ, destFrame, armGoalX, armGoalY, armGoalZ)) {
        std::cout << "JustinaTasks.->Cannot transform point. " << std::endl;
        return false;
    }
    std::cout << "JustinaTasks.->Moving ";
    if (withLeftArm)
        std::cout << "left arm";
    else
        std::cout << "right arm to " << armGoalX << "," << armGoalY << "," << armGoalZ << std::endl;

    if (withLeftArm) {

        if(!JustinaManip::isLaInPredefPos("navigation"))
            JustinaManip::laGoTo("navigation", 7000);
        else
            std::cout << "JustinaTasks.->The left arm already has in the navigation pose" << std::endl;
        // TODO This is for the subrutine to place cube on cube
        //JustinaManip::laGoToCartesianFeedback(armGoalX, armGoalY, armGoalZ, 20000);
        JustinaManip::laGoToCartesianTraj(armGoalX, armGoalY, armGoalZ, 20000);
        JustinaManip::laStopGoToCartesian();
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        JustinaManip::startLaOpenGripper(0.7);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
        ros::spinOnce();
        JustinaNavigation::moveDist(-0.2, 5000);
        JustinaManip::laGoTo("navigation", 5000);
        JustinaManip::startLaOpenGripper(0.0);
        JustinaManip::startHdGoTo(0.0, 0.0);
    } else {
        if(!JustinaManip::isRaInPredefPos("navigation"))
            JustinaManip::raGoTo("navigation", 10000);
        else
            std::cout << "JustinaTasks.->The right arm already has in the navigation pose" << std::endl;
        // TODO This is for the subrutine to place cube on cube
        //JustinaManip::raGoToCartesianFeedback(armGoalX, armGoalY, armGoalZ, 20000);
        JustinaManip::raGoToCartesianTraj(armGoalX, armGoalY, armGoalZ, 20000);
        JustinaManip::raStopGoToCartesian();
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        JustinaManip::startRaOpenGripper(0.7);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
        ros::spinOnce();
        JustinaNavigation::moveDist(-0.2, 5000);
        JustinaManip::raGoTo("navigation", 5000);
        JustinaManip::startRaOpenGripper(0.0);
        JustinaManip::startHdGoTo(0.0, 0.0);
    }
    JustinaNavigation::moveLateral(-movLateral, 6000);
    return true;
}

bool JustinaTasks::faceSort(vision_msgs::VisionFaceObject &i, vision_msgs::VisionFaceObject &j)
{
    return i.face_centroid.x < j.face_centroid.x;
}

bool JustinaTasks::setRoi(vision_msgs::VisionFaceObjects faces)
{
    //cv::Scalar frontLB, backRT;
    std::string configFileName = "configFile.xml";
    std::string configDir  = "";
    std::string configPath ; 
    configDir = ros::package::getPath("roi_tracker") + "/ConfigDir";
    if( !boost::filesystem::exists( configDir ) ) 
        boost::filesystem::create_directory( configDir ); 
    configPath = configDir + "/" +  configFileName;    
    
    bool Debug = true; 
    int noBins = 18; 
    float overPercWidth  = 0.750;
    float overPercHeight = 0.750;
    float overNoRectsWidth  = 4;
    float overNoRectsHeight = 4;  

    float scaleFactor = 0.20; 
    cv::Size scaleMax, scaleMin; 
    cv::Scalar frontLB, backRT;
    float scaleSteps = 3.00; 
     
    
    float matchThreshold = 0.85;

    scaleMax = cv::Size(640,480); 
    scaleMin = cv::Size(64,128);

    frontLB = cv::Scalar( 0.50, -0.30, 0.30 );
    backRT = cv::Scalar ( 2.00,  0.30, 2.00 );

    //vision_msgs::VisionFaceObjects faces;

    //faces = JustinaVision::getFaces("");
    std::sort(faces.recog_faces.begin(), faces.recog_faces.end(), faceSort);

    frontLB = cv::Scalar(faces.recog_faces[0].face_centroid.x - 0.2, 
                                            faces.recog_faces[0].face_centroid.y - 0.1, 
                                            faces.recog_faces[0].face_centroid.z - 0.4);

    std::cout << "frontLeftBot>> : " << faces.recog_faces[0].face_centroid.x - 0.2 << std::endl;

    backRT = cv::Scalar(faces.recog_faces[0].face_centroid.x + 0.2, 
                                            faces.recog_faces[0].face_centroid.y + 0.1, 
                                            faces.recog_faces[0].face_centroid.z - 0.2);

    std::cout << "backRightTop>> : " << faces.recog_faces[0].face_centroid.x + 0.2 << std::endl;


    try{
        // Getting configFile
        cv::FileStorage fs; 
        
        if(fs.open( configPath, fs.WRITE ) )
        {
            std::cout << ">> RoiTracker. Writing configFile:" << configFileName << ".Creating it." << std::endl; 

            fs << "Debug" << ( Debug ? 1 : 0 ); 
            fs << "noBins" << noBins; 

            fs << "frontLeftBot" << frontLB; 
            fs << "backRightTop" << backRT; 

            fs << "overPercWidth" << overPercWidth;
            fs << "overPercHeight" << overPercHeight;
            fs << "overNoRectsWidth" << overNoRectsWidth;
            fs << "overNoRectsHeight" << overNoRectsHeight ;  

            fs << "scaleFactor" << scaleFactor; 
            fs << "scaleSteps" << scaleSteps; 
            fs << "scaleMax" << scaleMax;
            fs << "scaleMin" << scaleMin; 

            fs << "matchThreshold" << matchThreshold;

            fs.release(); 
        }
        
    }
    catch(...)
    {
        std::cout << "Exception while openning file. Using default params..." << std::endl;
        return false; 
    }

    return true;
}
