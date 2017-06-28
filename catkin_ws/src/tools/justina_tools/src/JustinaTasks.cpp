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

    float idealX = 0.4;
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
    if (goalTorso < 0)
        goalTorso = 0;
    if (goalTorso > 0.45)
        goalTorso = 0.45;

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
}

bool JustinaTasks::graspObject(float x, float y, float z, bool withLeftArm,
        std::string idObject) {
    std::cout
        << "JustinaTasks.->Moving to a good-pose for grasping objects with ";
    if (withLeftArm)
        std::cout << "left arm" << std::endl;
    else
        std::cout << "right arm" << std::endl;

    bool objectInHand = false;
    float idealX = 0.425;
    float idealY = withLeftArm ? 0.234 : -0.235; //It is the distance from the center of the robot, to the center of the arm
    float idealZ = 0.618; //It is the ideal height for taking an object when torso is at zero height.

    float torsoSpine, torsoWaist, torsoShoulders;
    JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist,
            torsoShoulders);
    idealZ += torsoSpine;

    float objToGraspX = x;
    float objToGraspY = y;
    float objToGraspZ = z;
    std::cout << "JustinaTasks.->ObjToGrasp: " << "  " << objToGraspX << "  "
        << objToGraspY << "  " << objToGraspZ << std::endl;
    float movFrontal = -(idealX - objToGraspX);
    float movLateral = -(idealY - objToGraspY);
    float movVertical = -(idealZ - objToGraspZ);
    float goalTorso = torsoSpine + movVertical;
    if (goalTorso < 0)
        goalTorso = 0;
    if (goalTorso > 0.45)
        goalTorso = 0.45;

    std::cout << "JustinaTasks.->Adjusting with frontal=" << movFrontal
        << " lateral=" << movLateral << " and vertical=" << movVertical
        << std::endl;
    float lastRobotX, lastRobotY, lastRobotTheta;
    //JustinaNavigation::getRobotPose(lastRobotX, lastRobotY, lastRobotTheta);
    JustinaNavigation::getRobotPose(lastRobotX, lastRobotY, lastRobotTheta);
    //JustinaManip::startTorsoGoTo(goalTorso, 0, 0);
    JustinaNavigation::moveLateral(movLateral, 6000);
    JustinaNavigation::moveDist(movFrontal, 6000);
    int waitTime = (int) (30000 * movFrontal + 2000);
    //JustinaManip::waitForTorsoGoalReached(waitTime);

    bool found = false;
    std::vector<vision_msgs::VisionObject> recognizedObjects;
    int indexFound = 0;
    if (idObject.compare("") != 0) {
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
        JustinaManip::startLaOpenGripper(0.8);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        if(!JustinaManip::isLaInPredefPos("navigation"))
            JustinaManip::laGoTo("navigation", 7000);
        else
            std::cout << "JustinaTasks.->The left arm already has in the navigation pose" << std::endl;
 		
     	//Move the manipulator to objectOB

		JustinaManip::laGoToCartesian(objToGraspX - 0.10, objToGraspY - 0.25,
			      objToGraspZ, 0, 0, 1.5708, 0, 3000);
		JustinaNavigation::moveDist(0.10, 3000);
		boost::this_thread::sleep(boost::posix_time::milliseconds(500));

		JustinaManip::laGoToCartesian(objToGraspX - 0.10, objToGraspY - 0.15,
			      objToGraspZ, 0, 0, 1.5708, 0, 3000);
		boost::this_thread::sleep(boost::posix_time::milliseconds(500));

		JustinaManip::laGoToCartesian(objToGraspX - 0.04, objToGraspY - 0.03,
			      objToGraspZ, 0, 0, 1.5708, 0, 3000);
		boost::this_thread::sleep(boost::posix_time::milliseconds(500));
		
		JustinaManip::laGoToCartesian(objToGraspX, objToGraspY - 0.03,
			      objToGraspZ, 0, 0, 1.5708, 0, 3000);
		boost::this_thread::sleep(boost::posix_time::milliseconds(3500));

		//JustinaManip::laGoToCartesian(objToGraspX - 0.05, objToGraspY + 0.03,
		//objToGraspZ, 0, 0, 1.5708, 0, 3000);

		//boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

	

        JustinaManip::startLaCloseGripper(0.5);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
        ros::spinOnce();
        //JustinaManip::startTorsoGoTo(goalTorso + 0.03, 0, 0);
        //JustinaManip::waitForTorsoGoalReached(6000);
        if (JustinaManip::objOnLeftHand()) {
            JustinaManip::laGoToCartesian(objToGraspX - 0.13,
                    objToGraspY + 0.04, objToGraspZ, 0, 0, 1.5708, 0, 5000);
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
        JustinaManip::startRaOpenGripper(0.8);
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        if(!JustinaManip::isRaInPredefPos("navigation"))
            JustinaManip::raGoTo("navigation", 10000);
        else
            std::cout << "JustinaTasks.->The right arm already has in the navigation pose" << std::endl;

     	//Move the manipulator to object

		JustinaManip::raGoToCartesian(objToGraspX - 0.10, objToGraspY - 0.25,
					      objToGraspZ, 0, 0, 1.5708, 0, 3000);
		JustinaNavigation::moveDist(0.10, 3000);
		boost::this_thread::sleep(boost::posix_time::milliseconds(500));

		JustinaManip::raGoToCartesian(objToGraspX - 0.10, objToGraspY - 0.15,
					      objToGraspZ, 0, 0, 1.5708, 0, 3000);
		boost::this_thread::sleep(boost::posix_time::milliseconds(500));

		JustinaManip::raGoToCartesian(objToGraspX - 0.04, objToGraspY - 0.03,
					      objToGraspZ, 0, 0, 1.5708, 0, 3000);
		boost::this_thread::sleep(boost::posix_time::milliseconds(500));

		JustinaManip::raGoToCartesian(objToGraspX, objToGraspY - 0.03,
					      objToGraspZ, 0, 0, 1.5708, 0, 3000);
		boost::this_thread::sleep(boost::posix_time::milliseconds(3500));

		//JustinaManip::laGoToCartesian(objToGraspX - 0.05, objToGraspY + 0.03,
		//							objToGraspZ, 0, 0, 1.5708, 0, 3000);

		//boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

		JustinaManip::startRaCloseGripper(0.5);
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
        ros::spinOnce();
        //JustinaManip::startTorsoGoTo(goalTorso + 0.03, 0, 0);
        //JustinaManip::waitForTorsoGoalReached(6000);
        if (JustinaManip::objOnRightHand()) {
            JustinaManip::raGoToCartesian(objToGraspX - 0.1, objToGraspY + 0.04,
                    objToGraspZ, 0, 0, 1.5708, 0, 5000);
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
                    recog = getNearestRecognizedFace(facesObject, 3.0, centroidFace, genderRecog);
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

bool JustinaTasks::findPerson(std::string person, int gender, POSE pose) {

    std::vector<int> facesDistances;
    std::stringstream ss;

    JustinaVision::startFaceRecognitionOld();

    JustinaManip::startHdGoTo(0, 0.0);
    JustinaManip::waitForHdGoalReached(5000);

    std::cout << "Find a person " << person << std::endl;

    ss << person << ", I am going to find you";
    JustinaHRI::waitAfterSay(ss.str(), 2000);

    Eigen::Vector3d centroidFace;
    int genderRecog;
    bool recog = turnAndRecognizeFace(person, gender, pose, -M_PI_4, M_PI_4 / 2.0, M_PI_4, 0, -M_PI_4, -M_PI_4, M_PI_2, 2 * M_PI, centroidFace, genderRecog);
    std::cout << "Centroid Face in coordinates of robot:" << centroidFace(0, 0)
        << "," << centroidFace(1, 0) << "," << centroidFace(2, 0) << ")";
    std::cout << std::endl;
    //personLocation.clear();
    JustinaVision::stopFaceRecognition();

    ss.str("");
    if (!recog) {
        std::cout << "I have not found a person " << person << std::endl;
        ss << "I did not find the person " << person;
        JustinaHRI::waitAfterSay(ss.str(), 2000);
        return false;
    }

    std::cout << "I have found a person " << person << std::endl;
    ss << person << ", I found you";
    JustinaHRI::waitAfterSay(ss.str(), 2000);

    float cx, cy, cz;
    cx = centroidFace(0, 0);
    cy = centroidFace(1, 0);
    cz = centroidFace(2, 0);
    JustinaTools::transformPoint("/base_link", cx, cy, cz, "/map", cx, cy, cz);
    tf::Vector3 worldFaceCentroid(cx, cy, cz);

    JustinaHRI::waitAfterSay("I am getting close to you", 2000);
    closeToGoalWithDistanceTHR(worldFaceCentroid.x(), worldFaceCentroid.y(), 0.8, 20000);

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
    JustinaHRI::waitAfterSay(ss.str(), 2000);

    Eigen::Vector3d centroidGesture;
    bool recog = JustinaTasks::turnAndRecognizeGesture(gesture, -M_PI_4, M_PI_4 / 2.0, M_PI_4, -0.3, -0.2, -0.5, M_PI_2, 2 * M_PI, centroidGesture);
    std::cout << "Centroid Gesture in coordinates of robot:" << centroidGesture(0, 0) << "," << centroidGesture(1, 0) << "," << centroidGesture(2, 0) << ")";
    std::cout << std::endl;
    JustinaVision::stopSkeletonFinding();

    ss.str("");
    if (!recog) {
        std::cout << "I have not found a person" << std::endl;
        ss << "I did not find the person ";
        JustinaHRI::waitAfterSay(ss.str(), 2000);
        return false;
    }

    std::cout << "I have found a " << gestureSpeech << " person" << std::endl;
    ss << "I found you";
    JustinaHRI::waitAfterSay(ss.str(), 2000);

    float cx, cy, cz;
    cx = centroidGesture(0, 0);
    cy = centroidGesture(1, 0);
    cz = centroidGesture(2, 0);
    JustinaTools::transformPoint("/base_link", cx, cy, cz, "/map", cx, cy, cz);
    tf::Vector3 wgc(cx, cy, cz);

    JustinaHRI::waitAfterSay("I am getting close to you", 2000);

    closeToGoalWithDistanceTHR(wgc.x(), wgc.y(), 0.8, 20000);

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
    bool found = findPerson();
    if (!found)
        return false;
    std::stringstream ss;
    ss << "I am going to follow you to the " << goalLocation;
    std::cout << "Follow to the " << goalLocation << std::endl;
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
    JustinaHRI::waitAfterSay(ss.str(), 2000);

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
        JustinaHRI::waitAfterSay(ss.str(), 2000);
        return false;
    }

    std::cout << "I have found a person " << std::endl;
    ss << "I found you";
    JustinaHRI::waitAfterSay(ss.str(), 2000);

    float cx, cy, cz;
    cx = centroidFace(0, 0);
    cy = centroidFace(1, 0);
    cz = centroidFace(2, 0);
    JustinaTools::transformPoint("/base_link", cx, cy, cz, "/map", cx, cy, cz);
    tf::Vector3 worldFaceCentroid(cx, cy, cz);

    JustinaHRI::waitAfterSay("I am getting close to you", 2000);
    closeToGoalWithDistanceTHR(worldFaceCentroid.x(), worldFaceCentroid.y(), 0.8, 20000);

    JustinaVision::startFaceRecognitionOld();
    JustinaHRI::waitAfterSay("I have verified the information", 4000);
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

int JustinaTasks::manyGenderPerson(int gender){
    Eigen::Vector3d centroidFace;
    int genderRecog;
    //bool recog = turnAndRecognizeFace("", -1, -M_PI_4, M_PI_4, M_PI_4, M_PI_2, 2 * M_PI, centroidFace, genderRecog);
}

bool JustinaTasks::findObject(std::string idObject, geometry_msgs::Pose & pose,
        bool & withLeftOrRightArm) {
    std::vector<vision_msgs::VisionObject> recognizedObjects;
    std::stringstream ss;
    std::string toSpeech = idObject;

    boost::replace_all(idObject, "_", "-");
    boost::replace_all(toSpeech, "_", " ");

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
        JustinaHRI::waitAfterSay(ss.str(), 2000);
        return false;
    }

    ss << "I found the " << toSpeech;
    JustinaHRI::waitAfterSay(ss.str(), 2000);

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
        bool withLeftArm, std::string id) {
    std::cout << "Move actuator " << id << std::endl;
    std::stringstream ss;

    if (withLeftArm)
        std::cout << "left arm" << std::endl;
    else
        std::cout << "right arm" << std::endl;

    ss << "I am going to take the " << id;
    JustinaHRI::waitAfterSay(ss.str(), 2000);

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
            JustinaTasks::alignWithTable(0.35);
            JustinaManip::startHdGoTo(0, -0.785);
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
                    attemps < maxAttemps - 1 ? id : "");
    }

    //JustinaManip::laGoTo("home", 10000);
    return isGrasp;

}

bool JustinaTasks::dropObject(std::string id, bool withLeftOrRightArm, int timeout) {
    float x, y, z;

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
    JustinaManip::hdGoTo(0, -0.9, 5000);
    JustinaHRI::waitAfterSay("please put your hand", 2000);

    // If withLeftOrRightArm is false the arm to use is the right and else the arm to use is the left.
    if(!withLeftOrRightArm){
        JustinaManip::raGoTo("take", 10000);
        JustinaManip::getRightHandPosition(x, y, z);
    }
    else{
        JustinaManip::laGoTo("take", 10000);
        JustinaManip::getLeftHandPosition(x, y, z);
    }

    boost::this_thread::sleep(boost::posix_time::milliseconds(200));
    //JustinaVision::startHandDetectBB(0.50, -0.15, 0.95);
    JustinaVision::startHandDetectBB(x, y, z);
    ros::Rate rate(10);
    while (ros::ok() && !JustinaVision::getDetectionHandBB() && (curr - prev).total_milliseconds() < timeout) {
        rate.sleep();
        ros::spinOnce();
        curr = boost::posix_time::second_clock::local_time();
    }
    JustinaVision::stopHandDetectBB();
    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
    JustinaHRI::waitAfterSay("I am going hand over the object", 2000);
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

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

bool JustinaTasks::placeObject(bool withLeftArm, float h, bool placeBag) {
    std::cout << "JustinaTasks::placeObject..." << std::endl;
    std::vector<float> vacantPlane;
    std::vector<int> inliers;
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> z;
    std::vector<float> distance;
    float maximunInliers = 0;
    float objToGraspX;
    float objToGraspY;
    float objToGraspZ;

    int maxInliersIndex;

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
        x.push_back( vacantPlane[ i ] );
        y.push_back( vacantPlane[i+1] );
        z.push_back( vacantPlane[i+2] );
    }

    for(int i = 0; i < x.size();i++)
    {
        //std::cout << "P[" << i << "]:  (" << x[i] << ", " << y[i] << ", "  << z[i] << ")" << std::endl;
        //std::cout << "inliers[" << i << "]:  " << inliers[i] << std::endl;
        if(inliers[i] > maximunInliers)
        {
            maximunInliers = inliers[i];
            maxInliersIndex = i;
        }
    }

    std::cout << "Justina::Tasks->PlaceObject  P_max[" << maxInliersIndex << "]:  (" << x[maxInliersIndex] << ", " << y[maxInliersIndex] << ", "  << z[maxInliersIndex] << " + " << h << ")" << std::endl;
    std::cout << "Justina::Tasks->PlaceObject  inliers_max[" << maxInliersIndex << "]:  " << inliers[maxInliersIndex] << std::endl;

    std::string destFrame = withLeftArm ? "left_arm_link0" : "right_arm_link0";

    if(withLeftArm)
    {
        JustinaNavigation::moveLateral(y[maxInliersIndex]-0.34, 3000);
        y[maxInliersIndex] = 0.22;
        if (!JustinaTools::transformPoint("base_link", x[maxInliersIndex], y[maxInliersIndex],
                    z[maxInliersIndex]+ (z[maxInliersIndex]*0.05) + h, destFrame, objToGraspX, objToGraspY, objToGraspZ))
        {
            std::cout << "JustinaTasks.->Cannot transform point. " << std::endl;
            return false;
        }
        std::cout << "Moving left arm to P[wrtr]:  (" << x[maxInliersIndex] << ", " << y[maxInliersIndex] << ", "  << z[maxInliersIndex]+ (z[maxInliersIndex]*0.05) + h << ")" << std::endl;
        if(!JustinaManip::isLaInPredefPos("navigation"))
        {
            std::cout << "Left Arm is not already on navigation position" << std::endl;
            JustinaManip::laGoTo("navigation", 7000);
        }

        // Verify if the height of plane is longer than 1.2 if not calculate the
        // inverse kinematic.
        if(z[maxInliersIndex] > 1.2)
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
            JustinaManip::startLaGoTo("home");
            JustinaManip::startHdGoTo(0.0, 0.0);
        }

    }
    else
    {
        JustinaNavigation::moveLateral(y[maxInliersIndex]+0.32, 3000);
        y[maxInliersIndex] = -0.22;
        if (!JustinaTools::transformPoint("base_link", x[maxInliersIndex], y[maxInliersIndex],
                    z[maxInliersIndex] + (z[maxInliersIndex]*0.05) +h, destFrame, objToGraspX, objToGraspY, objToGraspZ))
        {
            std::cout << "JustinaTasks.->Cannot transform point. " << std::endl;
            return false;
        }
        std::cout << "Moving right arm to P[wrtr]:  (" << x[maxInliersIndex] << ", " << y[maxInliersIndex] << ", "  << z[maxInliersIndex]+ (z[maxInliersIndex]*0.05) + h << ")" << std::endl;
        if(!JustinaManip::isRaInPredefPos("navigation"))
        {
            std::cout << "Right Arm is not already on navigation position" << std::endl;
            JustinaManip::raGoTo("navigation", 7000);
        }

        if(z[maxInliersIndex] > 1.2)
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
            JustinaManip::startRaGoTo("home");
            JustinaManip::startHdGoTo(0.0, 0.0);

        }
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
        if(!success && timeout != 0){
            ss.str("");
            ss << "I cannot guide you to the  ";
            boost::algorithm::split(tokens, loc, boost::algorithm::is_any_of("_"));
            for(int i = 0; i < tokens.size(); i++)
                ss << tokens[i] << " ";
            JustinaHRI::waitAfterSay(ss.str(), 2500);
        }
        rate.sleep();
        ros::spinOnce();
        curr = boost::posix_time::second_clock::local_time();
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

bool JustinaTasks::findTable()
{
	std::cout << "JustinaTask::findTable" << std::endl;

	JustinaHRI::waitAfterSay("I am going to search the closes table", 2500);
	boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

	//Turn head to left
	JustinaHRI::waitAfterSay("I am serching table on my left hand", 2500);	
	JustinaManip::hdGoTo(0.9, -0.5, 4000);
	boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
	if(JustinaVision::findPlane())
	{
		JustinaHRI::waitAfterSay("I have found a table", 1500);
		JustinaNavigation::startMoveDistAngle(0.0, M_PI_2);
		JustinaManip::hdGoTo(0.0, -0.5, 4000);
		return true;
	}


	JustinaHRI::waitAfterSay("I am serching table in front of me", 1500);	
	JustinaManip::hdGoTo(0.0, -0.4, 4000);
	boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
	if(JustinaVision::findPlane())
	{
		JustinaHRI::waitAfterSay("I have found a table", 1500);
		return true;
	}

	//Turn head to right
	JustinaHRI::waitAfterSay("I am serching table on my right hand", 1500);	
	JustinaManip::hdGoTo(-0.9, -0.5, 4000);
	boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
	if(JustinaVision::findPlane())
	{
		JustinaHRI::waitAfterSay("I have found a table", 1500);
		JustinaNavigation::startMoveDistAngle(0.0, -M_PI_2);
		JustinaManip::hdGoTo(0.0, -0.5, 4000);
		return true;
	}

	return false;

}

bool JustinaTasks::findAndAlignTable()
{
	std::cout << "JustinaTask::findAndAlignTable" << std::endl;

	if(JustinaTasks::findTable())
	{
		JustinaHRI::waitAfterSay("I am searching the line of the table", 3000);
		JustinaNavigation::moveDist(-0.15, 3000);
		for(int i = 0; i < 4; i++)
		{
			if( JustinaTasks::alignWithTable(0.35) )
			{
				JustinaHRI::waitAfterSay("I found the table", 3000);
				return true;
			}
		}
	}
	else
	{
		return false;
	}

}