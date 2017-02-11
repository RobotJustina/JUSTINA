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
		std::vector<float> temp;
		for (int i = 0; i < 7; i++)
			temp.push_back(0);
		temp[5] = 1.0;
		JustinaManip::laGoToArticular(temp, 3000);
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

bool JustinaTasks::graspObject(float x, float y, float z, bool withLeftArm) {
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
	JustinaNavigation::getRobotPoseFromOdom(lastRobotX, lastRobotY,
			lastRobotTheta);
	//JustinaManip::startTorsoGoTo(goalTorso, 0, 0);
	JustinaNavigation::moveLateral(movLateral, 6000);
	JustinaNavigation::moveDist(movFrontal, 6000);
	int waitTime = (int) (30000 * movFrontal + 2000);
	//JustinaManip::waitForTorsoGoalReached(waitTime);
	float robotX, robotY, robotTheta;
	//JustinaNavigation::getRobotPose(robotX, robotY, robotTheta);
	JustinaNavigation::getRobotPoseFromOdom(robotX, robotY, robotTheta);
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
		boost::this_thread::sleep(boost::posix_time::milliseconds(4000));
		std::vector<float> temp;
		for (int i = 0; i < 7; i++)
			temp.push_back(0);
		temp[5] = 1.0;
		JustinaManip::laGoToArticular(temp, 7000);
		JustinaManip::laGoTo("navigation", 7000);
		JustinaManip::laGoToCartesian(objToGraspX - 0.03, objToGraspY - 0.04,
				objToGraspZ, 0, 0, 1.5708, 0, 12000);
		JustinaManip::startLaCloseGripper(0.4);
		boost::this_thread::sleep(boost::posix_time::milliseconds(4000));
		JustinaManip::startTorsoGoTo(goalTorso + 0.03, 0, 0);
		JustinaManip::waitForTorsoGoalReached(6000);
		JustinaNavigation::moveDist(-0.15, 3000);
		JustinaManip::laGoTo("navigation", 5000);
	} else {
		JustinaManip::startRaOpenGripper(0.7);
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
		JustinaManip::raGoTo("navigation", 10000);
		JustinaManip::raGoToCartesian(objToGraspX - 0.03, objToGraspY - 0.04,
				objToGraspZ, 0, 0, 1.5708, 0, 5000);
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
		JustinaManip::startRaCloseGripper(0.5);
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
		//JustinaManip::startTorsoGoTo(goalTorso + 0.03, 0, 0);
		//JustinaManip::waitForTorsoGoalReached(3000);
		JustinaNavigation::moveDist(-0.15, 3000);
		JustinaManip::raGoTo("navigation", 5000);
	}
}

void JustinaTasks::sayAndAsyncNavigateToLoc(std::string location) {
	std::stringstream ss;
	std::cout << "Navigation to " << location << std::endl;
	ss << "I will navigate to the " << location;
	JustinaHRI::say(ss.str());
	JustinaNavigation::startGetClose(location);
}

bool JustinaTasks::sayAndSyncNavigateToLoc(std::string location, int timeout) {
	std::stringstream ss;
	std::cout << "Navigation to " << location << std::endl;
	ss << "I will navigate to the " << location;
	JustinaHRI::say(ss.str());
	bool reachedLocation = JustinaNavigation::getClose(location, timeout);
	ss.str("");
	if (reachedLocation) {
		ss << "I have reached the " << location;
		JustinaHRI::waitAfterSay(ss.str(), 4000);
	} else {
		ss.str("");
		ss << "I cannot reached the " << location;
		JustinaHRI::waitAfterSay(ss.str(), 4000);
	}
	return reachedLocation;
}

std::vector<vision_msgs::VisionFaceObject> JustinaTasks::waitRecognizedFace(
		float timeout, std::string id, bool &recognized) {
	boost::posix_time::ptime curr;
	boost::posix_time::ptime prev =
			boost::posix_time::second_clock::local_time();
	boost::posix_time::time_duration diff;
	std::vector<vision_msgs::VisionFaceObject> lastRecognizedFaces;
	if (id.compare("") == 0)
		JustinaVision::facRecognize();
	else
		JustinaVision::facRecognize(id);
	do {
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
		ros::spinOnce();
		JustinaVision::getLastRecognizedFaces(lastRecognizedFaces);
		curr = boost::posix_time::second_clock::local_time();
	} while (ros::ok() && (curr - prev).total_milliseconds() < timeout
			&& lastRecognizedFaces.size() == 0);

	if (lastRecognizedFaces.size() > 0)
		recognized = true;
	else
		recognized = false;
	std::cout << "recognized:" << recognized << std::endl;
	return lastRecognizedFaces;
}

Eigen::Vector3d JustinaTasks::getNearestRecognizedFace(
		std::vector<vision_msgs::VisionFaceObject> facesObject,
		float distanceMax, bool &found) {
	int indexMin;
	float distanceMin = 99999999.0;
	Eigen::Vector3d faceCentroid = Eigen::Vector3d::Zero();
	found = false;
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
	}
	std::cout << "Face centroid:" << faceCentroid(0, 0) << ","
			<< faceCentroid(1, 0) << "," << faceCentroid(2, 0);
	std::cout << std::endl;
	return faceCentroid;
}

Eigen::Vector3d JustinaTasks::turnAndRecognizeFace(std::string id,
		float initAngPan, float incAngPan, float maxAngPan, float incAngleTurn,
		float maxAngleTurn, bool &recog) {

	float currAngPan = initAngPan;
	float currAngleTurn = 0.0;
	float turn = 0.0;
	bool continueReco = true;
	Eigen::Vector3d centroidFace = Eigen::Vector3d::Zero();

	do {
		std::cout << "Move base" << std::endl;
		std::cout << "currAngleTurn:" << currAngleTurn << std::endl;
		JustinaHardware::setHeadGoalPose(currAngPan, 0.0);
		JustinaNavigation::moveDistAngle(0, turn, 10000);
		JustinaHardware::waitHeadGoalPose(currAngPan, 0.0, 5000);
		do {
			std::cout << "Sync move head start" << std::endl;
			std::cout << "Head goal:" << currAngPan << std::endl;
			JustinaHardware::setHeadGoalPose(currAngPan, 0.0);
			JustinaHardware::waitHeadGoalPose(currAngPan, 0.0, 5000);
			std::cout << "Sync move head end" << std::endl;
			currAngPan += incAngPan;
			boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
			std::vector<vision_msgs::VisionFaceObject> facesObject =
					waitRecognizedFace(2000, id, recog);
			if (continueReco)
				centroidFace = getNearestRecognizedFace(facesObject, 3.0,
						recog);
			if (recog)
				continueReco = false;
		} while (ros::ok() && currAngPan <= maxAngPan && continueReco);
		std::cout << "End turnAndRecognizeFace" << std::endl;
		currAngleTurn += incAngleTurn;
		currAngPan = initAngPan;
		turn = incAngleTurn;
	} while (ros::ok() && currAngleTurn < maxAngleTurn && continueReco);
	return centroidFace;
}

bool JustinaTasks::findPerson(std::string person) {

	std::vector<int> facesDistances;
	std::stringstream ss;

	JustinaVision::startFaceRecognitionOld();

	JustinaHardware::setHeadGoalPose(0, 0.0);
	JustinaHardware::waitHeadGoalPose(0, 0.0, 5000);

	std::cout << "Find a person " << person << std::endl;

	ss << "I am going to find a person " << person;
	JustinaHRI::waitAfterSay(ss.str(), 2000);

	bool recog;
	Eigen::Vector3d centroidFace = turnAndRecognizeFace(person, -M_PI_4,
	M_PI_4, M_PI_4, M_PI_2, 2 * M_PI, recog);
	std::cout << "CentroidFace:" << centroidFace(0, 0) << ","
			<< centroidFace(1, 0) << "," << centroidFace(2, 0) << ")";
	std::cout << std::endl;
	//personLocation.clear();
	JustinaVision::stopFaceRecognition();

	ss.str("");
	if (!recog) {
		std::cout << "I have not found a person " << person << std::endl;
		ss << "I have not found a person " << person;
		JustinaHRI::waitAfterSay(ss.str(), 2000);
		return false;
	}

	std::cout << "I have found a person " << person << std::endl;
	ss << "I have found a person " << person;
	JustinaHRI::waitAfterSay(ss.str(), 2000);

	float cx, cy, cz;
	cx = centroidFace(0, 0);
	cy = centroidFace(1, 0);
	cz = centroidFace(2, 0);
	JustinaTools::transformPoint("/base_link", cx, cy, cz, "/map", cx, cy, cz);
	tf::Vector3 worldFaceCentroid(cx, cy, cz);

	JustinaHRI::waitAfterSay("I am getting close to you", 2000);
	//personLocation.push_back(worldFaceCentroid);

	JustinaNavigation::startGetClose(worldFaceCentroid.x(),
			worldFaceCentroid.y());

	float currx, curry, currtheta;
	bool finishReachedPerson = false;
	do {
		float distanceToGoal;
		JustinaNavigation::getRobotPose(currx, curry, currtheta);
		distanceToGoal = sqrt(
				pow(currx - worldFaceCentroid.x(), 2)
						+ pow(curry - worldFaceCentroid.y(), 2));
		if ((JustinaNavigation::obstacleInFront() && distanceToGoal < 1.0)
				|| distanceToGoal < 1.0)
			finishReachedPerson = true;
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		ros::spinOnce();
	} while (ros::ok() && !finishReachedPerson);

	JustinaHardware::setHeadGoalPose(0, 0.0);
	JustinaHardware::waitHeadGoalPose(0, 0.0, 5000);

	return true;
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

bool JustinaTasks::findObject(std::string idObject,
		geometry_msgs::Pose & pose) {
	std::vector<vision_msgs::VisionObject> recognizedObjects;
	std::stringstream ss;
	std::string toSpeech = idObject;

	boost::replace_all(idObject, "_", "-");
	boost::replace_all(toSpeech, "_", " ");

	std::cout << "Find a object " << idObject << std::endl;

	JustinaHardware::setHeadGoalPose(0, -0.7854);
	JustinaHardware::waitHeadGoalPose(0, -0.7854, 5000);

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
		ss << "I have not found the object " << toSpeech;
		JustinaHRI::waitAfterSay(ss.str(), 2000);
		return false;
	}

	ss << "I have found the object " << toSpeech;
	JustinaHRI::waitAfterSay(ss.str(), 2000);

	pose = recognizedObjects[indexFound].pose;
	std::cout << "Position:" << pose.position.x << "," << pose.position.y << ","
			<< pose.position.z << std::endl;
	std::cout << "Orientation:" << pose.orientation.x << ","
			<< pose.orientation.y << "," << pose.orientation.z << ","
			<< pose.orientation.w << std::endl;

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

	ss << "I am going to take an object " << id;
	JustinaHRI::waitAfterSay(ss.str(), 2000);

	JustinaTasks::graspObject(x, y, z, false);

	//JustinaManip::laGoTo("home", 10000);
	return true;

}

bool JustinaTasks::dropObject() {
	JustinaHRI::waitAfterSay("I am going to bring it to you", 2000);
	JustinaHRI::waitAfterSay("please put your hand", 2000);
	boost::this_thread::sleep(boost::posix_time::milliseconds(10000));
	JustinaManip::raGoTo("take", 10000);
	boost::this_thread::sleep(boost::posix_time::milliseconds(10000));
	JustinaHRI::waitAfterSay("I am going handover the object", 2000);
	JustinaManip::startRaOpenGripper(0.6);
	JustinaManip::raGoTo("home", 10000);
	return true;
}
