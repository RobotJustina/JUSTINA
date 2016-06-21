
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

#include "planning_msgs/PlanningCmdClips.h"
#include "planning_msgs/planning_cmd.h"

#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"

#include <vector>
#include <ctime>

using namespace boost::algorithm;

class GPSRTasks{
public:
	GPSRTasks(ros::NodeHandle * n = 0){
		JustinaNavigation::setNodeHandle(n);
		JustinaHardware::setNodeHandle(n);
		JustinaHRI::setNodeHandle(n);
		JustinaVision::setNodeHandle(n);
		if(n != 0){
			publisFollow = n->advertise<std_msgs::Bool>("/hri/human_following/start_follow", 1);	
			cltSpgSay = n->serviceClient<bbros_bridge::Default_ROS_BB_Bridge>("/spg_say");
		}
	}

	void initRosConnection(ros::NodeHandle * n, std::string locationsFilePath){
		JustinaNavigation::setNodeHandle(n);
		JustinaHardware::setNodeHandle(n);
		JustinaHRI::setNodeHandle(n);
		JustinaVision::setNodeHandle(n);
		publisFollow = n->advertise<std_msgs::Bool>("/hri/human_following/start_follow", 1);
		cltSpgSay = n->serviceClient<bbros_bridge::Default_ROS_BB_Bridge>("/spg_say");
		loadKnownLocations(locationsFilePath);
		listener = new tf::TransformListener();
		std::cout << "Size of map location:" << locations.size() << std::endl;
		//speechTasks.initRosConnection(n);
	}

	bool loadKnownLocations(std::string path)
	{
	    std::cout << "MvnPln.->Loading known locations from " << path << std::endl;
	    std::vector<std::string> lines;
	    std::ifstream file(path.c_str());
	    std::string tempStr;
	    while(std::getline(file, tempStr))
	        lines.push_back(tempStr);

	    //Extraction of lines without comments
	    for(size_t i=0; i< lines.size(); i++)
	    {
	        size_t idx = lines[i].find("//");
	        if(idx!= std::string::npos)
	            lines[i] = lines[i].substr(0, idx);
	    }

	    this->locations.clear();
	    float locX, locY, locAngle;
	    bool parseSuccess;
	    for(size_t i=0; i<lines.size(); i++)
	    {
	        //std::cout << "MvnPln.->Parsing line: " << lines[i] << std::endl;
	        std::vector<std::string> parts;
	        std::vector<float> loc;
	        boost::split(parts, lines[i], boost::is_any_of(" ,\t"), boost::token_compress_on);
	        if(parts.size() < 3)
	            continue;
	        //std::cout << "MvnPln.->Parsing splitted line: " << lines[i] << std::endl;
	        parseSuccess = true;
	        std::stringstream ssX(parts[1]);
	        if(!(ssX >> locX)) parseSuccess = false;
	        std::stringstream ssY(parts[2]);
	        if(!(ssY >> locY)) parseSuccess = false;
	        loc.push_back(locX);
	        loc.push_back(locY);
	        if(parts.size() >= 4)
	        {
	            std::stringstream ssAngle(parts[3]);
	            if(!(ssAngle >> locAngle)) parseSuccess = false;
	            loc.push_back(locAngle);
	        }

	        if(parseSuccess)
	        {
	            this->locations[parts[0]] = loc;
	        }
	    }
	    std::cout << "GPSRTasks.->Total number of known locations: " << this-locations.size() << std::endl;
	    for(std::map<std::string, std::vector<float> >::iterator it=this->locations.begin(); it != this->locations.end(); it++)
	    {
	        std::cout << "GPSRTasks.->Location " << it->first << " " << it->second[0] << " " << it->second[1];
	        if(it->second.size() > 2)
	            std::cout << " " << it->second[2];
	        std::cout << std::endl;
	    }
	    if(this->locations.size() < 1)
	        std::cout << "GPSRTasks.->WARNING: Cannot load known locations from file: " << path << ". There are no known locations." << std::endl;
	    
	    return true;
	}

	tf::Vector3 transformPoint(tf::StampedTransform transform, tf::Vector3 point){
		return transform * point;
	}

	tf::StampedTransform getTransform(std::string frame1, std::string frame2){
		tf::StampedTransform transform;
		try{
			listener->lookupTransform(frame1, frame2,  
                             	ros::Time(0), transform);
		}
		catch(tf::TransformException ex){
			std::cerr << "error:" << ex.what() << std::endl;
			ros::Duration(1.0).sleep();
		}
		return transform;
	}

	bool syncNavigate(std::string location, float timeOut){
		std::stringstream ss;
		std::cout << "Navigation to " << location << std::endl;
		ss << "I will navigate to the " << location;
		asyncSpeech(ss.str());
		bool reachedLocation = JustinaNavigation::getClose(location, timeOut);
		ss.str("");
		if(reachedLocation){
			ss << "I have reached the " << location;
			syncSpeech(ss.str(), 30000, 2000);
		}
		else{
			ss.str("");
			ss << "I cannot reached the " << location;
			syncSpeech(ss.str(), 30000, 2000);
		}
		return reachedLocation;
	}

	bool syncNavigate(float x, float y, float timeOut){
		return JustinaNavigation::getClose(x, y, timeOut);
	}

	bool syncMove(float distance, float angle, float timeOut){
		return JustinaNavigation::moveDistAngle(distance, angle, timeOut);
	}

	void waitHeadGoalPose(float goalHeadPan, float goalHeadTile, float timeOut){
		float currHeadPan, currHeadTile;
		float errorPan, errorTile;
		boost::posix_time::ptime curr;
		boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
		do{
			boost::this_thread::sleep(boost::posix_time::milliseconds(100));
			JustinaHardware::getHeadCurrentPose(currHeadPan, currHeadTile);
			errorPan = pow(currHeadPan - goalHeadPan, 2);
			errorTile = pow(currHeadTile - goalHeadTile, 2);
			curr = boost::posix_time::second_clock::local_time();
		}while(ros::ok() && errorPan > 0.003 && errorTile > 0.003 && (curr - prev).total_milliseconds() < timeOut);
	}

	void syncMoveHead(float goalHeadPan, float goalHeadTile, float timeOut){
		JustinaHardware::setHeadGoalPose(goalHeadPan, goalHeadTile);
		waitHeadGoalPose(goalHeadPan, goalHeadTile, timeOut);
	}

	void asyncMoveHead(float goalHeadPan, float goalHeadTile){
		JustinaHardware::setHeadGoalPose(goalHeadPan, goalHeadTile);
	}

	void eneableDisableQR(bool option){
		if(option)
			JustinaVision::startQRReader();
		else
			JustinaVision::stopQRReader();
	}

	std::vector<vision_msgs::VisionFaceObject> waitRecognizeFace(float timeOut, std::string id, bool &recognized){
		boost::posix_time::ptime curr;
		boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
		boost::posix_time::time_duration diff;
		std::vector<vision_msgs::VisionFaceObject> lastRecognizedFaces;
		if(id.compare("") == 0)
			JustinaVision::facRecognize();
		else
			JustinaVision::facRecognize(id);
		do{
			boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
			ros::spinOnce();
			JustinaVision::getLastRecognizedFaces(lastRecognizedFaces);
			curr = boost::posix_time::second_clock::local_time();
		}while(ros::ok() && (curr - prev).total_milliseconds() < timeOut && lastRecognizedFaces.size() == 0);

		if(lastRecognizedFaces.size() > 0)
			recognized = true;
		else
			recognized = false;
		std::cout << "recognized:" << recognized << std::endl;
		return lastRecognizedFaces;
	}

	Eigen::Vector3d filterRecognizeFace(std::vector<vision_msgs::VisionFaceObject> facesObject, float distanceMax, 
		bool &found){
		int indexMin;
		float distanceMin = 99999999.0;
		Eigen::Vector3d faceCentroid = Eigen::Vector3d::Zero();
		found = false;
		for(int i = 0; i < facesObject.size(); i++){
			vision_msgs::VisionFaceObject vro = facesObject[i];
			Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
			centroid(0,0) = vro.face_centroid.x;
			centroid(1,0) = vro.face_centroid.y;
			centroid(2,0) = vro.face_centroid.z;
			float dist = centroid.norm();
			if(dist < distanceMax && dist < distanceMin){
				indexMin = i;
				distanceMin = dist;
				found = true;
			}
		}
		if(found){
			std::cout << "I found the centroid nearest to robot" << std::endl;
			faceCentroid(0, 0) = facesObject[indexMin].face_centroid.x;
			faceCentroid(1, 0) = facesObject[indexMin].face_centroid.y;
			faceCentroid(2, 0) = facesObject[indexMin].face_centroid.z;
		}
		std::cout << "Face centroid:" << faceCentroid(0, 0) << "," << faceCentroid(1, 0) << "," 
			<< faceCentroid(2, 0) << std::endl;
		return faceCentroid;
	}

	Eigen::Vector3d turnAndRecognizeFace(std::string id,float initAngPan, float incAngPan, float maxAngPan, 
		float incAngleTurn, float maxAngleTurn, bool &recog){

		float currAngPan = initAngPan;
		float currAngleTurn = 0.0;
		float turn = 0.0;
		bool continueReco = true;
		Eigen::Vector3d centroidFace = Eigen::Vector3d::Zero();
		
		do{
			std::cout << "Move base" << std::endl;
			std::cout << "currAngleTurn:"  << currAngleTurn << std::endl;
			asyncMoveHead(currAngPan, 0.0);
			syncMove(0.0, turn, 5000);
			waitHeadGoalPose(currAngPan, 0.0, 5000);
			do{
				std::cout << "Sync move head start" << std::endl;
				std::cout << "Head goal:"  << currAngPan << std::endl;
				syncMoveHead(currAngPan, 0.0, 5000);
				std::cout << "Sync move head end" << std::endl;
				currAngPan += incAngPan;
				boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
				std::vector<vision_msgs::VisionFaceObject> facesObject = waitRecognizeFace(2000, id, recog);
				if(continueReco)
					centroidFace = filterRecognizeFace(facesObject, 3.0, recog);
				if(recog)
					continueReco = false;
			}while(ros::ok() && currAngPan <= maxAngPan && continueReco);
			std::cout << "End turnAndRecognizeFace" << std::endl;
			currAngleTurn += incAngleTurn;
			currAngPan = initAngPan;
			turn = incAngleTurn;
		}while(ros::ok() && currAngleTurn < maxAngleTurn && continueReco);
		return centroidFace;
	}

	bool syncSpeech(std::string textSpeech, float timeOut, float timeSleep){
		bbros_bridge::Default_ROS_BB_Bridge srv;
    	srv.request.parameters = textSpeech;
    	srv.request.timeout = timeOut;
    	if(cltSpgSay.call(srv)){
    		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    		return true;
    	}
		return false;
	}

	void asyncSpeech(std::string textSpeech){
		bbros_bridge::Default_ROS_BB_Bridge srv;
    	srv.request.parameters = textSpeech;
    	srv.request.timeout = 0;
    	cltSpgSay.call(srv);
	}

	bool syncDetectObjects(std::vector<vision_msgs::VisionObject>& recoObjList){
		return JustinaVision::detectObjects(recoObjList);
	}

	void getCurrPose(float &x, float &y, float &theta){
		JustinaNavigation::getRobotPose(x, y, theta);
	}

	bool findPerson(std::string person = ""){

		std::vector<int> facesDistances;
		std::stringstream ss;

		JustinaVision::startFaceRecognitionOld();
		syncMoveHead(0, 0, 5000);

		std::cout << "Find a person " << person << std::endl;

		ss << "I am going to find a person " << person;
		syncSpeech(ss.str(), 30000, 2000);

		bool recog;
		Eigen::Vector3d centroidFace = turnAndRecognizeFace(person, -M_PI_4, M_PI_4, M_PI_4, M_PI_2, 2 * M_PI, recog);
		std::cout << "CentroidFace:" << centroidFace(0,0) << "," << centroidFace(1,0) << "," << centroidFace(2,0) << ")" << std::endl;
		personLocation.clear();
		JustinaVision::stopFaceRecognition();

		ss.str("");
		if(!recog){
			std::cout << "I have not found a person " << person << std::endl;
			ss << "I have not found a person " << person;
			syncSpeech(ss.str(), 30000, 2000);
			return false;
		}

		std::cout << "I have found a person " << person << std::endl;
		ss << "I have found a person " << person;
		syncSpeech(ss.str(), 30000, 2000);

		tf::StampedTransform transform = getTransform("/map", "/base_link");
		std::cout << "Transform:" << transform.getOrigin().x() << "," << transform.getOrigin().y() << ","
				<< transform.getOrigin().z() << std::endl;

		/*tf::Vector3 kinectFaceCentroid(-centroidFace(1, 0), centroidFace(0, 0), 
					centroidFace(2, 0));*/
		tf::Vector3 kinectFaceCentroid(centroidFace(0, 0), centroidFace(1, 0), centroidFace(2, 0));
		tf::Vector3 worldFaceCentroid = transformPoint(transform, kinectFaceCentroid);
		std::cout << "Person 3D:" << worldFaceCentroid.x() << "," << worldFaceCentroid.y() << ","
				<< worldFaceCentroid.z() << std::endl;

		/*tf::StampedTransform transRobot = getTransform("/map", "/base_link");
		Eigen::Affine3d transRobotEigen;
		tf::transformTFToEigen(transRobot, transRobotEigen);
		Eigen::AngleAxisd angAxis(transRobotEigen.rotation());
		double currRobotAngle = angAxis.angle();
		Eigen::Vector3d axis = angAxis.axis();
		if(axis(2, 0) < 0.0)
			currRobotAngle = 2 * M_PI - currRobotAngle;

		Eigen::Vector3d robotPostion = transRobotEigen.translation();
		float deltaX = worldFaceCentroid.x() - robotPostion(0, 0);
    	float deltaY = worldFaceCentroid.y() - robotPostion(1, 0);
    	float distance = sqrt(pow(deltaX,2) + pow(deltaY,2));
    	float goalRobotAngle = atan2(deltaY, deltaX);
    	if (goalRobotAngle < 0.0f)
        	goalRobotAngle = 2 * M_PI + goalRobotAngle;
		float turn = goalRobotAngle - currRobotAngle;*/

		//std::cout << "turn:" << turn << "distance:" << distance << std::endl;
		//syncMove(0.0, turn, 10000);
		//syncMove(distance - 0.9, 0.0, 10000);

        personLocation.push_back(worldFaceCentroid);
		syncNavigate(worldFaceCentroid.x(), worldFaceCentroid.y(), 10000);
		//syncMove(0.0, turn, 10000);

		syncMoveHead(0, 0, 5000);

		return true;
	}

	bool findMan(std::string goalLocation){
		bool found = findPerson();
		if(!found)
			return false;
		std::stringstream ss;
		ss << "I have a follow you to the " << goalLocation << std::endl;
		std::cout << "Follow to the " << goalLocation << std::endl;
		asyncSpeech(ss.str());
		std_msgs::Bool msg;
		msg.data = true;
		publisFollow.publish(msg);
		
		float currx, curry, currtheta;
		float errorx, errory;
		float dis;
		std::vector<float> location = locations.find(goalLocation)->second;
		do{
			getCurrPose(currx, curry, currtheta);
			errorx = currx - location[0];
			errory = curry - location[1];
			dis = sqrt(pow(errorx,2) + pow(errory,2));
			boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		}while(ros::ok() && dis > 0.6);

		std::cout << "I have reach a location to follow a person in the " << goalLocation << std::endl;
		ss.str("");
		ss << "I have finish follow a person ";
		asyncSpeech(ss.str());

		msg.data = false;
		publisFollow.publish(msg);
		return true;

	}

	bool findObject(std::string idObject, geometry_msgs::Pose & pose){
		std::vector<vision_msgs::VisionObject> recognizedObjects;
		std::stringstream ss;

		std::cout << "Find a object " << idObject << std::endl;

		syncMoveHead(0, -0.7854, 5000);
		float x1, y1, z1, x2, y2, z2;
		bool foundLine = JustinaVision::findLine(x1, y1, z1, x2, y2, z2);
		std::cout << "foundLine:" << foundLine << std::endl;
		if(!foundLine){
			ss << "I have not found an object " << idObject;
			syncSpeech(ss.str(), 30000, 2000);
			return false;
		}
		std::cout << "P1(" << x1 << "," << y1 << "," << z1 << ")" << std::endl;
		std::cout << "P2(" << x2 << "," << y2 << "," << z2 << ")" << std::endl;

		// This is for transform the point to coordinates of world
		/*tf::StampedTransform transform = getTransform("/map", "/base_link");
		std::cout << "Transform:" << transform.getOrigin().x() << "," << transform.getOrigin().y() << ","
				<< transform.getOrigin().z() << std::endl;

		tf::Vector3 p1(x1, y1, z1);
		tf::Vector3 p2(x2, y2, z2);
		tf::Vector3 p1w = transformPoint(transform, p1);
		tf::Vector3 p2w = transformPoint(transform, p2);
		std::cout << "Line 3D:" << p1w.x() << "," << p1w.y() << "," << p1w.z() << std::endl;
		std::cout << "Line 3D:" << p2w.x() << "," << p2w.y() << "," << p2w.z() << std::endl;*/
		
		syncMoveHead(0, 0, 5000);

		float deltax , deltay;
		deltax = x1 - x2;
		deltay = y1 - y2;

		float currx, curry, currtheta;
		getCurrPose(currx, curry, currtheta);
		float secondPx = currx + cos(currtheta);
		float secondPy = curry + sin(currtheta);
		
		Eigen::Vector3d v1 = Eigen::Vector3d::Zero();
		v1(0, 0) = currx - secondPx;
		v1(1, 0) = curry - secondPy;

		Eigen::Vector3d v2 = Eigen::Vector3d::Zero();
		v2(0, 0) = x1 - x2;
		v2(1, 0) = y1 - y2;

		float angle = acos(v1.dot(v2) / (v1.norm() * v2.norm()));
		std::cout << "angle:" << angle << std::endl;

		float angleToTurn = angle - M_PI_2;
		syncMove(0.0, angleToTurn, 5000);


		/*float A1 = y2 - y1;
	    float B1 = x1 - x2;
	    float C1 = A1 * x1 + B1 * y1;
	    float A2 = secondPy - curry;
	    float B2 = currx - secondPx;
	    float C2 = A2 * currx + B2 * curry;
	    double det = A1 * B2 - A2 * B1;
	    double x = (B2 * C1 - B1 * C2) / det;
	    double y = (A1 * C2 - A2 * C1) / det;

	    Eigen::Vector3d pointIntersect = Eigen::Vector3d::Zero();
	    pointIntersect(0, 0) = x - currx;
	    pointIntersect(1, 0) = y - curry;*/

	    std::cout << "norm:" << x1 - 0.3 << std::endl;
	    if(x1  > 0.3)
			syncMove(x1 - 0.3, 0.0, 5000);

		ss << "I am going to find an object " <<  idObject;
		syncSpeech(ss.str(), 30000, 2000);

		syncMoveHead(0, -0.7854, 5000);
		bool found = syncDetectObjects(recognizedObjects);

		int indexFound = 0;
		for(int i = 0; i < found && recognizedObjects.size(); i++){
			vision_msgs::VisionObject vObject = recognizedObjects[i];
			if(vObject.id.compare(idObject) == 0){
				found = true;
				indexFound = i;
				break;
			}
		}

		ss.str("");
		if(!found || recognizedObjects.size() == 0){
			ss << "I have not found an object " << idObject;
			syncSpeech(ss.str(), 30000, 2000);
			return false;
		}
		
		ss << "I have found an object " << idObject;
		syncSpeech(ss.str(), 30000, 2000);

		pose = recognizedObjects[indexFound].pose;
		std::cout << "Position:" << pose.position.x << "," << pose.position.y << "," << pose.position.z << std::endl;
		std::cout << "Orientation:" << pose.orientation.x << "," << pose.orientation.y << 
			"," << pose.orientation.z << "," << pose.orientation.w << std::endl;

		return true;
	}

	bool obstacleInFront(){
		return JustinaNavigation::obstacleInFront();
	}

	std::vector<tf::Vector3> getPersonLocation(){
		return personLocation;
	}

private:
	ros::Publisher publisFollow;
	ros::ServiceClient cltSpgSay;
	tf::TransformListener * listener;
	std::map<std::string, std::vector<float> > locations;
	std::vector<tf::Vector3> personLocation;
};


enum SMState{
	SM_INIT,
	SM_SAY_WAIT_FOR_DOOR,
	SM_WAIT_FOR_DOOR,
	SM_NAVIGATE_TO_THE_LOCATION,
	SM_SEND_INIT_CLIPS,
	SM_RUN_SM_CLIPS
};

ros::Publisher command_response_pub;
std::string testPrompt;
GPSRTasks tasks;
SMState state;
bool runSMCLIPS = false;
bool startSignalSM = false;
planning_msgs::PlanningCmdClips initMsg;

ros::ServiceClient srvCltGetTasks;
ros::ServiceClient srvCltInterpreter;
ros::ServiceClient srvCltWaitConfirmation;
ros::ServiceClient srvCltWaitForCommand;
ros::ServiceClient srvCltAnswer;

void callbackCmdSpeech(const planning_msgs::PlanningCmdClips::ConstPtr& msg)
{
	std::cout << testPrompt << "--------- Command Speech ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;
	bool success = true;
	startSignalSM = true;
	
	if(!runSMCLIPS)
		success = false;

	success = success & ros::service::waitForService("/planning_clips/wait_command", 50000);
	if(success){
		planning_msgs::planning_cmd srv;
		srv.request.name = "test_wait";
		srv.request.params = "Ready";
		if(srvCltWaitForCommand.call(srv)){
			std::cout << "Response of wait for command:" << std::endl;
			std::cout << "Success:" << (long int)srv.response.success << std::endl;
			std::cout << "Args:" << srv.response.args << std::endl;
		}
		else{
			std::cout << testPrompt << "Failed to call service of wait_command" << std::endl;
			responseMsg.successful = 0;
		}
		responseMsg.params = srv.response.args;
		responseMsg.successful = srv.response.success;
	}
	else{
		if(!runSMCLIPS){
			initMsg = responseMsg;
			return;
		}
		std::cout << testPrompt << "Needed services are not available :'(" << std::endl;
		responseMsg.successful = 0;
	}
	if(runSMCLIPS)
		command_response_pub.publish(responseMsg);
}

void callbackCmdInterpret(const planning_msgs::PlanningCmdClips::ConstPtr& msg)
{
	std::cout << testPrompt << "--------- Command interpreter ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	bool success = ros::service::waitForService("/planning_clips/interpreter", 5000);
	if(success){
		planning_msgs::planning_cmd srv;
		srv.request.name = "test_interprete";
		srv.request.params = "Ready to interpretation";
		if(srvCltInterpreter.call(srv)){
			std::cout << "Response of interpreter:" << std::endl;
			std::cout << "Success:" << (long int)srv.response.success << std::endl;
			std::cout << "Args:" << srv.response.args << std::endl;
			responseMsg.params = srv.response.args;
		responseMsg.successful = srv.response.success;
		}
		else{
			std::cout << testPrompt << "Failed to call service of interpreter" << std::endl;
			responseMsg.successful = 0;
		}
	}
	else{
		std::cout << testPrompt << "Needed services are not available :'(" << std::endl;
		responseMsg.successful = 0;
	}
	command_response_pub.publish(responseMsg);

}

void callbackCmdConfirmation(const planning_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command confirmation ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;
	
	bool success = ros::service::waitForService("spg_say", 5000);
	success = success & ros::service::waitForService("/planning_clips/confirmation", 5000);
	if(success){
		std::string to_spech = responseMsg.params;
		boost::replace_all(to_spech, "_", " ");
		std::stringstream ss;
		ss << "Do you want me " << to_spech;
		std::cout << "------------- to_spech: ------------------ " << ss.str() << std::endl;
		tasks.syncSpeech(ss.str(), 30000, 2000);

		planning_msgs::planning_cmd srv;
		srv.request.name = "test_confirmation";
		srv.request.params = responseMsg.params;
		if(srvCltWaitConfirmation.call(srv)){
			std::cout << "Response of confirmation:" << std::endl;
			std::cout << "Success:" << (long int)srv.response.success << std::endl;
			std::cout << "Args:" << srv.response.args << std::endl;
			if(srv.response.success)
				tasks.syncSpeech("Ok i start to make the command", 30000, 2000);
			else
				tasks.syncSpeech("Repeate the command please", 30000, 2000);

			responseMsg.params = srv.response.args;
			responseMsg.successful = srv.response.success;
		}
		else{
			std::cout << testPrompt << "Failed to call service of confirmation" << std::endl;
			responseMsg.successful = 0;
			tasks.syncSpeech("Repeate the command please", 30000, 2000);
		}
	}
	else{
		std::cout << testPrompt << "Needed services are not available :'(" << std::endl;
		responseMsg.successful = 0;
	}
	command_response_pub.publish(responseMsg);
}

void callbackCmdGetTasks(const planning_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command get tasks ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	bool success = ros::service::waitForService("/planning_clips/get_task" ,5000);
	if(success){
		planning_msgs::planning_cmd srv;
		srv.request.name = "cmd_task";
		srv.request.params = "Test of get_task module";
		if(srvCltGetTasks.call(srv)){
			std::cout << "Response of get tasks:" << std::endl;
			std::cout << "Success:" << (long int)srv.response.success << std::endl;
			std::cout << "Args:" << srv.response.args << std::endl;
			responseMsg.params = srv.response.args;
			responseMsg.successful = srv.response.success;
		}
		else{
			std::cout << testPrompt << "Failed to call get tasks" << std::endl;
			responseMsg.successful = 0;
		}
	}
	else{
		std::cout << testPrompt << "Needed services are not available :'(" << std::endl;
		responseMsg.successful = 0;
	}
	command_response_pub.publish(responseMsg);
}

void callbackCmdNavigation(const planning_msgs::PlanningCmdClips::ConstPtr& msg)
{
	std::cout << testPrompt << "--------- Command Navigation ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	std::vector<std::string> tokens;
	std::string str = responseMsg.params;
	split(tokens, str, is_any_of(" "));

	bool success = true;

	if(tokens[1] == "person"){
		success = true;
	}
	else{
		success = tasks.syncNavigate(tokens[1], 120000);
	}
	if(success)
		responseMsg.successful = 1;
	else
		responseMsg.successful = 0;
	command_response_pub.publish(responseMsg);
}

void callbackCmdAnswer(const planning_msgs::PlanningCmdClips::ConstPtr& msg){

	std::cout << testPrompt << "--------- Command answer a question ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	std::stringstream ss;
	std::vector<std::string> tokens;
	std::string str = responseMsg.params;
	split(tokens, str, is_any_of(" "));

	bool success = ros::service::waitForService("spg_say", 5000);
	if(success){
		std::string param1 = tokens[1];
		if(param1.compare("a_question") == 0){
			success = ros::service::waitForService("/planning_clips/answer", 5000);
			if(success){
				success = tasks.syncSpeech("I am waiting for the user question", 30000, 2000);
				planning_msgs::planning_cmd srv;
				srvCltAnswer.call(srv);
				if(srv.response.success)
					success = tasks.syncSpeech(srv.response.args, 30000, 2000);
				else
					success = false;
			}
		}
		else if(param1.compare("your_name") == 0 ){
			tasks.syncSpeech("Hellow my name is justina", 30000, 2000);
		}
		else if(param1.compare("your_team_name") == 0 || param1.compare("the_name_of_your_team") == 0 ){
			tasks.syncSpeech("Hello my team is pumas", 30000, 2000);
		}
		else if(param1.compare("introduce_yourself") == 0 ){
			tasks.syncSpeech("Hello my name is justina", 30000, 2000);
			tasks.syncSpeech("i am from Mexico city", 30000, 2000);
			tasks.syncSpeech("my team is pumas", 30000, 2000);
			tasks.syncSpeech("of the national autonomous university of mexico", 30000, 2000);
		}
		else if(param1.compare("the_day") == 0 || param1.compare("the_time") == 0){
			ss.str("");
			//std::locale::global(std::locale("de_DE.utf8"));
			//std::locale::global(std::locale("en_us.utf8"));
			time_t now = time(0);
			char* dt = ctime(&now);
		    std::cout << "Day:" << dt << std::endl;
		    tasks.syncSpeech(dt, 30000, 2000);
		}
		else if(param1.compare("what_time_is_it") == 0){
			ss.str("");
			std::time_t now = time(0);
			std::tm *ltm = localtime(&now);
			ss << "The time is " << ltm->tm_hour << " " << ltm->tm_min;
			tasks.syncSpeech(ss.str(), 30000, 2000);
		}	
		else if(param1.compare("what_day_is_tomorrow") == 0){
			std::time_t now = time(0);
			std::tm *ltmnow = localtime(&now);
			std::cout << "Curr day :" << ltmnow->tm_mday << std::endl;
			ltmnow->tm_mday = ltmnow->tm_mday + 1;
			std::cout << "Tomorrow day :" << ltmnow->tm_mday << std::endl;
			std::time_t tomorrow = std::mktime(ltmnow);
			char* dt = ctime(&tomorrow);
			std::cout << "Tomorrow format :" << dt << std::endl;
			tasks.syncSpeech(dt, 30000, 2000);
		}
		else if(param1.compare("the_day_of_the_month") == 0){
			ss.str("");
			//std::locale::global(std::locale("de_DE.utf8"));
			time_t now = time(0);
			char* dt = ctime(&now);
		    std::cout << "Day:" << dt << std::endl;
		    tasks.syncSpeech(dt, 30000, 2000);
		}
	}
	else
		success = false;

	if(success)
		responseMsg.successful = 1;
	else
		responseMsg.successful = 0;
	command_response_pub.publish(responseMsg);
}

void callbackCmdFindObject(const planning_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command find a object ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	std::vector<std::string> tokens;
	std::string str = responseMsg.params;
	split(tokens, str, is_any_of(" "));
	std::stringstream ss;

	bool success = ros::service::waitForService("spg_say" ,5000);
	if(success){
		std::cout << testPrompt << "find: " << tokens[0] << std::endl;
		
		ss.str("");
		if(tokens[0] == "person"){
			success = tasks.findPerson();
			ss << responseMsg.params << " " << 1 << " " << 1 << " " << 1;
		}else if(tokens[0] == "man"){
			success = tasks.findMan(tokens[1]);
			ss << responseMsg.params;
		}
		else if(tokens[0] == "specific"){
			success = tasks.findPerson(tokens[1]);
			ss << responseMsg.params;
		}
		else{
			geometry_msgs::Pose pose;
			success = tasks.findObject(tokens[0], pose);
			ss << responseMsg.params << " " << pose.position.x << " " << pose.position.y 
				<< " " << pose.position.z;
		}
		responseMsg.params = ss.str();
	}
	if(success)
		responseMsg.successful = 1;
	else
		responseMsg.successful = 0;
	command_response_pub.publish(responseMsg);
}


void callbackAskFor(const planning_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command Ask for ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	std::stringstream ss;
	ss << responseMsg.params << " " << "table";
	responseMsg.params = ss.str();
	responseMsg.successful = 1;
	command_response_pub.publish(responseMsg);
}

void callbackStatusObject(const planning_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command Status object ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	std::stringstream ss;
	ss << responseMsg.params << " " << "open";
	responseMsg.params = ss.str();
	responseMsg.successful = 1;
	command_response_pub.publish(responseMsg);
}

void callbackUnknown(const planning_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command unknown ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	responseMsg.successful = 1;
	command_response_pub.publish(responseMsg);
}

int main(int argc, char **argv){

	ros::init(argc, argv, "gpsr_test");
	ros::NodeHandle n;
	ros::Rate rate(10);

	srvCltGetTasks = n.serviceClient<planning_msgs::planning_cmd>("/planning_clips/get_task");
	srvCltInterpreter = n.serviceClient<planning_msgs::planning_cmd>("/planning_clips/interpreter");
	srvCltWaitConfirmation = n.serviceClient<planning_msgs::planning_cmd>("/planning_clips/confirmation");
	srvCltWaitForCommand = n.serviceClient<planning_msgs::planning_cmd>("/planning_clips/wait_command");
	srvCltAnswer = n.serviceClient<planning_msgs::planning_cmd>("/planning_clips/answer");

	ros::Subscriber subCmdSpeech = n.subscribe("/planning_clips/cmd_speech", 1 , callbackCmdSpeech);
	ros::Subscriber subCmdInterpret = n.subscribe("/planning_clips/cmd_int", 1 , callbackCmdInterpret);
	ros::Subscriber subCmdConfirmation = n.subscribe("/planning_clips/cmd_conf", 1, callbackCmdConfirmation);
	ros::Subscriber subCmdGetTasks = n.subscribe("/planning_clips/cmd_task", 1, callbackCmdGetTasks);

	ros::Subscriber subCmdNavigation = n.subscribe("/planning_clips/cmd_goto", 1, callbackCmdNavigation);
	ros::Subscriber subCmdAnswer = n.subscribe("/planning_clips/cmd_answer", 1, callbackCmdAnswer);
	ros::Subscriber subCmdFindObject = n.subscribe("/planning_clips/cmd_find_object", 1, callbackCmdFindObject);
	ros::Subscriber subCmdAskFor = n.subscribe("/planning_clips/cmd_ask_for", 1, callbackAskFor);
	ros::Subscriber subCmdStatusObject = n.subscribe("/planning_clips/cmd_status_object", 1, callbackStatusObject);
	ros::Subscriber subCmdUnknown = n.subscribe("/planning_clips/cmd_unknown", 1, callbackUnknown);

	command_response_pub = n.advertise<planning_msgs::PlanningCmdClips>("/planning_clips/command_response", 1);
	ros::Publisher pubPersonPose = n.advertise<geometry_msgs::PointStamped>("/hri/face_recognizer/face_pose", 1);

	std::string locationsFilePath = "";
    for(int i=0; i < argc; i++){
        std::string strParam(argv[i]);
        if(strParam.compare("-f") == 0)
            locationsFilePath = argv[++i];
    }

	tasks.initRosConnection(&n, locationsFilePath);

	geometry_msgs::PointStamped msgPerson;
	msgPerson.header.frame_id = "map";

	//ros::spin();
	while(ros::ok()){

		switch(state){
			case SM_INIT:
				if(startSignalSM){
					tasks.syncSpeech("I'm ready for the gpsr test", 30000, 2000);
					state = SM_SAY_WAIT_FOR_DOOR;
				}
				break;
			case SM_SAY_WAIT_FOR_DOOR:
				tasks.syncSpeech("I'm waiting for the door to be open", 30000, 2000);
				state = SM_WAIT_FOR_DOOR;
				break;
			case SM_WAIT_FOR_DOOR:
				if(!tasks.obstacleInFront())
					state = SM_NAVIGATE_TO_THE_LOCATION;
				break;
			case SM_NAVIGATE_TO_THE_LOCATION:
				tasks.syncSpeech("I can see now that the door is open", 30000, 2000);
	            std::cout << "GPSRTest.->First try to move" << std::endl;
	            if(!tasks.syncNavigate("arena", 120000)){
	                std::cout << "GPSRTest.->Second try to move" << std::endl;
	                if(!tasks.syncNavigate("arena", 120000)){
	                    std::cout << "GPSRTest.->Third try to move" << std::endl;
	                    if(tasks.syncNavigate("arena", 120000)){
	                    	tasks.syncSpeech("I'm ready for a spoken command", 30000, 2000);
	            			state = SM_SEND_INIT_CLIPS;
	                    }
	                }
	                else{
	                	tasks.syncSpeech("I'm ready for a spoken command", 30000, 2000);
	            		state = SM_SEND_INIT_CLIPS;
	                }
	            }
	            else{
	            	tasks.syncSpeech("I'm ready for a spoken command", 30000, 2000);
	            	state = SM_SEND_INIT_CLIPS;
	            }
				break;
			case SM_SEND_INIT_CLIPS:
				tasks.eneableDisableQR(true);
				initMsg.successful = false;
				runSMCLIPS = true;
				command_response_pub.publish(initMsg);
				state = SM_RUN_SM_CLIPS;
				break;
			case SM_RUN_SM_CLIPS:
				break;
		}

		std::vector<tf::Vector3> personLocation = tasks.getPersonLocation();
		if(personLocation.size() > 0){
			msgPerson.point.x= personLocation[0].x();
            msgPerson.point.y= personLocation[0].y();
            msgPerson.point.z = personLocation[0].z();
            pubPersonPose.publish(msgPerson);
		}

		rate.sleep();
		ros::spinOnce();
	}

	tasks.eneableDisableQR(false);

	return 0;

}
