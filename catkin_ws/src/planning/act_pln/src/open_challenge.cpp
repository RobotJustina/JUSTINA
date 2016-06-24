
#include "ros/ros.h"

#include "planning_msgs/PlanningCmdClips.h"
#include "planning_msgs/planning_cmd.h"

#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"

#include <vector>

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
		tf::TransformListener listener;
		bool updateTransform = false;
		tf::StampedTransform transform;
		do{
			try{
				listener.lookupTransform(frame1, frame2,  
	                             	ros::Time(0), transform);
				updateTransform = true;
			}
			catch(tf::TransformException ex){
				std::cerr << "error:" << ex.what() << std::endl;
				ros::Duration(1.0).sleep();
			}
		}while(ros::ok() && !updateTransform);
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
		std::stringstream ss;
		if(angle != 0){
			ss << "I am Turn";
			syncSpeech(ss.str(), 30000, 2000);
		}
		if(distance != 0){
			ss.str("");
			ss << "I am Advance";
			syncSpeech(ss.str(), 30000, 2000);
		}
		return JustinaNavigation::moveDistAngle(distance, angle, timeOut);
	}

	void waitHeadGoalPose(float goalHeadPan, float goalHeadTile, float timeOut){
		float currHeadPan, currHeadTile;
		float errorPan, errorTile;
		boost::posix_time::ptime curr;
		boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
		boost::posix_time::time_duration diff;
		do{
			boost::this_thread::sleep(boost::posix_time::milliseconds(100));
			JustinaHardware::getHeadCurrentPose(currHeadPan, currHeadTile);
			errorPan = pow(currHeadPan - goalHeadPan, 2);
			errorTile = pow(currHeadTile - goalHeadTile, 2);
			curr = boost::posix_time::second_clock::local_time();
		}while(ros::ok() && errorPan > 0.1 && errorTile > 0.1 && (curr - prev).total_milliseconds() < timeOut);
	}

	void syncMoveHead(float goalHeadPan, float goalHeadTile, float timeOut){
		JustinaHardware::setHeadGoalPose(goalHeadPan, goalHeadTile);
		waitHeadGoalPose(goalHeadPan, goalHeadTile, timeOut);
	}

	void asyncMoveHead(float goalHeadPan, float goalHeadTile){
		JustinaHardware::setHeadGoalPose(goalHeadPan, goalHeadTile);
	}

	std::vector<vision_msgs::VisionFaceObject> waitRecognizeFace(float timeOut, std::string id, bool &recognized){
		boost::posix_time::ptime curr;
		boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
		boost::posix_time::time_duration diff;
		std::vector<vision_msgs::VisionFaceObject> lastRecognizedFaces;
		do{
			boost::this_thread::sleep(boost::posix_time::milliseconds(100));
			if(id.compare("") == 0)
				JustinaVision::facRecognize();
			else
				JustinaVision::facRecognize(id);
			JustinaVision::getLastRecognizedFaces(lastRecognizedFaces);
			curr = boost::posix_time::second_clock::local_time();
			ros::spinOnce();
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
		
		asyncMoveHead(initAngPan, 0.0);
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
				boost::this_thread::sleep(boost::posix_time::milliseconds(500));
				std::vector<vision_msgs::VisionFaceObject> facesObject = waitRecognizeFace(1000, id, recog);
				if(continueReco)
					centroidFace = filterRecognizeFace(facesObject, 3.0, recog);
				boost::this_thread::sleep(boost::posix_time::milliseconds(500));
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

		syncMoveHead(0, 0, 5000);
		std::vector<int> facesDistances;
		std::stringstream ss;

		std::cout << "Find a person " << person << std::endl;

		ss << "I am going to find a person " << person;
		syncSpeech(ss.str(), 30000, 2000);

		bool recog;
		JustinaVision::startFaceRecognition();
		Eigen::Vector3d centroidFace = turnAndRecognizeFace(person, -M_PI_4, M_PI_4, M_PI_4, M_PI_2, 2 * M_PI, recog);
		JustinaVision::stopFaceRecognition();
		std::cout << "recog:" << recog << std::endl;

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

		tf::StampedTransform transformKinect = getTransform("/map", "/kinect_link");
		std::cout << "Transform kinect_link 3D:" << transformKinect.getOrigin().x() << "," << transformKinect.getOrigin().y() << ","
				<< transformKinect.getOrigin().z() << std::endl;

		tf::Vector3 kinectFaceCentroid(-centroidFace(1, 0), centroidFace(0, 0), 
					centroidFace(2, 0));
		tf::Vector3 worldFaceCentroid = transformPoint(transformKinect, kinectFaceCentroid);
		std::cout << "Kinect Person 3D:" << worldFaceCentroid.x() << "," << worldFaceCentroid.y() << ","
				<< worldFaceCentroid.z() << std::endl;

		tf::StampedTransform transRobot = getTransform("/map", "/base_link");
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
		float turn = goalRobotAngle - currRobotAngle;

		std::cout << "turn:" << turn << "distance:" << distance << std::endl;
		syncMove(0.0, turn, 10000);
		syncMove(distance - 0.9, 0.0, 10000);

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
		ss << "I have finish follow a person " << std::endl;
		asyncSpeech(ss.str());

		msg.data = false;
		publisFollow.publish(msg);
		return true;

	}

	bool findObject(std::string idObject, geometry_msgs::Pose & pose){
		std::vector<vision_msgs::VisionObject> recognizedObjects;

		std::cout << "Find a object " << idObject << std::endl;

		std::stringstream ss;
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

private:
	ros::Publisher publisFollow;
	ros::ServiceClient cltSpgSay;
	std::map<std::string, std::vector<float> > locations;
};

ros::Publisher command_response_pub;
std::string testPrompt;
bool hasBeenInit;
GPSRTasks tasks;

ros::ServiceClient srvCltGetTasks;
ros::ServiceClient srvCltInterpreter;
ros::ServiceClient srvCltWaitConfirmation;
ros::ServiceClient srvCltWaitForCommand;
ros::ServiceClient srvCltAnswer;
ros::ServiceClient srvCltWhatSee;

void callbackCmdWorld(const planning_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << testPrompt << "--------- Command World ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;
	
	int robert = 0;
	int other = 0;
	int robertCI = 0;
	int robertCD = 0;
	int otherCI = 0;
	int otherCD = 0;
	int arthur = 0;
	int arthurCI  = 0;
	int arthurCD  = 0;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	planning_msgs::PlanningCmdClips responseModify;
	responseModify.name = "cmd_modify";
	responseModify.params = "modify";
	responseModify.id = msg->id;

	std::stringstream ss;

	//responseMsg.successful = 1;
	//command_response_pub.publish(responseMsg);

	
	
	//bool success = ros::service::waitForService("spg_say", 5000);
	bool success = ros::service::waitForService("/planning_open_challenge/what_see", 5000);
	if(success){

		planning_msgs::planning_cmd srv;
		srv.request.name = "test_what_see";
		srv.request.params = responseMsg.params;
		if(srvCltWhatSee.call(srv)){
			JustinaVision::startFaceRecognition();
			bool recognized = false;
			float timeOut = 25000.0;
			std::vector<vision_msgs::VisionFaceObject> lastRecognizedFaces;

			boost::posix_time::ptime curr;
			boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
			boost::posix_time::time_duration diff;
			
			std::cout << "Response of what do you see:" << std::endl;
			std::cout << "Success:" << (long int)srv.response.success << std::endl;
			std::cout << "Args:" << srv.response.args << std::endl;


			do{
			boost::this_thread::sleep(boost::posix_time::milliseconds(100));
			JustinaVision::facRecognize();
			JustinaVision::getLastRecognizedFaces(lastRecognizedFaces);
			
			for(int i=0; i<lastRecognizedFaces.size(); i++)
			{
				if(lastRecognizedFaces[i].id == "robert")
				{
					robert++;
					if(i == 0)
					{
						robertCI++;
					}
					else
					{
						robertCD++;
					}
				}
				else if(lastRecognizedFaces[i].id == "arthur")
				{
					arthur++;
					if(i == 0)
					{
						arthurCI++;
					}
					else
					{
						arthurCD++;
					}
				}
				else if(lastRecognizedFaces[i].id == "unknown")
				{
					other++;
					if(i == 0)
					{
						otherCI++;
					}
					else
					{
						otherCD++;
					}
				}

				
				
			}

			curr = boost::posix_time::second_clock::local_time();
			ros::spinOnce();
			}while(ros::ok() && (curr - prev).total_milliseconds()< timeOut && srv.response.args == "what_see_yes");

			if(arthurCI != arthurCD && arthurCI > arthurCD)
			{
				std::cout << "Arthur esta a la Izquerda" << std::endl;
				ss << "arthur izquierda";
				tasks.syncSpeech("Arthur is in the left", 30000, 2000);
			}
			else if(arthurCI != arthurCD && arthurCI < arthurCD)
			{
				std::cout << "Arthur esta a la Derecha" << std::endl;
				ss << "arthur derecha";
				tasks.syncSpeech("Arthur is in the right", 30000, 2000);
			}
			else
			{
				if(arthur>0){
					std::cout << "Arthur esta SOLO" << std::endl;
					ss << "arthur solo";
					tasks.syncSpeech("Arthur is the only person I can see", 30000, 2000);
				}
				else
				{
					ss << "arthur nil";
				}
			}

			if(robertCI != robertCD && robertCI > robertCD)
			{
				std::cout << "Robert esta a la Izquerda" << std::endl;
				ss << " robert izquierda";
				tasks.syncSpeech("Robert is in the left", 30000, 2000);
			}
			else if(robertCI != robertCD && robertCI < robertCD)
			{
				std::cout << "Robert esta a la Derecha" << std::endl;
				ss << " robert derecha";
				tasks.syncSpeech("Robert is in the right", 30000, 2000);
			}
			else
			{
				if(robert>0)
				{
					std::cout << "Robert esta SOLO" << std::endl;
					ss << " robert solo";
					tasks.syncSpeech("Robert is the only person I can see", 30000, 2000);
				}
				else
				{
					ss << " robert nil";
				}
			}
			
			
			
			std::string s = ss.str();
			responseModify.params = s;
			responseModify.successful = 1;
			if(srv.response.args == "what_see_yes")
				command_response_pub.publish(responseModify);
			

			//std::cout << "Vector: " << lastRecognizedFaces[0].id << std::endl;
			std::cout << "Robert times: " << robert << std::endl;
			std::cout << "arthur times: " << arthur << std::endl;
			
			//std::cout << "unknown times: " << other << std::endl;
			
			std::cout << "RobertIzquierda times: " << robertCI << std::endl;
			//std::cout << "unknowmIzquierda times: " << otherCI << std::endl;
			std::cout << "ArthurIzquierda times: " << arthurCI << std::endl;

			std::cout << "RobertDerecha times: " << robertCD << std::endl;
			//std::cout << "unknownDerecha times: " << otherCD << std::endl;
			std::cout << "ArthurDerecha times: " << arthurCD << std::endl;
			
			/*if(lastRecognizedFaces.size()>0)
				recognized = true;
			else
				recognized = false;*/
			
			
			//command_response_pub.publish(responseMsg);
			JustinaVision::stopFaceRecognition();
			responseMsg.params = srv.response.args;
			responseMsg.successful = srv.response.success;

			
		}
		else{
			std::cout << testPrompt << "Failed to call service what do you see" << std::endl;
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

void callbackCmdDescribe(const planning_msgs::PlanningCmdClips::ConstPtr& msg)
{
	std::cout << testPrompt << "--------- Command Describe ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseDescribe;
	responseDescribe.name = "cmd_world";
	responseDescribe.params =  "what_see_yes";
	responseDescribe.id = msg->id;
	responseDescribe.successful = 1;

	std::vector<std::string> tokens;
	std::string str = msg->params;
	split(tokens, str, is_any_of(" "));

	if(tokens[3] != "nil" && tokens[1] != "nil")
	{
		tasks.syncSpeech("There are no persons in the room", 30000, 2000);
		std::cout << "There are no persons in the room" << std::endl;
	}
	else{
		if(tokens[3] != "nil" && tokens[3] != "solo"){
			if(tokens[1] == "derecha")
				tasks.syncSpeech("Robert is in the left of arthur", 30000, 2000);
			if(tokens[1] == "izquerda")
				tasks.syncSpeech("Robert is in the right of arthur", 30000, 2000);
		
		}
		if(tokens[3] == "nil")
			tasks.syncSpeech("Robert is the only person in the room", 30000, 2000);

		if(tokens[1] != "nil" && tokens[1] != "solo"){
			if(tokens[3] == "derecha")
				tasks.syncSpeech("Arthur is in the left of robert", 30000, 2000);
			if(tokens[3] == "izquerda")
				tasks.syncSpeech("Arthur is in the right of robert", 30000, 2000);
		}
		if(tokens[1] == "nil")
			tasks.syncSpeech("Arthur is the only person in the room", 30000, 2000);
	}
	//std::stringstream ss;
	command_response_pub.publish(responseDescribe);
}


int main(int argc, char **argv){

	ros::init(argc, argv, "open_challenge_test");
	ros::NodeHandle n;
	
	srvCltWhatSee = n.serviceClient<planning_msgs::planning_cmd>("/planning_open_challenge/what_see");
	ros::Subscriber subCmdWorld = n.subscribe("/planning_open_challenge/cmd_world", 1 , callbackCmdWorld);
	ros::Subscriber subCmdDescribe = n.subscribe("/planning_open_challenge/cmd_describe", 1 , callbackCmdDescribe);

	command_response_pub = n.advertise<planning_msgs::PlanningCmdClips>("/planning_open_challenge/command_response", 1);

	std::string locationsFilePath = "";
    for(int i=0; i < argc; i++){
        std::string strParam(argv[i]);
        if(strParam.compare("-f") == 0)
            locationsFilePath = argv[++i];
    }

	tasks.initRosConnection(&n, locationsFilePath);

	ros::spin();

	return 0;

}
