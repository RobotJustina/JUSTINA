#include "robot_service_manager/navigationtasks.h"
#include "robot_service_manager/speechgeneratortasks.h"
#include "robot_service_manager/robotarmtasks.h"
#include "robot_service_manager/facerecognitiontasks.h"
#include "robot_service_manager/headstatus.h"
#include "robot_service_manager/objectrecotasks.h"
#include "robot_service_manager/speechgeneratortasks.h"

#include "action_planner/states_machines.h"
#include "ros/ros.h"
#include "planning_msgs/PlanningCmdClips.h"
#include "planning_msgs/planning_cmd.h"
#include <std_msgs/Bool.h>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <boost/algorithm/string.hpp>

#include <iostream>

#include <tf/transform_listener.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

using namespace boost::algorithm;

class GPSRTasks{
public:
	GPSRTasks(ros::NodeHandle * n = 0){
		faceTasks.initRosConnection(n);
		headStatus.initRosConnection(n);
		navTasks.initRosConnection(n);
		objDetTasks.initRosConnection(n);
		navStatus.initRosConnection(n);
		//speechTasks.initRosConnection(n);
	}

	void initRosConnection(ros::NodeHandle * n = 0){
		faceTasks.initRosConnection(n);
		headStatus.initRosConnection(n);
		navTasks.initRosConnection(n);
		objDetTasks.initRosConnection(n);
		navStatus.initRosConnection(n);
		publisFollow = n->advertise<std_msgs::Bool>("/hri/human_following/start_follow", 1);
		//speechTasks.initRosConnection(n);
	}

	void waitMoveHead(float goalHeadPan, float goalHeadTile, float timeOut){
		float currHeadPan, currHeadTile;
		float errorPan, errorTile;
		headStatus.setHeadPose(goalHeadPan, goalHeadTile);
		boost::posix_time::ptime curr = boost::posix_time::second_clock::local_time();
		boost::posix_time::ptime prev;
		boost::posix_time::time_duration diff;
		do{
			prev = curr;
			boost::this_thread::sleep(boost::posix_time::milliseconds(100));
			headStatus.getHeadPose(currHeadPan, currHeadTile);
			errorPan = pow(currHeadPan - goalHeadPan, 2);
			errorTile = pow(currHeadTile - goalHeadTile, 2);
			curr = boost::posix_time::second_clock::local_time();
		}while(ros::ok() && errorPan > 0.1 && errorTile > 0.1 && (curr - prev).total_milliseconds() < timeOut);
	}

	std::vector<FaceRecognitionTasks::FaceObject> turnAndRecognizeFace(float angleTurn = M_PI_4, float maxAngleTurn = 2 * M_PI){
		bool faceRecognized;
		std::vector<FaceRecognitionTasks::FaceObject> facesObject;
		float currAngleTurn = 0.0;
		do{
			faceRecognized = faceTasks.recognizeFaces(facesObject, 10000);
			std::cout << "faceRecognized:" << faceRecognized << std::endl;
			if(!faceRecognized){
				navTasks.syncMove(0.0, angleTurn, 50000);
				currAngleTurn += angleTurn;
				boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
			}
		}while(ros::ok() && !faceRecognized && currAngleTurn <= maxAngleTurn);
		return facesObject;
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

	tf::Vector3 transformPoint(tf::StampedTransform transform, tf::Vector3 point){
		return transform * point;
	}

	bool syncNavigate(std::string location, float timeOut){
		std::stringstream ss;
		std::cout << "Navigation to " << location << std::endl;
		ss << "I will navigate to the " << location;
		asyncSpeech(ss.str());
		bool reachedLocation = navTasks.syncGetClose(location, timeOut);
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
		return navTasks.syncGetClose(x, y, timeOut);
	}

	void asyncNavigate(float x, float y){
		navStatus.setGetCloseGoal(x, y);
	}

	bool syncSpeech(std::string textSpeech, float timeOut, float timeSleep){
		bool success = speechTasks.syncSpeech(textSpeech, timeOut);
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
		return success;
	}

	void asyncSpeech(std::string textSpeech){
		speechTasks.asyncSpeech(textSpeech);
	}

	bool findPerson(std::string person = ""){

		waitMoveHead(0, 0, 5000);
		std::vector<int> facesDistances;
		std::stringstream ss;

		std::cout << "Find a person " << person << std::endl;

		ss << "I am going to find a person " << person;
		syncSpeech(ss.str(), 30000, 2000);

		std::vector<FaceRecognitionTasks::FaceObject> facesObject = turnAndRecognizeFace();
		ss.str("");
		if(facesObject.size() == 0){
			ss << "I have not found a person " << person;
			syncSpeech(ss.str(), 30000, 2000);
			return false;
		}

		ss << "I have found a person " << person;
		syncSpeech(ss.str(), 30000, 2000);

		for(int i = 0; i < facesObject.size(); i++){
			std::cout << "Face centroid 3D:" << facesObject[i].faceCentroid.x << "," << facesObject[i].faceCentroid.y << ","
				<< facesObject[i].faceCentroid.z << std::endl;

		}

		tf::StampedTransform transformKinect = getTransform("/map", "/kinect_link");
		std::cout << "Transform kinect_link 3D:" << transformKinect.getOrigin().x() << "," << transformKinect.getOrigin().y() << ","
				<< transformKinect.getOrigin().z() << std::endl;

		float minDistance = 0;
		int indexMin;
		for(int i = 0; i < facesObject.size(); i++){
			tf::Vector3 kinectFaceCentroid(-facesObject[i].faceCentroid.y, facesObject[i].faceCentroid.x, 
					facesObject[i].faceCentroid.z);
			tf::Vector3 worldFaceCentroid = transformPoint(transformKinect, kinectFaceCentroid);
			Eigen::Vector3d worldFaceCentroidEigen;
			tf::vectorTFToEigen(worldFaceCentroid ,worldFaceCentroidEigen);
			Eigen::Vector3d kinectPosition(transformKinect.getOrigin().x(), transformKinect.getOrigin().y(),
				transformKinect.getOrigin().z());
			float mod = (worldFaceCentroidEigen - kinectPosition).norm();
			if(i == 0 || mod < minDistance){
				minDistance = mod;
				indexMin = i;
			}

			std::cout << "Kinect Person 3D:" << worldFaceCentroid.x() << "," << worldFaceCentroid.y() << ","
				<< worldFaceCentroid.z() << std::endl;
		}

		tf::Vector3 nearestKinectPersonCentroid(-facesObject[indexMin].faceCentroid.y, facesObject[indexMin].faceCentroid.x, 
				facesObject[indexMin].faceCentroid.z);
		tf::Vector3 nearestWorldPersonCentroid = transformPoint(transformKinect, nearestKinectPersonCentroid);

		std::cout << "Neareast Person 3D:" << nearestWorldPersonCentroid.x() << "," << nearestWorldPersonCentroid.y() << ","
				<< nearestWorldPersonCentroid.z() << std::endl;


		tf::StampedTransform transRobot = getTransform("/map", "/base_link");
		Eigen::Affine3d transRobotEigen;
		tf::transformTFToEigen(transRobot, transRobotEigen);
		Eigen::AngleAxisd angAxis(transRobotEigen.rotation());
		double currRobotAngle = angAxis.angle();
		Eigen::Vector3d axis = angAxis.axis();
		if(axis(2, 0) < 0.0)
			currRobotAngle = 2 * M_PI - currRobotAngle;

		Eigen::Vector3d robotPostion = transRobotEigen.translation();
		float deltaX = nearestWorldPersonCentroid.x() - robotPostion(0, 0);
    	float deltaY = nearestWorldPersonCentroid.y() - robotPostion(1, 0);
    	float goalRobotAngle = atan2(deltaY, deltaX);
    	if (goalRobotAngle < 0.0f)
        	goalRobotAngle = 2 * M_PI + goalRobotAngle;

		float turn = goalRobotAngle - currRobotAngle;

		syncNavigate(nearestWorldPersonCentroid.x(), nearestWorldPersonCentroid.y(), 20000);
		navTasks.syncMove(0.0, turn, 20000);

		waitMoveHead(0, 0, 5000);

		return true;
	}

	bool findMan(){
		bool found = findPerson();
		if(!found)
			return false;
		std_msgs::Bool msg;
		msg.data = true;
		publisFollow.publish(msg);

		boost::this_thread::sleep(boost::posix_time::milliseconds(20000));	

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

		waitMoveHead(0, -0.7854, 5000);
		bool found = objDetTasks.detectObjects(recognizedObjects);

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
	FaceRecognitionTasks faceTasks;
	HeadStatus headStatus;
	NavigationTasks navTasks;
	NavigationStatus navStatus;
	ObjectRecoTasks objDetTasks;
	SpeechGeneratorTasks speechTasks;

	ros::Publisher publisFollow;
};

class GPSRSM
{
public:
	/*
	*	ADD THE STATES ID YOU NEED FOR EACH STATE FUNCTION (IF YOU WANT)
	*/
	enum States
	{
		InitialState,
		FinalState
	};

	//for the SM api
	StatesMachines SM;
	//stpln
	static ServiceManager srv_man;
	static ros::NodeHandle * nh;
	static std::string classPrompt;
	static ros::Publisher command_response_pub;
	static bool hasBeenInit;

	static ros::Subscriber subCmdSpeech;
	static ros::Subscriber subCmdInterpret;
	static ros::Subscriber subCmdConfirmation;
	static ros::Subscriber subCmdGetTasks;
	static ros::Subscriber subCmdNavigation;
	static ros::Subscriber subCmdAnswer;
	static ros::Subscriber subCmdFindObject;
	static ros::Subscriber subCmdAskFor;
	static ros::Subscriber subCmdStatusObject;
	static ros::Subscriber subCmdUnknown;

	static ros::ServiceClient srvCltInterpreter;
	static ros::ServiceClient srvCltGetTasks;
	static ros::ServiceClient srvCltWaitConfirmation;
	static ros::ServiceClient srvCltWaitForCommand;
	
	static ros::ServiceClient srvCltGetObject;
	static ros::ServiceClient srvCltHandOverObject;
	static ros::ServiceClient srvCltPlaceObject;
	static ros::ServiceClient srvCltFindPerson;
	static ros::ServiceClient srvCltNavigation;
	static ros::ServiceClient srvCltAnswer;

	static NavigationTasks navTasks;
	static RobotArmTasks armTasks;
	static FaceRecognitionTasks faceTasks;
	static GPSRTasks tasks;
	
	static int initialState();
	static int finalState();

	static void callbackCmdSpeech(const planning_msgs::PlanningCmdClips::ConstPtr& msg);
	static void callbackCmdInterpret(const planning_msgs::PlanningCmdClips::ConstPtr& msg);
	static void callbackCmdConfirmation(const planning_msgs::PlanningCmdClips::ConstPtr& msg);
	static void callbackCmdGetTasks(const planning_msgs::PlanningCmdClips::ConstPtr& msg);
	static void callbackCmdNavigation(const planning_msgs::PlanningCmdClips::ConstPtr& msg);
	static void callbackCmdAnswer(const planning_msgs::PlanningCmdClips::ConstPtr& msg);
	static void callbackCmdFindObject(const planning_msgs::PlanningCmdClips::ConstPtr& msg);
	static void callbackAskFor(const planning_msgs::PlanningCmdClips::ConstPtr& msg);
	static void callbackStatusObject(const planning_msgs::PlanningCmdClips::ConstPtr& msg);
	static void callbackUnknown(const planning_msgs::PlanningCmdClips::ConstPtr& msg);
	
	/**********************************************************************/
	
	/*
	* A particular constructor for your state machine
	* Initialize your state machine here (add states, define the final state, define the execution method, etc)
	*/
	GPSRSM(ros::NodeHandle* n);
	bool execute();


private:
	static bool findObject(planning_msgs::PlanningCmdClips cmd);
	static planning_msgs::planning_cmd findPerson();
	static ros::Subscriber subsToRobotPositionSim;


};

ros::NodeHandle * GPSRSM::nh;
std::string GPSRSM::classPrompt;
ros::Publisher GPSRSM::command_response_pub;
ServiceManager GPSRSM::srv_man;
bool GPSRSM::hasBeenInit;
NavigationTasks GPSRSM::navTasks;
RobotArmTasks GPSRSM::armTasks;
FaceRecognitionTasks GPSRSM::faceTasks;
GPSRTasks GPSRSM::tasks;

ros::Subscriber GPSRSM::subCmdSpeech;
ros::Subscriber GPSRSM::subCmdInterpret;
ros::Subscriber GPSRSM::subCmdConfirmation;
ros::Subscriber GPSRSM::subCmdGetTasks;
ros::Subscriber GPSRSM::subCmdNavigation;
ros::Subscriber GPSRSM::subCmdAnswer;
ros::Subscriber GPSRSM::subCmdFindObject;
ros::Subscriber GPSRSM::subCmdAskFor;
ros::Subscriber GPSRSM::subCmdStatusObject;
ros::Subscriber GPSRSM::subCmdUnknown;

ros::ServiceClient GPSRSM::srvCltGetTasks;
ros::ServiceClient GPSRSM::srvCltInterpreter;
ros::ServiceClient GPSRSM::srvCltWaitConfirmation;
ros::ServiceClient GPSRSM::srvCltWaitForCommand;

ros::ServiceClient GPSRSM::srvCltGetObject;
ros::ServiceClient GPSRSM::srvCltHandOverObject;
ros::ServiceClient GPSRSM::srvCltFindPerson;
ros::ServiceClient GPSRSM::srvCltNavigation;
ros::ServiceClient GPSRSM::srvCltAnswer;

ros::ServiceClient GPSRSM::srvCltPlaceObject;

int GPSRSM::initialState()
{
	return (int)InitialState;
}

int GPSRSM::finalState()
{
	std::cout << classPrompt << "FinalState reached" << std::endl;
	ros::service::waitForService("spg_say", 5000); //This service is optional for this test
	srv_man.spgenSay("I have finished the gpsr test", 7000);
	return (int)FinalState;
}

void GPSRSM::callbackCmdSpeech(const planning_msgs::PlanningCmdClips::ConstPtr& msg)
{
	std::cout << classPrompt << "--------- Command Speech ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	bool success = true;
	if(!hasBeenInit){
		success = tasks.syncSpeech("I am ready for a spoken command", 30000, 2000);
		hasBeenInit = true;
	}

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
			std::cout << classPrompt << "Failed to call service of wait_command" << std::endl;
			responseMsg.successful = 0;
		}
		responseMsg.params = srv.response.args;
		responseMsg.successful = srv.response.success;
	}
	else{
		std::cout << classPrompt << "Needed services are not available :'(" << std::endl;
		responseMsg.successful = 0;
	}
	command_response_pub.publish(responseMsg);
}

void GPSRSM::callbackCmdInterpret(const planning_msgs::PlanningCmdClips::ConstPtr& msg)
{
	std::cout << classPrompt << "--------- Command interpreter ---------" << std::endl;
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
			std::cout << classPrompt << "Failed to call service of interpreter" << std::endl;
			responseMsg.successful = 0;
		}
	}
	else{
		std::cout << classPrompt << "Needed services are not available :'(" << std::endl;
		responseMsg.successful = 0;
	}
	command_response_pub.publish(responseMsg);

}

void GPSRSM::callbackCmdConfirmation(const planning_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << classPrompt << "--------- Command confirmation ---------" << std::endl;
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
			std::cout << classPrompt << "Failed to call service of confirmation" << std::endl;
			responseMsg.successful = 0;
			tasks.syncSpeech("Repeate the command please", 30000, 2000);
		}
	}
	else{
		std::cout << classPrompt << "Needed services are not available :'(" << std::endl;
		responseMsg.successful = 0;
	}
	command_response_pub.publish(responseMsg);
}

void GPSRSM::callbackCmdGetTasks(const planning_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << classPrompt << "--------- Command get tasks ---------" << std::endl;
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
			std::cout << classPrompt << "Failed to call get tasks" << std::endl;
			responseMsg.successful = 0;
		}
	}
	else{
		std::cout << classPrompt << "Needed services are not available :'(" << std::endl;
		responseMsg.successful = 0;
	}
	command_response_pub.publish(responseMsg);
}

void GPSRSM::callbackCmdNavigation(const planning_msgs::PlanningCmdClips::ConstPtr& msg)
{
	std::cout << classPrompt << "--------- Command Navigation ---------" << std::endl;
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

void GPSRSM::callbackCmdAnswer(const planning_msgs::PlanningCmdClips::ConstPtr& msg){

	std::cout << classPrompt << "--------- Command answer a question ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	bool success = ros::service::waitForService("spg_say", 5000);
	success = success & ros::service::waitForService("/planning_clips/answer", 5000);
	if(success){
		success = tasks.syncSpeech("I am waiting for the user question", 30000, 2000);
		planning_msgs::planning_cmd srv;
		srvCltAnswer.call(srv);
		if(srv.response.success)
			success = tasks.syncSpeech(srv.response.args, 30000, 2000);
		else
			success = false;
	}
	else
		success = false;

	if(success)
		responseMsg.successful = 1;
	else
		responseMsg.successful = 0;
	command_response_pub.publish(responseMsg);
}

void GPSRSM::callbackCmdFindObject(const planning_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << classPrompt << "--------- Command find a object ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	std::vector<std::string> tokens;
	std::string str = responseMsg.params;
	ros::Rate rate(1);
	split(tokens, str, is_any_of(" "));
	std::stringstream ss;

	bool success = ros::service::waitForService("spg_say" ,5000);
	if(success){
		std::cout << classPrompt << "find: " << tokens[0] << std::endl;
		
		ss.str("");
		if(tokens[0] == "person"){
			success = tasks.findPerson();
			ss << responseMsg.params << " " << 0 << " " << 0 << " " << 0;
		}else if(tokens[0] == "man"){
			success = tasks.findMan();
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


void GPSRSM::callbackAskFor(const planning_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << classPrompt << "--------- Command Ask for ---------" << std::endl;
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

void GPSRSM::callbackStatusObject(const planning_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << classPrompt << "--------- Command Status object ---------" << std::endl;
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

void GPSRSM::callbackUnknown(const planning_msgs::PlanningCmdClips::ConstPtr& msg){
	std::cout << classPrompt << "--------- Command unknown ---------" << std::endl;
	std::cout << "name:" << msg->name << std::endl;
	std::cout << "params:" << msg->params << std::endl;

	planning_msgs::PlanningCmdClips responseMsg;
	responseMsg.name = msg->name;
	responseMsg.params = msg->params;
	responseMsg.id = msg->id;

	responseMsg.successful = 1;
	command_response_pub.publish(responseMsg);
}

GPSRSM::GPSRSM(ros::NodeHandle* n)
{
	nh = n;
	//add states to the state machine
	SM.addState((int)InitialState, &initialState);
	SM.addState((int)FinalState, &finalState, true);

	navTasks.initRosConnection(n);
	armTasks.initRosConnection(n);
	faceTasks.initRosConnection(n);
	tasks.initRosConnection(n);

	srvCltGetTasks = n->serviceClient<planning_msgs::planning_cmd>("/planning_clips/get_task");
	srvCltInterpreter = n->serviceClient<planning_msgs::planning_cmd>("/planning_clips/interpreter");
	srvCltWaitConfirmation = n->serviceClient<planning_msgs::planning_cmd>("/planning_clips/confirmation");
	srvCltWaitForCommand = n->serviceClient<planning_msgs::planning_cmd>("/planning_clips/wait_command");
	srvCltAnswer = n->serviceClient<planning_msgs::planning_cmd>("/planning_clips/answer");

	//srvCltPlaceObject = n->serviceClient<planning_msgs::wait_for_place_object>("/simple_task_planner/wait_for_place_object");

	subCmdSpeech = n->subscribe("/planning_clips/cmd_speech", 1 , GPSRSM::callbackCmdSpeech);
	subCmdInterpret = n->subscribe("/planning_clips/cmd_int", 1 , GPSRSM::callbackCmdInterpret);
	subCmdConfirmation = n->subscribe("/planning_clips/cmd_conf", 1, GPSRSM::callbackCmdConfirmation);
	subCmdGetTasks = n->subscribe("/planning_clips/cmd_task", 1, GPSRSM::callbackCmdGetTasks);

	subCmdNavigation = n->subscribe("/planning_clips/cmd_goto", 1, GPSRSM::callbackCmdNavigation);
	subCmdAnswer = n->subscribe("/planning_clips/cmd_answer", 1, GPSRSM::callbackCmdAnswer);
	subCmdFindObject = n->subscribe("/planning_clips/cmd_find_object", 1, GPSRSM::callbackCmdFindObject);
	subCmdAskFor = n->subscribe("/planning_clips/cmd_ask_for", 1, GPSRSM::callbackAskFor);
	subCmdStatusObject = n->subscribe("/planning_clips/cmd_status_object", 1, GPSRSM::callbackStatusObject);
	subCmdUnknown = n->subscribe("/planning_clips/cmd_unknown", 1, GPSRSM::callbackUnknown);

	command_response_pub = n->advertise<planning_msgs::PlanningCmdClips>("/planning_clips/command_response", 1);
	hasBeenInit = false;

}

bool GPSRSM::execute()
{
	ros::Rate loop(10);
	while(SM.runNextStep() && ros::ok())
	{
		loop.sleep();
		ros::spinOnce();
	}
	return true;
}