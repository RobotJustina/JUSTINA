#include "action_planner/service_manager.h"
//#include "vision/vsn_findonplanes.h"
//#include "vision/vsn_personreco.h"


bool ServiceManager::vsnFindByColor(std::string& objectName)
{
    /*
	std::string service_name ("/vsn_findbycolor");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<vision::vsn_findbycolor>(service_name);	//create the service caller

	vision::vsn_findbycolor srv;	//create the service and fill it with the parameters

	if(client.call(srv))	//call the service with the parameters contained in srv
	{
		ROS_DEBUG_STREAM_NAMED("action_planner", service_name << " service called successfully");
		objectName = srv.response.object_name;
		return true;
	}
	else
		ROS_ERROR_STREAM_NAMED("action_planner", "an error acurred when trying to call the " << service_name << " service.");

        */
	return false;

}

/*
*	Implements a synchronous call to the vision node to perform the search of object on a plane
*	Receives:
*		object : the object the robot must find
*		recognized	:	the MarkerArray of the recognized object
*	Returns:
*		true : if the vision module find the object
*		false : otherwise
*/
bool ServiceManager::vsnFindOnPlanes(std_msgs::String object, visualization_msgs::MarkerArray &recognized)
{
    /*
	std::string service_name ("/vsn_findonplanes");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<vision::vsn_findonplanes>(service_name);	//create the service caller

	vision::vsn_findonplanes srv;	//create the service and fill it with the parameters
	srv.request.obj_name = object;

	if(client.call(srv))	//call the service with the parameters contained in srv
	{
		ROS_DEBUG_STREAM_NAMED("action_planner", service_name << " service called successfully with parameters " << srv.request);
		recognized = srv.response.recognized;
		return true;
	}
	else
		ROS_ERROR_STREAM_NAMED("action_planner", "an error acurred when trying to call the " << service_name << " service with parameters " << srv.request);
*/
	return false;
}

	//override by confidence
bool ServiceManager::vsnFindOnPlanes(std_msgs::String object, visualization_msgs::MarkerArray &recognized, std::vector<float> &errors)
{
    /*
	std::string service_name ("/vsn_findonplanes");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<vision::vsn_findonplanes>(service_name);	//create the service caller

	vision::vsn_findonplanes srv;	//create the service and fill it with the parameters
	srv.request.obj_name = object;

	if(client.call(srv))	//call the service with the parameters contained in srv
	{
		ROS_DEBUG_STREAM_NAMED("action_planner", service_name << " service called successfully with parameters " << srv.request);
		recognized = srv.response.recognized;
		errors = srv.response.confidence.data;
		return true;
	}
	else
		ROS_ERROR_STREAM_NAMED("action_planner", "an error acurred when trying to call the " << service_name << " service with parameters " << srv.request);
*/
	return false;
}

/*
*	Implements a synchronous call to the vision node to perform the recognition of a person by its clothes
*	Receives:
*		recognizedPersonName	:	the name of the recognized person (reference)
*	Returns:
*		true : if the vision module find and recognize a person
*		false : otherwise
*/
bool ServiceManager::vsnPersonReco(std::string &recognizedPersonName)
{
    /*
	std::string service_name("/vsn_personreco");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<vision::vsn_personreco>(service_name);	//create the service caller

	vision::vsn_personreco srv;	//create the service and fill it with the parameters

	if(client.call(srv))	//call the service with the parameters contained in srv
	{
		ROS_DEBUG_STREAM_NAMED("action_planner", service_name << " service called successfully.");
		recognizedPersonName = srv.response.person_name;
		return true;
	}
	else
		ROS_ERROR_STREAM_NAMED("action_planner", "an error acurred when trying to call the " << service_name << " service.");
*/
	
	return false;
}
