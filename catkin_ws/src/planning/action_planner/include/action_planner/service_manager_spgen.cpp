#include "action_planner/service_manager.h"
#include "bbros_bridge/Default_ROS_BB_Bridge.h"

/*
*	Implements an asynchronous call to the voice generator to play a voice message
*	Receives:
*		text_to_synthesize : the message to play
*/
void ServiceManager::spgenAsay(std::string text_to_synthesize)
{
	std::string service_name ("/spg_asay");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<bbros_bridge::Default_ROS_BB_Bridge>(service_name);	//create the service caller

	bbros_bridge::Default_ROS_BB_Bridge srv;	//create the service and fill it with the parameters
	srv.request.parameters = text_to_synthesize;
	srv.request.timeout = 0;

	if(client.call(srv))	//call the service with the parameters contained in srv
	{
		ROS_DEBUG_STREAM_NAMED("action_planner", service_name << " service called successfully with parameters: " << text_to_synthesize);
	}
	else
	{
		ROS_ERROR_STREAM_NAMED("action_planner", "an error acurred when trying to call the " << service_name << " service with parameters: " << text_to_synthesize);
	}
}

/*
*	Implements a synchronous call to the voice generator to play a voice message
*	Receives:
*		text_to_synthesize : the message to play
*		timeout : the timeout time
*	Return:
*		true : if the message was played
*		false : otherwise
*/
bool ServiceManager::spgenSay(std::string text_to_synthesize, int timeout)
{
	std::string service_name ("/spg_say");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<bbros_bridge::Default_ROS_BB_Bridge>(service_name);	//create the service caller

	bbros_bridge::Default_ROS_BB_Bridge srv;	//create the service and fill it with the parameters
	srv.request.parameters = text_to_synthesize;
	srv.request.timeout = timeout;

	if(client.call(srv))	//call the service with the parameters contained in srv
	{
		ROS_DEBUG_STREAM_NAMED("action_planner", service_name << " service called successfully with parameters: " << text_to_synthesize);
		return true;
	}
	else
	{
		ROS_ERROR_STREAM_NAMED("action_planner", "an error acurred when trying to call the " << service_name << " service with parameters: " << text_to_synthesize);
	}
	return false;
}
