#include "action_planner/service_manager.h"
#include "bbros_bridge/Default_ROS_BB_Bridge.h"

/*
* Implements a call to the prsfnd BB module to perform the pf_find (find a human) command
* Receives:
*	personToFind	:	a string containing the name of the human to find
*	timeout	:	timeout for the bb command
*/
bool ServiceManager::prsfndFind(std::string personToFind, int timeout)
{
	std::string service_name ("/pf_find");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<bbros_bridge::Default_ROS_BB_Bridge>(service_name);	//create the service caller

	bbros_bridge::Default_ROS_BB_Bridge srv;	//create the service and fill it with the parameters
	srv.request.parameters = personToFind;
	srv.request.timeout = timeout;

	if(client.call(srv))	//call the service with the parameters contained in srv
	{
		ROS_DEBUG_STREAM_NAMED("action_planner", service_name << " service called successfully with parameters " << srv.request);
		return srv.response.success;
	}
	else
	{
		ROS_ERROR_STREAM_NAMED("action_planner", "an error acurred when trying to call the " << service_name << " service with parameters" << srv.request);
	}
	return false;
}

/*
* Implements a call to the prsfnd BB module to perform the pf_remember (remember a human) command
* Receives:
*	humanName	:	a string containing the name of the human to remember
*	timeout	:	timeout for the bb command
*/
bool ServiceManager::prsfndRemember(std::string humanName, int timeout)
{
	std::string service_name ("/pf_remember");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<bbros_bridge::Default_ROS_BB_Bridge>(service_name);	//create the service caller

	bbros_bridge::Default_ROS_BB_Bridge srv;	//create the service and fill it with the parameters
	srv.request.parameters = humanName;
	srv.request.timeout = timeout;

	if(client.call(srv))	//call the service with the parameters contained in srv
	{
		ROS_DEBUG_STREAM_NAMED("action_planner", service_name << " service called successfully with parameters " << srv.request);
		return srv.response.success;
	}
	else
	{
		ROS_ERROR_STREAM_NAMED("action_planner", "an error acurred when trying to call the " << service_name << " service with parameters" << srv.request);
	}
	return false;
}

/*
* Implements a call to the prsfnd BB module to perform the pf_amnesia (clear the prsfnd database) command
* Receives:
*	timeout	:	timeout for the bb command
*/
bool ServiceManager::prsfndAmnesia(int timeout)
{
	std::string service_name ("/pf_amnesia");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<bbros_bridge::Default_ROS_BB_Bridge>(service_name);	//create the service caller

	bbros_bridge::Default_ROS_BB_Bridge srv;	//create the service and fill it with the parameters
	srv.request.parameters = "";
	srv.request.timeout = timeout;

	if(client.call(srv))	//call the service with the parameters contained in srv
	{
		ROS_DEBUG_STREAM_NAMED("action_planner", service_name << " service called successfully with parameters " << srv.request);
		return srv.response.success;
	}
	else
	{
		ROS_ERROR_STREAM_NAMED("action_planner", "an error acurred when trying to call the " << service_name << " service with parameters" << srv.request);
	}
	return false;
}
