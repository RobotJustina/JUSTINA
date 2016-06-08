#include "action_planner/service_manager.h"
#include "head/hd_lookat.h"
#include "head/hd_torque.h"


/*
*	Implements a synchronous call to the head node to move the robots head a specified (pan, tilt) position
*	Receives:
*		goalPan : the goal pan position
*		goalTilt : the goal tilt position
*		currentTilt : the final tilt of the robots head after perform the command
*		currentPan : the final pan of the robots head after perform the command
*	Returns:
*		true : if the robot performs correctly the action
*		false : otherwise
*/
bool ServiceManager::hdLookAt(std_msgs::Float32 goalPan, std_msgs::Float32 goalTilt, std_msgs::Float32 &currentPan, std_msgs::Float32 &currentTilt)
{
	std::string service_name ("/hd_lookat");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<head::hd_lookat>(service_name);	//create the service caller

	head::hd_lookat srv;	//create the service and fill it with the parameters
	srv.request.goalPan = goalPan;
	srv.request.goalTilt = goalTilt;

	if(client.call(srv))	//call the service with the parameters contained in srv
	{
		ROS_DEBUG_STREAM_NAMED("action_planner", service_name << " service called successfully with parameters " << srv.request);
		currentPan = srv.response.currentPan;
		currentTilt = srv.response.currentTilt;
		return true;
	}
	else
	{
		ROS_ERROR_STREAM_NAMED("action_planner", "an error acurred when trying to call the " << service_name << " service with parameters" << srv.request);
		ROS_ERROR_STREAM_NAMED("action_planner", "Error message received from " << service_name << " : " << srv.response.errors);
	}
	return false;
}

/*
*	Implements a synchronous call to the head node to enable/disable the head torque
*	Receives:
*		enable : true if you want to enable the head torque, false otherwise
*	Returns:
*		true : if the robot performs correctly the action
*		false : otherwise
*/
bool ServiceManager::hdTorque(std_msgs::Bool enable)
{
	std::string service_name ("/hd_torque");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<head::hd_torque>(service_name);	//create the service caller

	head::hd_torque srv;	//create the service and fill it with the parameters
	srv.request.enable = enable;

	if(client.call(srv))	//call the service with the parameters contained in srv
	{
		ROS_DEBUG_STREAM_NAMED("action_planner", service_name << " service called successfully with parameters " << srv.request);
		return true;
	}
	else
	{
		ROS_ERROR_STREAM_NAMED("action_planner", "an error acurred when trying to call the " << service_name << " service with parameters" << srv.request);
		ROS_ERROR_STREAM_NAMED("action_planner", "Error message received from " << service_name << " : " << srv.response.errors);
	}
	return false;
}
