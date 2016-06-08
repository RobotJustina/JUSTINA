#include "action_planner/service_manager.h"
#include "arms/arm_opengrip.h"
#include "arms/arm_closegrip.h"
#include "arms/arm_abspos_xyzrpye.h"
#include "arms/arm_goto.h"

/*
*	Implements a synchronous call to the arms node to open a robot hand
*	Receives:
*		side : to indicate which side of the robot must perform the action
*		position : the final open position of the hand
*	Return:
*		true : if the robot hand was open
*		false : otherwise
*/
bool ServiceManager::armsOpenGrip(RobotKnowledge::ARM_SIDE side, std_msgs::Float32 position)
{
	std::string service_name;

	switch(side)
	{
		case RobotKnowledge::LeftArm:
			service_name = "/la_opengrip";
			break;
		case RobotKnowledge::RightArm:
			service_name = "/ra_opengrip";
			break;
		default:
			service_name = "/la_opengrip";
			break;
	}

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<arms::arm_opengrip>(service_name);	//create the service caller

	arms::arm_opengrip srv;	//create the service and fill it with the parameters
	srv.request.position = position;

	if(client.call(srv))	//call the service with the parameters contained in srv
	{
		ROS_DEBUG_STREAM_NAMED("action_planner", service_name << " service called successfully with parameters: " << position);
		return true;
	}
	else
	{
		ROS_ERROR_STREAM_NAMED("action_planner", "an error acurred when trying to call the " << service_name << " service with parameters: " << position);
	}
	return false;
}

/*
*	Implements a synchronous call to the arms node to close a robot hand
*	Receives:
*		side : to indicate which side of the robot must perform the action
*		torque : indicates the torque of the closing action
*	Return:
*		true : if the robot hand was closed
*		false : otherwise
*/
bool ServiceManager::armsCloseGrip(RobotKnowledge::ARM_SIDE side, std_msgs::Float32 torque)
{
	std::string service_name;

	switch(side)
	{
		case RobotKnowledge::LeftArm:
			service_name = "/la_closegrip";
			break;
		case RobotKnowledge::RightArm:
			service_name = "/ra_closegrip";
			break;
		default:
			service_name = "/la_closegrip";
			break;
	}

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<arms::arm_closegrip>(service_name);	//create the service caller

	arms::arm_closegrip srv;	//create the service and fill it with the parameters
	srv.request.torque = torque;

	if(client.call(srv))	//call the service with the parameters contained in srv
	{
		ROS_DEBUG_STREAM_NAMED("action_planner", service_name << " service called successfully with parameters: " << torque);
		return true;
	}
	else
	{
		ROS_ERROR_STREAM_NAMED("action_planner", "an error acurred when trying to call the " << service_name << " service with parameters: " << torque);
	}
	return false;
}

/*
*	Implements a synchronous call to the arms node to move a robot arm to a (x, y, z, roll, pitch, yaw, elbow) position
*	Receives:
*		x : the x goal position
*		y : the y goal position
*		z	:	the z goal position
*		roll	:	the roll goal position
*		pitch	:	the putch goal position
*		yaw	:	the yaw goal position
*		elbow	:	the elbow goal position
*	Return:
*		true : if the robot arm reach the location
*		false : otherwise
*/
bool ServiceManager::armsAbsPos(RobotKnowledge::ARM_SIDE side, std_msgs::Float32 x, std_msgs::Float32 y,
	std_msgs::Float32 z, std_msgs::Float32 roll, std_msgs::Float32 pitch, std_msgs::Float32 yaw, std_msgs::Float32 elbow)
{
	std::string service_name;

	switch(side)
	{
		case RobotKnowledge::LeftArm:
			service_name = "/la_abspos_xyzrpye";
			break;
		case RobotKnowledge::RightArm:
			service_name = "/ra_abspos_xyzrpye";
			break;
		default:
			service_name = "/la_abspos_xyzrpye";
			break;
	}

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<arms::arm_abspos_xyzrpye>(service_name);	//create the service caller

	arms::arm_abspos_xyzrpye srv;	//create the service and fill it with the parameters
	srv.request.goalX = x;
	srv.request.goalY = y;
	srv.request.goalZ = z;
	srv.request.goalRoll = roll;
	srv.request.goalPitch = pitch;
	srv.request.goalYaw = yaw;
	srv.request.goalElbow = elbow;

	if(client.call(srv))	//call the service with the parameters contained in srv
	{
		ROS_DEBUG_STREAM_NAMED("action_planner", service_name << " service called successfully with parameters: " << srv.request);
		return true;
	}
	else
	{
		ROS_ERROR_STREAM_NAMED("action_planner", "an error acurred when trying to call the " << service_name << " service with parameters: " << srv.request);
		ROS_ERROR_STREAM_NAMED("action_planner", "Error message received from " << service_name << " : " << srv.response.errors);
	}
	return false;
}

/*
*	Implements a synchronous call to the arms node to move a robot arm to a string predef position
*	Receives:
*		location : the location that the robot arm must reach
*	Return:
*		true : if the robot arm reach the location
*		false : otherwise
*/
bool ServiceManager::armsGoTo(RobotKnowledge::ARM_SIDE side, std_msgs::String location)
{
	std::string service_name;

	switch(side)
	{
		case RobotKnowledge::LeftArm:
			service_name = "/la_goto";
			break;
		case RobotKnowledge::RightArm:
			service_name = "/ra_goto";
			break;
		default:
			service_name = "/la_goto";
			break;
	}

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<arms::arm_goto>(service_name);	//create the service caller

	arms::arm_goto srv;	//create the service and fill it with the parameters
	srv.request.location = location;

	if(client.call(srv))	//call the service with the parameters contained in srv
	{
		ROS_DEBUG_STREAM_NAMED("action_planner", service_name << " service called successfully with parameters: " << srv.request);
		return true;
	}
	else
	{
		ROS_ERROR_STREAM_NAMED("action_planner", "an error acurred when trying to call the " << service_name << " service with parameters: " << srv.request);
		ROS_ERROR_STREAM_NAMED("action_planner", "Error message received from " << service_name << " : " << srv.response.errors);
	}
	return false;
}
