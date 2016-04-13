#include <iostream>
#include "ros/ros.h"
#include "manip_msgs/InverseKinematicsFloatArray.h"
#include "manip_msgs/InverseKinematicsPath.h"
#include "manip_msgs/InverseKinematicsPose.h"
#include "InverseKinematics.h"

void printHelp()
{
    std::cout << "MANIPULATION SIMPLE MOVEMENTS BY MARCOSOFT..." << std::endl;
    std::cout << " - This node calculates the inverse kinematics and executes a" << std::endl;
    std::cout << " - low level control for moving arm to predefined positions." << std::endl;
    std::cout << " - Inverse kinematics is calculated considering a 7DOF manipulator" << std::endl;
    std::cout << " - with all joints of revolute type and with a spheric wrist" << std::endl;
    std::cout << " - In cartesian coords, the seven values are x, y, z, roll, pitch, yaw and elbow. " << std::endl;
    std::cout << "PLEASE DON'T TRY TO OPERATE JUSTINA IF YOU ARE NOT QUALIFIED ENOUGH" << std::endl;
}

bool callbackInverseKinematicsFloatArray(manip_msgs::InverseKinematicsFloatArray::Request &req,
                                         manip_msgs::InverseKinematicsFloatArray::Response &resp)
{
}

bool callbackInverseKinematicsPath(manip_msgs::InverseKinematicsPath::Request &req, manip_msgs::InverseKinematicsPath::Response &resp)
{
}

bool callbackInverseKinematicsPose(manip_msgs::InverseKinematicsPose::Request &req, manip_msgs::InverseKinematicsPose::Response &resp)
{
}

int main(int argc, char** argv)
{
    for (int i = 0; i < argc; i++)
	{
		std::string strParam(argv[i]);
		if (strParam.compare("--help") == 0 || strParam.compare("-h") == 0)
		{
			printHelp();
            return 0;
		}
	}
    std::cout << "INITIALIZING MANIPULATION SIMPLE MOVEMENTS BY MARCOSOFT... " << std::endl;
    ros::init(argc, argv, "low_level_moves");
    ros::NodeHandle n;
    ros::ServiceServer srvSrvIKFloatArray = n.advertiseService("low_level_moves/ik_float_array", callbackInverseKinematicsFloatArray);
    ros::ServiceServer srvSrvIKPath = n.advertiseService("low_level_moves/ik_path", callbackInverseKinematicsPath);
    ros::ServiceServer srvSrvIKPose = n.advertiseService("low_level_moves/ik_pose", callbackInverseKinematicsPose);
    ros::Rate loop(10);

    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
