#include <iostream>
#include "ros/ros.h"
#include "manip_msgs/InverseKinematicsFloatArray.h"
#include "manip_msgs/InverseKinematicsPath.h"
#include "manip_msgs/InverseKinematicsPose.h"
#include "manip_msgs/DirectKinematics.h"
#include "InverseKinematics.h"

//T O D O :   T H I S   I S   A   V E R Y   I M P O R T A N T   T O - D O !!!!!!!!!
//Dimensions of the arms should be taken from the robot description (urdf file in the planning/knowledge/hardware/justina.xml)
//Values of D1, D2, D3 and D4 correspond to Denavig-Hartenberg parameters and are given in the urdf
//In the origin tag of each joint.

void printHelp()
{
    std::cout << "INVERSE KINEMATICS GEOMETRIC BY MARCOSOFT..." << std::endl;
    std::cout << " - This node calculates the inverse kinematics using a geometric approach." << std::endl;
    std::cout << " - Inverse kinematics is calculated considering a 7DOF manipulator" << std::endl;
    std::cout << " - with all joints of revolute type and with a spheric wrist." << std::endl;
    std::cout << " - Calculations are made assuming that cartesian coords are w.r.t each arm." << std::endl;
    std::cout << " - Since both arms have almost the same frame (same orientation, " << std::endl;
    std::cout << " - and translated one from each other by the shoulders lenght), then, inverse kinematics" << std::endl;
    std::cout << " - can be calculated in the same way." << std::endl;
    std::cout << " - Planning algorithms should do the corresponding transforms (eg from base_link to left_arm_link)." << std::endl;
    std::cout << " - In cartesian coords, the seven values are x, y, z, roll, pitch, yaw and elbow. " << std::endl;
    std::cout << "PLEASE DON'T TRY TO OPERATE JUSTINA IF YOU ARE NOT QUALIFIED ENOUGH" << std::endl;
}

bool callbackInverseKinematicsFloatArray(manip_msgs::InverseKinematicsFloatArray::Request &req,
                                         manip_msgs::InverseKinematicsFloatArray::Response &resp)
{
    //In all cases, the resulting articular pose is a std::vector<float> with seven values corresponding to the 7DOF of the arm
    if(req.cartesian_pose.data.size() == 7)
    {
        std::cout << "InverseKinematics.->Calculating inverse kinematics from float array with seven values..." << std::endl;
        return InverseKinematics::GetInverseKinematics(req.cartesian_pose.data, resp.articular_pose.data);
    }
    if(req.cartesian_pose.data.size() == 6)
    {
        float x = req.cartesian_pose.data[0];
        float y = req.cartesian_pose.data[1];
        float z = req.cartesian_pose.data[2];
        float roll = req.cartesian_pose.data[3];
        float pitch = req.cartesian_pose.data[4];
        float yaw = req.cartesian_pose.data[5];
        std::cout << "InverseKinematics.->Calculating inverse kinematics from float array with six values..." << std::endl;
        return InverseKinematics::GetInverseKinematics(x, y, z, roll, pitch, yaw, resp.articular_pose.data);
    }
    if (req.cartesian_pose.data.size() == 3)
    {
        float x = req.cartesian_pose.data[0];
        float y = req.cartesian_pose.data[1];
        float z = req.cartesian_pose.data[2];
        std::cout << "InverseKinematics.->Calculating inverse kinematics from float array with three values..." << std::endl;
        return InverseKinematics::GetInverseKinematics(x, y, z, resp.articular_pose.data);
    }
    std::cout << "Ik_Geometric.->Cannot calculate inv kinematics: Invalid number of args in request." << std::endl;
    return false;
}

bool callbackInverseKinematicsFloatArrayWithoutOpt(manip_msgs::InverseKinematicsFloatArray::Request &req,
                                         manip_msgs::InverseKinematicsFloatArray::Response &resp)
{
    //In all cases, the resulting articular pose is a std::vector<float> with seven values corresponding to the 7DOF of the arm
    if(req.cartesian_pose.data.size() == 7)
    {
        std::cout << "InverseKinematics.->Calculating inverse kinematics from float array with seven values..." << std::endl;
        return InverseKinematics::GetInverseKinematics(req.cartesian_pose.data, resp.articular_pose.data);
    }
    if(req.cartesian_pose.data.size() == 6)
    {
        float x = req.cartesian_pose.data[0];
        float y = req.cartesian_pose.data[1];
        float z = req.cartesian_pose.data[2];
        float roll = req.cartesian_pose.data[3];
        float pitch = req.cartesian_pose.data[4];
        float yaw = req.cartesian_pose.data[5];
        std::cout << "InverseKinematics.->Calculating inverse kinematics from float array with six values..." << std::endl;
        return InverseKinematics::GetInverseKinematics(x, y, z, roll, pitch, yaw, resp.articular_pose.data);
    }
    if (req.cartesian_pose.data.size() == 3)
    {
        float x = req.cartesian_pose.data[0];
        float y = req.cartesian_pose.data[1];
        float z = req.cartesian_pose.data[2];
        std::cout << "InverseKinematics.->Calculating inverse kinematics from float array with three values..." << std::endl;
        return InverseKinematics::GetInverseKinematicsWithoutOptimization(x, y, z, resp.articular_pose.data);
    }
    std::cout << "Ik_Geometric.->Cannot calculate inv kinematics: Invalid number of args in request." << std::endl;
    return false;
}

bool callbackInverseKinematicsPath(manip_msgs::InverseKinematicsPath::Request &req, manip_msgs::InverseKinematicsPath::Response &resp)
{
    //Cartesian path is a nav_msgs::Path and articular path is an array of Float32MultiArray (yes, is an array of arrays)
    return InverseKinematics::GetInverseKinematics(req.cartesian_path, resp.articular_path);
}

bool callbackInverseKinematicsPose(manip_msgs::InverseKinematicsPose::Request &req, manip_msgs::InverseKinematicsPose::Response &resp)
{
    //Cartesian pose is a geometry_msgs::Pose and the resulting articular pose is a std::vector<float> with seven values
    return InverseKinematics::GetInverseKinematics(req.cartesian_pose, resp.articular_pose.data);
}

bool callbackDirectKinematics(manip_msgs::DirectKinematics::Request &req, manip_msgs::DirectKinematics::Response &resp)
{
    return InverseKinematics::GetDirectKinematics(req.articular_pose.data, resp.cartesian_pose.data);
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
    std::cout << "INITIALIZING INVERSE KINEMATICS GEOMETRIC BY MARCOSOFT... " << std::endl;
    ros::init(argc, argv, "low_level_moves");
    ros::NodeHandle n;
    ros::ServiceServer srvSrvIKFloatArray = n.advertiseService("/manipulation/ik_geometric/ik_float_array", callbackInverseKinematicsFloatArray);
    ros::ServiceServer srvSrvIKFloatArrayWithoutOp = n.advertiseService("/manipulation/ik_geometric/ik_float_array_without_opt", callbackInverseKinematicsFloatArrayWithoutOpt);
    ros::ServiceServer srvSrvIKPath = n.advertiseService("/manipulation/ik_geometric/ik_path", callbackInverseKinematicsPath);
    ros::ServiceServer srvSrvIKPose = n.advertiseService("/manipulation/ik_geometric/ik_pose", callbackInverseKinematicsPose);
    ros::ServiceServer srvSrvDirectKin = n.advertiseService("/manipulation/ik_geometric/direct_kinematics", callbackDirectKinematics);
    ros::Rate loop(10);

    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
