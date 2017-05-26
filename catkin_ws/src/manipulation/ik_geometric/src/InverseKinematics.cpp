#include "InverseKinematics.h"

bool InverseKinematics::GetInverseKinematics(std::vector<float>& cartesian, std::vector<float>& articular)
{
    std::cout << "Trying to calculate InverseKinematics .... " << std::endl;
    return true;
}


bool InverseKinematics::GetInverseKinematics(float x, float y, float z, float roll, float pitch, float yaw, std::vector<float>& articular)
{

    return true;
}

bool InverseKinematics::GetInverseKinematics(float x, float y, float z, std::vector<float>& articular)
{

    return true;
}

bool InverseKinematics::GetDirectKinematics(std::vector<float>& articular, std::vector<float>& cartesian)
{
	std::cout << "Trying to calculate DirectKinematics" << std::endl;

	if(articular.size() != 7)
	{
		std::cout << "Error in articular size, it must contain seven values" << std::endl;
		return false;
	}

	float dhD[7] = {0, 0, 0.27, 0, 0.2126, 0, 0.13};
    float dhA[7] = {0.0603, 0, 0, 0, 0, 0, 0};
    float dhAlpha[7] = {1.5708, 1.5708, -1.5708, 1.5708, -1.5708, 1.5708, 0};
    float dhTheta[7] = {0, 1.5708, -1.5708, 0, 0, 0, 0};

    tf::Quaternion q;
    tf::Transform R07;
    R07.setIdentity();
    for(size_t i=0; i < 7; i++)
    {
        tf::Transform temp;
        temp.setOrigin(tf::Vector3(dhA[i]*cos(articular[i]), dhA[i]*sin(articular[i]), dhD[i]));
        q.setRPY(dhAlpha[i],0,articular[i] + dhTheta[i]);
        temp.setRotation(q);
        R07 = R07 * temp;
    }

    tf::Vector3 endEffector(0,0,0);
    endEffector = R07 * endEffector; //XYZ position of the end effector
    q = R07.getRotation();

    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    cartesian.clear();
    cartesian.push_back(endEffector.x());
    cartesian.push_back(endEffector.y());
    cartesian.push_back(endEffector.z());
    cartesian.push_back(roll - 1.5708); //This minus pi/2 corrects the fact that the final-effector frame is not aligned with left_arm_link0
    cartesian.push_back(pitch);
    cartesian.push_back(yaw - 1.5708);
    cartesian.push_back(0);

    std::cout << "DirectKinematics.->Calculated cartesian: " << std::endl;
    for (int i=0; i < 7; i++) std::cout << "   " << cartesian[i] << std::endl;
	std::cout << std::endl;



    return true;
}

