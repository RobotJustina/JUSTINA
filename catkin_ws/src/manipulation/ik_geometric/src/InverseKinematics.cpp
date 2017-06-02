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
	std::cout << "Trying to calculate DirectKinematics (Edgar Version)" << std::endl;

	if(articular.size() != 7)
	{
		std::cout << "Error in articular size, it must contain seven values" << std::endl;
		return false;
	}

	//Denavith-Hartenberg Parameters
	float dhD[7] = {0, 0, 0.275, 0, 0.226, 0, 0.165};
    float dhA[7] = {0.065, 0, 0, 0, 0, 0, 0};
    float dhAlpha[7] = {1.5708, 1.5708, -1.5708, 1.5708, -1.5708, 1.5708, 0};
    float dhTheta[7] = {0, 1.5708, -1.5708, 0, 0, 0, 0};


    tf::Quaternion q, q01, q12, q23, q34, q45, q56, q67;
    tf::Transform R01, R12, R23, R34, R45, R56, R67;
    tf::Transform R07;

    R01.setIdentity();
    R12.setIdentity();
    R23.setIdentity();
    R34.setIdentity();
    R45.setIdentity();
    R56.setIdentity();
    R67.setIdentity();

    R07.setIdentity();

    R01.setOrigin(tf::Vector3( dhA[0]*cos(articular[0]), dhA[0]*sin(articular[0]), dhD[0]) );
    R12.setOrigin(tf::Vector3( dhA[1]*cos(articular[1]), dhA[1]*sin(articular[1]), dhD[1]) );
    R23.setOrigin(tf::Vector3( dhA[2]*cos(articular[2]), dhA[2]*sin(articular[2]), dhD[2]) );
    R34.setOrigin(tf::Vector3( dhA[3]*cos(articular[3]), dhA[3]*sin(articular[3]), dhD[3]) );
    R45.setOrigin(tf::Vector3( dhA[4]*cos(articular[4]), dhA[4]*sin(articular[4]), dhD[4]) );
    R56.setOrigin(tf::Vector3( dhA[5]*cos(articular[5]), dhA[5]*sin(articular[5]), dhD[5]) );
    R67.setOrigin(tf::Vector3( dhA[6]*cos(articular[6]), dhA[6]*sin(articular[6]), dhD[6]) );

    q01.setRPY(dhAlpha[0], 0, articular[0] + dhTheta[0]);
    q12.setRPY(dhAlpha[1], 0, articular[1] + dhTheta[1]);
    q23.setRPY(dhAlpha[2], 0, articular[2] + dhTheta[2]);
    q34.setRPY(dhAlpha[3], 0, articular[3] + dhTheta[3]);
    q45.setRPY(dhAlpha[4], 0, articular[4] + dhTheta[4]);
    q56.setRPY(dhAlpha[5], 0, articular[5] + dhTheta[5]);
    q67.setRPY(dhAlpha[6], 0, articular[6] + dhTheta[6]);

    R01.setRotation(q01);
    R12.setRotation(q12);
    R23.setRotation(q23);
    R34.setRotation(q34);
    R45.setRotation(q45);
    R56.setRotation(q56);
    R67.setRotation(q67);


    //This is the transformation from baseArm to endEffector
    R07 = R01 * R12 * R23 * R34 * R45 * R56 * R67;

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

