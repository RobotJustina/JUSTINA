#include "LowLevelControl.h"

LowLevelControl::LowLevelControl()
{
	this->classPrompt = "Control.->";
	this->robotDiam = 0.0f;
    this->controlType = 0;
    //MaxAngular:
    //1.5 was used in the old motion planner where angular linear speeds were intended to be in rad/s
    //0.9 was used in the new ROS system where base speeds are considered to be in [-1,1] with 1 = max-motor-speed, with the differential base
    this->MaxAngular = 0.8;
    //Max Angular
    //0.7 was used in the old motion planner where linear speeds were intended to be in m/s
    //0.45 was used in the new ROS system with the differential base
    this->MaxLinear = 0.8;
    this->exp_alpha = 0.463;
    this->exp_beta = 0.1;//0.126; 
    this->lastMaxLinear = 0;
}

LowLevelControl::~LowLevelControl()
{
	
}

void LowLevelControl::SetRobotParams(float diameter)
{
	this->robotDiam = diameter;
	std::cout << classPrompt << "Setting parameters: RobotDiam: " << this->robotDiam << std::endl;
}

void LowLevelControl::CalculateSpeeds(float robotX, float robotY, float robotTheta, float goalX, float goalY, float& lSpeed, float& rSpeed, bool backwards)
{
    float errorX = goalX - robotX;
    float errorY = goalY - robotY;
	float distError = sqrt(errorX * errorX + errorY * errorY);
    float angError = atan2(errorY, errorX) - robotTheta;
   	if (backwards)
		angError += M_PI;
    if(angError > M_PI) angError -= 2 * M_PI;
	if(angError <= -M_PI) angError += 2 * M_PI;

	if(this->controlType == CTRL_EXPONENTIAL)
	{
		//std::cout << "TESTING LOW LEVEL CONTROL: Calculating with exponentials" << std::endl;
		distError = sqrt(distError)*1.5;
		float exp_MaxLinear = distError < this->MaxLinear ? distError : this->MaxLinear;
		if(exp_MaxLinear < 0.5f) exp_MaxLinear = 0.5f;
		if (fabs(exp_MaxLinear - lastMaxLinear) >= 0.1f)
		{
			if(exp_MaxLinear > lastMaxLinear)
				exp_MaxLinear = lastMaxLinear + 0.1f;
			else 
				exp_MaxLinear = lastMaxLinear - 0.1f;
		}
		lastMaxLinear = exp_MaxLinear;
		float expTrans = -(angError * angError) / (2 * this->exp_alpha * this->exp_alpha);
		float vTrans = exp_MaxLinear * exp(expTrans);
		//Angular component
		float expRot = (1 + exp(-angError / this->exp_beta));
		float vAng = this->MaxAngular * (2 / expRot - 1);
		if (!backwards)
		{
			lSpeed = vTrans - this->robotDiam / 2.0f * vAng;
			rSpeed = vTrans + this->robotDiam / 2.0f * vAng;
		}
		else
		{
			lSpeed = -vTrans - this->robotDiam / 2.0f * vAng;
			rSpeed = -vTrans + this->robotDiam / 2.0f * vAng;
		}
	}
}

void LowLevelControl::CalculateSpeeds(float currentTheta, float goalAngle, float& lSpeed, float& rSpeed)
{
	float angError = goalAngle - currentTheta;
    if(angError > M_PI) angError -= 2 * M_PI;
	if(angError <= -M_PI) angError += 2 * M_PI;
	if(fabs(angError) < 0.3)
	  {
	    if(angError < 0)
	      angError = -0.3;
	    else
	      angError = 0.3;
	  }
	if (this->controlType == CTRL_EXPONENTIAL)
	{
		float expRot = (1 + exp(-angError / (this->exp_beta*0.3f)));
		float vAng = this->MaxAngular * (2 / expRot - 1);
		lSpeed = -this->robotDiam / 2.0f * vAng;
		rSpeed = +this->robotDiam / 2.0f * vAng;
	}
}

void LowLevelControl::CalculateSpeeds(float robotX, float robotY, float robotTheta, float goalX, float goalY, float& lSpeed, float& rSpeed)
{
	this->CalculateSpeeds(robotX, robotY, robotTheta, goalX, goalY, lSpeed, rSpeed, false);
}

void LowLevelControl::CalculateSpeedsLateral(float robotX, float robotY, float robotTheta, float goalX, float goalY,
                                             double& linearY, double& angular, bool backwards)
{
    float errorX = goalX - robotX;
    float errorY = goalY - robotY;
	float distError = sqrt(errorX * errorX + errorY * errorY);
    float angError = atan2(errorY, errorX) - robotTheta;
    if(angError > M_PI) angError -= 2 * M_PI;
	if(angError <= -M_PI) angError += 2 * M_PI;

    angError -= M_PI/2; //When moving lateral, angular zero-error points to Y-axis
    if(angError > M_PI) angError -= 2 * M_PI;
	if(angError <= -M_PI) angError += 2 * M_PI;

    if(backwards)
        angError += M_PI;
    if(angError > M_PI) angError -= 2 * M_PI;
	if(angError <= -M_PI) angError += 2 * M_PI;

    distError = sqrt(distError);
    float exp_MaxLinear = distError < this->MaxLinear ? distError : this->MaxLinear;
    if(exp_MaxLinear < 0.18f) exp_MaxLinear = 0.18f;
    if (fabs(exp_MaxLinear - lastMaxLinear) >= 0.08f)
    {
        if(exp_MaxLinear > lastMaxLinear)
            exp_MaxLinear = lastMaxLinear + 0.08f;
        else 
            exp_MaxLinear = lastMaxLinear - 0.08f;
    }
    lastMaxLinear = exp_MaxLinear;
    float expTrans = -(angError * angError) / (2 * this->exp_alpha * this->exp_alpha);
    float vTrans = exp_MaxLinear * exp(expTrans);
    //Angular component
    float expRot = (1 + exp(-angError / this->exp_beta));
    float vAng = this->MaxAngular * (2 / expRot - 1);

    if(backwards)
        vTrans *= -1;

    linearY = vTrans*0.5;
    angular = vAng;
}
