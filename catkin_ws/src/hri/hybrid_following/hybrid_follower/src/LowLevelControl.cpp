#include "LowLevelControl.h"

LowLevelControl::LowLevelControl()
{
	this->classPrompt = "Control.->";
    //This is for the old omni base
	/*this->robotDiam = 0.25f;//0.48
    this->controlType = 0;
    this->MaxAngular = 0.6; //1.5
    this->MaxLinear = 1.0; //0.7
    this->exp_alpha = 0.463;
    this->exp_beta = 0.126; 
    this->lastMaxLinear = 0;*/
	this->classPrompt = "Control.->";
	this->robotDiam = 0.25f;
    this->controlType = 0;
    this->MaxAngular = 0.6;//0.3
    this->MaxLinear = 0.35;
    this->exp_alpha = 0.2;
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
        distError -= 0.7; //Distance to stop in front of a human
        if(distError < 0) distError = 0;
        distError = sqrt(distError);
		float exp_MaxLinear = distError < this->MaxLinear ? distError : this->MaxLinear;
		if(exp_MaxLinear < 0.1f) exp_MaxLinear = 0.0f;
		if (fabs(exp_MaxLinear - lastMaxLinear) >= 0.2f)
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
			lSpeed = -vTrans - this->robotDiam / 2.0f * vAng ;
			rSpeed = -vTrans + this->robotDiam / 2.0f * vAng;
		}
	}
}

void LowLevelControl::CalculateSpeeds(float currentTheta, float goalAngle, float& lSpeed, float& rSpeed)
{
	float angError = goalAngle - currentTheta;
    if(angError > M_PI) angError -= 2 * M_PI;
	if(angError <= -M_PI) angError += 2 * M_PI;
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
