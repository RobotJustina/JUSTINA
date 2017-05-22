#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>

#define CTRL_EXPONENTIAL 0
#define CTRL_PROPORTIONAL 1

class LowLevelControl
{
public:
	LowLevelControl();
	~LowLevelControl();
	
	std::string classPrompt;
	int controlType;
	float robotDiam;
	float MaxAngular;
	float MaxLinear;
	float exp_alpha;
	float exp_beta;
	float lastMaxLinear;

	void SetRobotParams(float diameter);
	void CalculateSpeeds(float robotX, float robotY, float robotTheta, float goalX, float goalY, float& lSpeed, float& rSpeed, bool backwards);
	void CalculateSpeeds(float currentTheta, float goalAngle, float& lSpeed, float& rSpeed);
    void CalculateSpeeds(float robotX, float robotY, float robotTheta, float goalX, float goalY, float& lSpeed, float& rSpeed);
};
