#ifndef ACT_PLN_RBT_KNW
#define ACT_PLN_RBT_KNW

#include "ros/ros.h"
#include <map>
#include <string>

class RobotKnowledge
{
public:
	/*
	* Robot ARMS Constants
	*/
	enum ARM_SIDE
	{
		LeftArm,
		RightArm
	};

	/*
	* Robot Tests
	*/
	enum SM
	{
		Perception_FB,
		Navigation_FB,
		Speech_FB,
		KnowHome_TB,
		Welcoming_TB,
		GrannyAnnie_TB,
		GPSR,
		DefaultTest
	};

	//dictionary to map the object instances to classes
	std::map<std::string, std::string> objectDictionary;

	RobotKnowledge();
private:
	/*
	* Node handle and subscriber for speech reco
	*/
	//ros::NodeHandle nh_reco;
	//ros::Subscriber reco_sub;

	//static void updateRecognizedSentences(const bbros_bridge::RecognizedSpeech::ConstPtr& recognizedSentence);
};

#endif
