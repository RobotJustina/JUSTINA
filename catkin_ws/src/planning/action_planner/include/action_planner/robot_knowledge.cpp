#include "action_planner/robot_knowledge.h"


/*
* Constructor
*/
RobotKnowledge::RobotKnowledge()
{
	objectDictionary["a1"] = "a";
	objectDictionary["a2"] = "a";
	objectDictionary["a3"] = "a";
	objectDictionary["a4"] = "a";
	objectDictionary["b1"] = "b";
	objectDictionary["b2"] = "b";
	objectDictionary["c1"] = "c";
	objectDictionary["c2"] = "c";
	objectDictionary["d1"] = "d";
	objectDictionary["d2"] = "d";
//	//Initialize the recognized sentences with default values
//	recoSentenceTuple defaultRecoValue;
//	recognizedSentences = std::vector<recoSentenceTuple>(3, defaultRecoValue);
//	//recognizedSentences.push_back(defaultRecoValue);
//	//recognizedSentences.push_back(defaultRecoValue);
//	//recognizedSentences.push_back(defaultRecoValue);
//	
//	//subscribe to the recognizedSpeech topic
//	reco_sub = nh_reco.subscribe("recognizedSpeech", 100, updateRecognizedSentences);
//
//	std::cout << "reco initial value: " << recognizedSentences[0].hypotesis << " " << recognizedSentences[0].confidence << std::endl;
//	std::cout << "reco initial value: " << recognizedSentences[1].hypotesis << " " << recognizedSentences[1].confidence << std::endl;
//	std::cout << "reco initial value: " << recognizedSentences[2].hypotesis << " " << recognizedSentences[2].confidence << std::endl;
}


