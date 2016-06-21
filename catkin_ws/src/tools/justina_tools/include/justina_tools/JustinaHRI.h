#pragma once
#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "boost/date_time/posix_time/posix_time.hpp"
#include "bbros_bridge/Default_ROS_BB_Bridge.h"
#include "hri_msgs/RecognizedSpeech.h"

class JustinaHRI
{
private:
    static bool is_node_set;
    //Members for operating speech synthesis and recognition. (Assuming that blackboard modules are used)
    static ros::Publisher pubFakeSprRecognized; 
    static ros::Publisher pubFakeSprHypothesis;
    static ros::Subscriber subSprRecognized; 
    static ros::Subscriber subSprHypothesis;
    static ros::ServiceClient cltSpgSay;
    //Members for operating human_follower node
    static ros::Publisher pubFollowStartStop;
    static ros::Publisher pubLegsEnable;
    static ros::Subscriber subLegsFound;
    //Variables for speech
    static std::string _lastRecoSpeech;
    static std::vector<std::string> _lastSprHypothesis;
    static std::vector<float> _lastSprConfidences;
    static bool newSprRecognizedReceived;
    static bool _legsFound;
    //Variabeles for qr reader
    static ros::Subscriber subQRReader;
    static boost::posix_time::ptime timeLastQRReceived;
    static std::string lastQRReceived;

public:
    //
    //The startSomething functions return inmediately after starting the requested action
    //The others, block until the action is finished
    //

    static bool setNodeHandle(ros::NodeHandle* nh);
    //Methos for speech synthesis and recognition
    static bool waitForSpeechRecognized(std::string& recognizedSentence, int timeOut_ms);
    static bool waitForSpeechHypothesis(std::vector<std::string>& sentences, std::vector<float>& confidences, int timeOut_ms);
    static bool waitForSpecificSentence(std::string expectedSentence, int timeOut_ms);
    static bool waitForSpecificSentence(std::string option1, std::string option2, std::string& recog, int timeOut_ms);
    static bool waitForSpecificSentence(std::string option1, std::string option2, std::string option3,
                                        std::string& recog, int timeOut_ms);
    static bool waitForSpecificSentence(std::string option1, std::string option2, std::string option3, std::string option4,
                                        std::string& recog, int timeOut_ms);
    static bool waitForSpecificSentence(std::vector<std::string>& options, std::string& recognized, int timeOut_ms);
    static bool waitForUserConfirmation(bool& confirmation, int timeOut_ms);
    static std::string lastRecogSpeech();
    static void fakeSpeechRecognized(std::string sentence);
    static void startSay(std::string strToSay);
    static void say(std::string strToSay);
    //Methods for human following
    static void startFollowHuman();
    static void stopFollowHuman();
    static void enableLegFinder(bool enable);
    static bool frontalLegsFound();

private:
    //Speech recog and synthesis
    static void callbackSprRecognized(const std_msgs::String::ConstPtr& msg);
    static void callbackSprHypothesis(const hri_msgs::RecognizedSpeech::ConstPtr& msg);
    //human following
    static void callbackLegsFound(const std_msgs::Empty::ConstPtr& msg);
    //Methods for qr reader
    static void callbackQRRecognized(const std_msgs::String::ConstPtr& msg);
};
