#pragma once
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "bbros_bridge/Default_ROS_BB_Bridge.h"
#include "hri_msgs/RecognizedSpeech.h"

class JustinaHRI
{
public:
    static bool is_node_set;
    //Members for operating speech synthesis and recognition. (Assuming that blackboard modules are used)
    static ros::ServiceClient cltSpGenSay;
    static ros::Publisher pubSprRecognized; 
    static ros::Publisher pubSprHypothesis;
    //Members for operating human_follower node
    static ros::Publisher pubFollowStart;
    static ros::Publisher pubFollowStop;

    //Variables for speech
    static std::string lastRecoSpeech;

    //
    //The startSomething functions return inmediately after starting the requested action
    //The others, block until the action is finished
    //

    static bool setNodeHandle(ros::NodeHandle* nh);
    //Methos for speech synthesis and recognition
    static bool waitForSpokenSentence(std::string& recognizedSentence, int timeOut_ms);
    static void fakeSpokenSentence(std::string sentence);
    static void startSay(std::string strToSay);
    static void say(std::string strToSay);
    //Methods for human following
    static void startFollowHuman();
    static void stopFollowHuman();
};
