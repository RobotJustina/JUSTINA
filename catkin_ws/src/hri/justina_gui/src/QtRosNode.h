#pragma once
#include <iostream>
#include <cmath>
#include <QThread>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Path.h"
#include "hri_msgs/RecognizedSpeech.h"
#include "navig_msgs/PathFromMap.h"
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaNavigation.h"

class QtRosNode : public QThread
{
Q_OBJECT
public:
    QtRosNode();
    ~QtRosNode();

    ros::NodeHandle* n;
    //ros::Publisher pub_SimpleMove_GoalDist; //All this commented lines should be replaced by some
    //ros::Publisher pub_SimpleMove_GoalPath; //JustinaHardware, JustinaNavigation, JustinaVision, 
    //ros::Publisher pub_Head_GoalPose; //JustinaHRI or JustinaManipulation
    //ros::Publisher pub_La_GoalPose;
    //ros::Publisher pub_Ra_GoalPose;
    //ros::Publisher pub_Spg_Say;
    ros::Publisher pub_Spr_Recognized; //These are not in a JustinaSomething class cause they are fake
    ros::Publisher pub_Spr_Hypothesis;
    bool gui_closed;
    
    void run();
    void setNodeHandle(ros::NodeHandle* nh);
    //bool call_PathCalculator_WaveFront(float currentX, float currentY, float currentAng, float goalX,
    //                                     float goalY, float goalAng, nav_msgs::Path& resultPath);
    //bool call_PathCalculator_AStar(float currentX, float currentY, float currentAng, float goalX,
    //                             float goalY, float goalAngle, nav_msgs::Path& resultPath);
    //void publish_SimpleMove_GoalDist(float goalDist);
    //void publish_SimpleMove_GoalPath(nav_msgs::Path& path);  //Same as before, this methods should be
    //void publish_Head_GoalPose(float pan, float tilt);       //Replaced for JustinaSomething methods
    //void publish_La_GoalPose(std::vector<float> angles);
    //void publish_Ra_GoalPose(std::vector<float> angles);
    //void publish_Spg_Say(std::string strToSay);
    void publish_Spr_Recognized(std::string fakeRecoString);

signals:
    void updateGraphics();
    void onRosNodeFinished();
    /*
    void onCurrentRobotPoseReceived(float currentX, float currentY, float currentTheta);
    void onCurrentHeadPoseReceived(float pan, float tilt);
    void onCurrentLaPoseReceived(std::vector<float> angles);
    void onCurrentRaPoseReceived(std::vector<float> angles);
    void onNavigationGoalReached(bool success);
    */

private:
    //In these methods, signals are emmited to update the gui anytime a new value arrives.
    //Neverthless, in order to use JustinaSomething classes, signals will be emmited in
    //run() method, periodically, and in the slots of the main window, the value will be
    //checked using some JustinaSomething class.
    //TODO: maybe we can emmit only one signal: updateGuiValues()
    //and inside the slot associated with this signal, update all values by getting
    //Justina current values using JustinaSomething classes
    //void callbackRobotCurrentPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    //void callbackHeadCurrentPose(const std_msgs::Float32MultiArray::ConstPtr& msg);
    //void callbackLaCurrentPose(const std_msgs::Float32MultiArray::ConstPtr& msg);
    //void callbackRaCurrentPose(const std_msgs::Float32MultiArray::ConstPtr& msg);
    //void callbackNavigGoalReached(const std_msgs::Bool::ConstPtr& msg);
};
