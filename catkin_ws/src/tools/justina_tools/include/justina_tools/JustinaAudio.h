#pragma once
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "audio_msgs/srvAngle.h"



class JustinaAudio
{

private:
	static bool is_node_set;
	static ros::Subscriber subAngle;
	static ros::Subscriber subFlag;
	static ros::Publisher pubStart; 
	static ros::ServiceClient servAng; 

public:
	static bool setNodeHandle(ros::NodeHandle* nh);
    static bool startSimpleAudioSource();
    static bool isProcessTerminate();
    static float getAudioSource();

private:    
    static void arrayCallback(const std_msgs::Float32MultiArray::ConstPtr& array);
    static void flagCallback(const std_msgs::Bool::ConstPtr& flag);
    //static void task1();
   
};


