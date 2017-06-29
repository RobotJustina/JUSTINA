#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <iostream>
#include <sstream>
#include <string>
#include "vision_msgs/Skeletons.h"
#include "vision_msgs/GestureSkeleton.h"
#include "vision_msgs/GestureSkeletons.h"
#include "geometry_msgs/Point.h"
#include "vision_msgs/HandSkeletonPos.h"
#include <visualization_msgs/Marker.h>

ros::Publisher pubGestures;
ros::Publisher pubRHnadPos;
ros::Publisher pubLHnadPos;

void callbackGetGesture(const vision_msgs::Skeletons& msg)
{
    vision_msgs::Skeletons skeletons;
	vision_msgs::Skeleton skeleton;

    skeletons = msg;
    
    vision_msgs::GestureSkeletons gestures_detected;

    while (!skeletons.skeletons.empty())
  	{
    	skeleton = skeletons.skeletons.back();

		if(skeleton.right_hand.position.y > (skeleton.right_hip.position.y + 0.20) && 
		   skeleton.right_hand.position.z > skeleton.right_hip.position.z && 
		   skeleton.right_hand.position.z < skeleton.neck.position.z)
		{
			vision_msgs::GestureSkeleton gesture_detected;

			gesture_detected.id = skeleton.user_id;
			gesture_detected.gesture = "pointing_right";
			gesture_detected.gesture_centroid.x = skeleton.torso.position.x;
			gesture_detected.gesture_centroid.y = skeleton.torso.position.y;
			gesture_detected.gesture_centroid.z = skeleton.torso.position.z;
			//pubGesture.publish(gesture_detected);
			gestures_detected.recog_gestures.push_back(gesture_detected);
 			std::cout << "User: " << skeleton.user_id << " Pointing right" << std::endl;
		}

		if(skeleton.left_hand.position.y < (skeleton.left_hip.position.y - 0.20) && 
		   skeleton.left_hand.position.z > skeleton.left_hip.position.z && 
		   skeleton.left_hand.position.z < skeleton.neck.position.z)
		{
			vision_msgs::GestureSkeleton gesture_detected;

			gesture_detected.id = skeleton.user_id;
			gesture_detected.gesture = "pointing_left";
			gesture_detected.gesture_centroid.x = skeleton.torso.position.x;
			gesture_detected.gesture_centroid.y = skeleton.torso.position.y;
			gesture_detected.gesture_centroid.z = skeleton.torso.position.z;
			//pubGesture.publish(gesture_detected);
			gestures_detected.recog_gestures.push_back(gesture_detected);
			std::cout << "User: " << skeleton.user_id << " Pointing left" << std::endl;
		}

    	if(skeleton.right_hand.position.z > skeleton.neck.position.z)
    	{
			vision_msgs::GestureSkeleton gesture_detected;

			gesture_detected.id = skeleton.user_id;
			gesture_detected.gesture = "right_hand_rised";
			gesture_detected.gesture_centroid.x = skeleton.torso.position.x;
			gesture_detected.gesture_centroid.y = skeleton.torso.position.y;
			gesture_detected.gesture_centroid.z = skeleton.torso.position.z;
			//pubGesture.publish(gesture_detected);
			gestures_detected.recog_gestures.push_back(gesture_detected);
			std::cout << "User: " << skeleton.user_id << " Right hand rised" << std::endl;
    	}

    	if(skeleton.left_hand.position.z  > skeleton.neck.position.z)
    	{
			vision_msgs::GestureSkeleton gesture_detected;

			gesture_detected.id = skeleton.user_id;
			gesture_detected.gesture = "left_hand_rised";
			gesture_detected.gesture_centroid.x = skeleton.torso.position.x;
			gesture_detected.gesture_centroid.y = skeleton.torso.position.y;
			gesture_detected.gesture_centroid.z = skeleton.torso.position.z;
			//pubGesture.publish(gesture_detected);
			gestures_detected.recog_gestures.push_back(gesture_detected);
			std::cout << "User: " << skeleton.user_id << " Left hand rised" << std::endl;
    	}

		skeletons.skeletons.pop_back();
  	}
  	pubGestures.publish(gestures_detected);
}

void callbackGetRHandPos(const vision_msgs::Skeletons& msg)
{
	vision_msgs::Skeletons skeletons;

    skeletons = msg;
    geometry_msgs::Point handCentroid;

    vision_msgs::HandSkeletonPos hands_pos;
    
    for(int i = 0; i < skeletons.skeletons.size(); i++){
    	vision_msgs::Skeleton skeleton = skeletons.skeletons[i];
    	handCentroid.x = skeleton.right_hand.position.x;
    	handCentroid.y = skeleton.right_hand.position.y;
    	handCentroid.z = skeleton.right_hand.position.z;
    	hands_pos.hands_position.push_back(handCentroid);
    }

    pubRHnadPos.publish(hands_pos);
}

void callbackGetLHandPos(const vision_msgs::Skeletons& msg)
{
	vision_msgs::Skeletons skeletons;

    skeletons = msg;

    geometry_msgs::Point handCentroid;

    vision_msgs::HandSkeletonPos hands_pos;

    for(int i = 0; i < skeletons.skeletons.size(); i++){
    	vision_msgs::Skeleton skeleton = skeletons.skeletons[i];
    	handCentroid.x = skeleton.left_hand.position.x;
    	handCentroid.y = skeleton.left_hand.position.y;
    	handCentroid.z = skeleton.left_hand.position.z;
    	hands_pos.hands_position.push_back(handCentroid);
    }
    pubLHnadPos.publish(hands_pos);
}



int main(int argc, char** argv)
{
	std::cout << "INITIALIZING GESTURE RECOGNIZER SKELETONS..." << std::endl;
    ros::init(argc, argv, "gesture_recognizer");
    ros::NodeHandle n;

    ros::Subscriber subGetGesture = n.subscribe("/vision/skeleton_finder/skeleton_recog", 1, callbackGetGesture);
    ros::Subscriber subGetRHandPos = n.subscribe("/vision/skeleton_finder/skeleton_recog", 1, callbackGetRHandPos);
    ros::Subscriber subGetLHandPos = n.subscribe("/vision/skeleton_finder/skeleton_recog", 1, callbackGetLHandPos);
    pubGestures = n.advertise<vision_msgs::GestureSkeletons> ("/vision/gesture_recog_skeleton/gesture_recog", 1);
    pubRHnadPos = n.advertise<vision_msgs::HandSkeletonPos> ("/vision/gesture_recog_skeleton/right_hand_pos", 1);
    pubLHnadPos = n.advertise<vision_msgs::HandSkeletonPos> ("/vision/gesture_recog_skeleton/left_hand_pos", 1);

    ros::Rate loop(30);
    
    std::cout << "GestureRecognizer.->Running..." << std::endl;
    
    while(ros::ok())
    {
        loop.sleep();
        ros::spinOnce();
    }

    return 0;
}
