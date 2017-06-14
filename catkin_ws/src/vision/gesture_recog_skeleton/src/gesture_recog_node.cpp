#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <iostream>
#include <sstream>
#include <string>
#include "vision_msgs/Skeletons.h"
#include "vision_msgs/GestureSkeleton.h"
#include "vision_msgs/GestureSkeletons.h"
#include <visualization_msgs/Marker.h>

ros::Publisher vis_pubRight; 
ros::Publisher vis_pubLeft; 
ros::Publisher pubGestures;

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
			gesture_detected.gesture_centroid.x = skeleton.right_hand.position.x;
			gesture_detected.gesture_centroid.y = skeleton.right_hand.position.y;
			gesture_detected.gesture_centroid.z = skeleton.right_hand.position.z;
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
			gesture_detected.gesture_centroid.x = skeleton.left_hand.position.x;
			gesture_detected.gesture_centroid.y = skeleton.left_hand.position.y;
			gesture_detected.gesture_centroid.z = skeleton.left_hand.position.z;
			//pubGesture.publish(gesture_detected);
			gestures_detected.recog_gestures.push_back(gesture_detected);
			std::cout << "User: " << skeleton.user_id << " Pointing left" << std::endl;
		}

    	if(skeleton.right_hand.position.z > skeleton.neck.position.z)
    	{
			vision_msgs::GestureSkeleton gesture_detected;

			gesture_detected.id = skeleton.user_id;
			gesture_detected.gesture = "right_hand_rised";
			gesture_detected.gesture_centroid.x = skeleton.right_hand.position.x;
			gesture_detected.gesture_centroid.y = skeleton.right_hand.position.y;
			gesture_detected.gesture_centroid.z = skeleton.right_hand.position.z;
			//pubGesture.publish(gesture_detected);
			gestures_detected.recog_gestures.push_back(gesture_detected);
			std::cout << "User: " << skeleton.user_id << " Right hand rised" << std::endl;
    	}

    	if(skeleton.left_hand.position.z > skeleton.neck.position.z)
    	{
			vision_msgs::GestureSkeleton gesture_detected;

			gesture_detected.id = skeleton.user_id;
			gesture_detected.gesture = "left_hand_rised";
			gesture_detected.gesture_centroid.x = skeleton.left_hand.position.x;
			gesture_detected.gesture_centroid.y = skeleton.left_hand.position.y;
			gesture_detected.gesture_centroid.z = skeleton.left_hand.position.z;
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
	vision_msgs::Skeleton skeleton;

    skeletons = msg;

    while (!skeletons.skeletons.empty())
  	{
    	skeleton = skeletons.skeletons.back();

    	visualization_msgs::Marker markerRightHand;
		markerRightHand.header.frame_id = "map";
		markerRightHand.header.stamp = ros::Time();
		markerRightHand.ns = "RightHand";
		markerRightHand.id = 0;
		markerRightHand.type = visualization_msgs::Marker::SPHERE;
		markerRightHand.action = visualization_msgs::Marker::ADD;
		markerRightHand.pose.position.x = skeleton.right_hand.position.x;
		markerRightHand.pose.position.y = skeleton.right_hand.position.y;
		markerRightHand.pose.position.z = skeleton.right_hand.position.z;
		markerRightHand.pose.orientation.x = 0.0;
		markerRightHand.pose.orientation.y = 0.0;
		markerRightHand.pose.orientation.z = 0.0;
		markerRightHand.pose.orientation.w = 1.0;
		markerRightHand.scale.x = 0.1;
		markerRightHand.scale.y = 0.1;
		markerRightHand.scale.z = 0.1;
		markerRightHand.color.a = 1.0; // Don't forget to set the alpha!
		markerRightHand.color.r = 0.0;
		markerRightHand.color.g = 1.0;
		markerRightHand.color.b = 0.0;
		//only if using a MESH_RESOURCE marker type:
		markerRightHand.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
		vis_pubRight.publish( markerRightHand );
    }
}


int main(int argc, char** argv)
{
	std::cout << "INITIALIZING GESTURE RECOGNIZER SKELETONS..." << std::endl;
    ros::init(argc, argv, "gesture_recognizer");
    ros::NodeHandle n;

    ros::Subscriber subGetGesture = n.subscribe("/vision/skeleton_finder/skeleton_recog", 1, callbackGetGesture);
    vis_pubRight = n.advertise<visualization_msgs::Marker> ("visualization_marker", 0 );
    vis_pubLeft = n.advertise<visualization_msgs::Marker> ("visualization_marker", 0 );
    pubGestures = n.advertise<vision_msgs::GestureSkeletons> ("/vision/gesture_recog_skeleton/gesture_recog", 1);

    ros::Rate loop(30);
    
    std::cout << "GestureRecognizer.->Running..." << std::endl;
    
    while(ros::ok())
    {
        loop.sleep();
        ros::spinOnce();
    }

    return 0;
}
