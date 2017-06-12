#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <iostream>
#include <sstream>
#include <string>
#include "vision_msgs/Skeletons.h"
#include "vision_msgs/GestureSkeleton.h"
#include <visualization_msgs/Marker.h>

ros::Publisher vis_pubRight; 
ros::Publisher vis_pubLeft; 
ros::Publisher pubGesture;

void callbackGetGesture(const vision_msgs::Skeletons& msg)
{
    vision_msgs::Skeletons skeletons;
	vision_msgs::Skeleton skeleton;

    skeletons = msg;
    float dot;
    float lenSq1;
    float lenSq2;
    float angle;

    while (!skeletons.skeletons.empty())
  	{
    	skeleton = skeletons.skeletons.back();

    	/*geometry_msgs::Vector3 v1;
    	geometry_msgs::Vector3 v2;

    	v1.x = skeleton.torso.position.x - skeleton.neck.position.x;
    	v1.y = skeleton.torso.position.y - skeleton.neck.position.y;
    	v1.z = skeleton.torso.position.z - skeleton.neck.position.z;

    	v2.x = skeleton.right_hand.position.x - skeleton.neck.position.x;
    	v2.y = skeleton.right_hand.position.y - skeleton.neck.position.y;
    	v2.z = skeleton.right_hand.position.z - skeleton.neck.position.z;

    	dot = (v1.x * v2.x) + (v1.y * v2.y) + (v1.z * v2.z);    
		lenSq1 = (v1.x * v1.x) + (v1.y * v1.y) + (v1.z * v1.z);
		lenSq2 = (v2.x * v2.x) + (v2.y * v2.y) + (v2.z * v2.z);
		angle = acos(dot / sqrt(lenSq1 * lenSq2));
		angle = angle * 180.0 / 3.14159265; 

		if(angle > 30.0)
		{
			std::cout << "User: " << skeleton.user_id << " Pointing right" << std::endl;
			std::cout << "Angle: " << angle << std::endl;
		}*/

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
			pubGesture.publish(gesture_detected);
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
			pubGesture.publish(gesture_detected);
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
			pubGesture.publish(gesture_detected);
			std::cout << "User: " << skeleton.user_id << " Right hand rised" << std::endl;
			//std::cout << "hand position en z: " << skeleton.right_hand.position.x << std::endl;
			//std::cout << "shoulder position en z: " << skeleton.right_shoulder.position.x << std::endl;
    	}

    	if(skeleton.left_hand.position.z > skeleton.neck.position.z)
    	{
			vision_msgs::GestureSkeleton gesture_detected;

			gesture_detected.id = skeleton.user_id;
			gesture_detected.gesture = "left_hand_rised";
			gesture_detected.gesture_centroid.x = skeleton.left_hand.position.x;
			gesture_detected.gesture_centroid.y = skeleton.left_hand.position.y;
			gesture_detected.gesture_centroid.z = skeleton.left_hand.position.z;
			pubGesture.publish(gesture_detected);
			std::cout << "User: " << skeleton.user_id << " Left hand rised" << std::endl;
			//std::cout << "hand position en z: " << skeleton.left_hand.position.x << std::endl;
			//std::cout << "shoulder position en z: " << skeleton.left_shoulder.position.x << std::endl;
    	}

		skeletons.skeletons.pop_back();

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

		visualization_msgs::Marker markerLeftHand;
		markerLeftHand.header.frame_id = "map";
		markerLeftHand.header.stamp = ros::Time();
		markerLeftHand.ns = "LeftHand";
		markerLeftHand.id = 1;
		markerLeftHand.type = visualization_msgs::Marker::SPHERE;
		markerLeftHand.action = visualization_msgs::Marker::ADD;
		markerLeftHand.pose.position.x = skeleton.left_hand.position.x;
		markerLeftHand.pose.position.y = skeleton.left_hand.position.y;
		markerLeftHand.pose.position.z = skeleton.left_hand.position.z;
		markerLeftHand.pose.orientation.x = 0.0;
		markerLeftHand.pose.orientation.y = 0.0;
		markerLeftHand.pose.orientation.z = 0.0;
		markerLeftHand.pose.orientation.w = 1.0;
		markerLeftHand.scale.x = 0.1;
		markerLeftHand.scale.y = 0.1;
		markerLeftHand.scale.z = 0.1;
		markerLeftHand.color.a = 1.0; // Don't forget to set the alpha!
		markerLeftHand.color.r = 1.0;
		markerLeftHand.color.g = 0.0;
		markerLeftHand.color.b = 0.0;
		//only if using a MESH_RESOURCE marker type:
		markerLeftHand.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
		vis_pubLeft.publish( markerLeftHand );
  	}
}

int main(int argc, char** argv)
{
	std::cout << "INITIALIZING GESTURE RECOGNIZER SKELETONS..." << std::endl;
    ros::init(argc, argv, "gesture_recognizer");
    ros::NodeHandle n;

    ros::Subscriber subRisingHand = n.subscribe("/vision/skeleton_finder/skeleton_recog", 1, callbackGetGesture);
    vis_pubRight = n.advertise<visualization_msgs::Marker> ("visualization_marker", 0 );
    vis_pubLeft = n.advertise<visualization_msgs::Marker> ("visualization_marker", 0 );
    pubGesture = n.advertise<vision_msgs::GestureSkeleton> ("/vision/gesture_recog_skeleton/gesture_recog", 1);

    ros::Rate loop(30);
    
    std::cout << "GestureRecognizer.->Running..." << std::endl;
    
    while(ros::ok())
    {
        loop.sleep();
        ros::spinOnce();
    }

    return 0;
}
