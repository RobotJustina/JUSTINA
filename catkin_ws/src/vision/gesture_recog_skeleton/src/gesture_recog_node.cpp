#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <iostream>
#include <sstream>
#include <string>

#include "vision_msgs/Skeletons.h"
#include <visualization_msgs/Marker.h>

ros::Publisher vis_pubRight; 
ros::Publisher vis_pubLeft; 

void callbackisRisingHand(const vision_msgs::Skeletons& msg)
{
    vision_msgs::Skeletons skeletons;
	vision_msgs::Skeleton skeleton;

    skeletons = msg;

    //std::cout << "callbackisRisingHand" << std::endl;

    while (!skeletons.skeletons.empty())
  	{
    	skeleton = skeletons.skeletons.back();


    	if(skeleton.right_hand.position.z > skeleton.right_shoulder.position.z)
    	{
			std::cout << "User: " << skeleton.user_id << "Right hand rised" << std::endl;
			std::cout << "hand position en z: " << skeleton.right_hand.position.x << std::endl;
			std::cout << "shoulder position en z: " << skeleton.right_shoulder.position.x << std::endl;
    	}

    	if(skeleton.left_hand.position.z > skeleton.left_shoulder.position.z)
    	{
			std::cout << "User: " << skeleton.user_id << "Left hand rised" << std::endl;
			std::cout << "hand position en z: " << skeleton.left_hand.position.x << std::endl;
			std::cout << "shoulder position en z: " << skeleton.left_shoulder.position.x << std::endl;
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

    ros::Subscriber subRisingHand = n.subscribe("/vision/skeleton_finder/skeleton_recog", 1, callbackisRisingHand);
    vis_pubRight = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    vis_pubLeft = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    ros::Rate loop(30);
    
    std::cout << "GestureRecognizer.->Running..." << std::endl;
    
    while(ros::ok())
    {
        loop.sleep();
        ros::spinOnce();
    }

    return 0;
}
