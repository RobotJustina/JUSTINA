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

enum GestureMethod{
    NITE, OPENPOSE_3D, OPENPOSE_2D
};

ros::Publisher pubGestures;
ros::Publisher pubRHnadPos;
ros::Publisher pubLHnadPos;
ros::Publisher pubTorsoPos;

GestureMethod gestureMethod;

bool findIndexJoint(const vision_msgs::Skeleton skeleton, std::string name_joint, int &indexJoint){
    for(int i = 0; i < skeleton.joints.size(); i++){
        if(skeleton.joints[i].name_joint.data.compare(name_joint) == 0){
            indexJoint = i;
            return true;
        }
    }
    return false;
}

void callbackGetGestureSkeletonFinder(const vision_msgs::Skeletons& msg)
{
    vision_msgs::Skeletons skeletons;
    vision_msgs::Skeleton skeleton;

    skeletons = msg;

    vision_msgs::GestureSkeletons gestures_detected;

    while (!skeletons.skeletons.empty())
    {
        skeleton = skeletons.skeletons.back();
        int indexRightHand, indexRightHip, indexLeftHand, indexLeftHip, indexNeck, indexTorso;
        bool foundRightHand, foundRightHip, foundLeftHand, foundLeftHip, foundNeck, foundTorso;
        foundRightHand = findIndexJoint(skeleton, "right_hand", indexRightHand);
        foundRightHip = findIndexJoint(skeleton, "right_hip", indexRightHip);
        foundLeftHand = findIndexJoint(skeleton, "left_hand", indexLeftHand);
        foundLeftHip = findIndexJoint(skeleton, "left_hip", indexLeftHip);
        foundNeck = findIndexJoint(skeleton, "neck", indexNeck);
        foundTorso = findIndexJoint(skeleton, "torso", indexTorso);

        if(foundRightHand && foundRightHip && foundNeck && foundTorso && 
                skeleton.joints[indexRightHand].position.y > (skeleton.joints[indexRightHip].position.y + 0.20) && 
                skeleton.joints[indexRightHand].position.z > skeleton.joints[indexRightHip].position.z && 
                skeleton.joints[indexRightHand].position.z < skeleton.joints[indexNeck].position.z)
        {
            vision_msgs::GestureSkeleton gesture_detected;

            gesture_detected.id = skeleton.user_id;
            gesture_detected.gesture = "pointing_right";
            gesture_detected.gesture_centroid.x = skeleton.joints[indexTorso].position.x;
            gesture_detected.gesture_centroid.y = skeleton.joints[indexTorso].position.y;
            gesture_detected.gesture_centroid.z = skeleton.joints[indexTorso].position.z;
            //pubGesture.publish(gesture_detected);
            gestures_detected.recog_gestures.push_back(gesture_detected);
            std::cout << "User: " << skeleton.user_id << " Pointing right" << std::endl;
        }
        else
            std::cout << "User: " << skeleton.user_id << " Can not compute the gesture pointing right" << std::endl;

        if(foundLeftHand && foundLeftHip && foundNeck && foundTorso &&
                skeleton.joints[indexLeftHand].position.y < (skeleton.joints[indexLeftHip].position.y - 0.20) && 
                skeleton.joints[indexLeftHand].position.z > skeleton.joints[indexLeftHip].position.z && 
                skeleton.joints[indexLeftHand].position.z < skeleton.joints[indexNeck].position.z)
        {
            vision_msgs::GestureSkeleton gesture_detected;

            gesture_detected.id = skeleton.user_id;
            gesture_detected.gesture = "pointing_left";
            gesture_detected.gesture_centroid.x = skeleton.joints[indexTorso].position.x;
            gesture_detected.gesture_centroid.y = skeleton.joints[indexTorso].position.y;
            gesture_detected.gesture_centroid.z = skeleton.joints[indexTorso].position.z;
            //pubGesture.publish(gesture_detected);
            gestures_detected.recog_gestures.push_back(gesture_detected);
            std::cout << "User: " << skeleton.user_id << " Pointing left" << std::endl;
        }
        else
            std::cout << "User: " << skeleton.user_id << " Can not compute the gesture pointing left" << std::endl;

        if(foundRightHand && foundNeck && foundTorso && 
                skeleton.joints[indexRightHand].position.z > skeleton.joints[indexNeck].position.z)
        {
            vision_msgs::GestureSkeleton gesture_detected;

            gesture_detected.id = skeleton.user_id;
            gesture_detected.gesture = "right_hand_rised";
            gesture_detected.gesture_centroid.x = skeleton.joints[indexTorso].position.x;
            gesture_detected.gesture_centroid.y = skeleton.joints[indexTorso].position.y;
            gesture_detected.gesture_centroid.z = skeleton.joints[indexTorso].position.z;
            //pubGesture.publish(gesture_detected);
            gestures_detected.recog_gestures.push_back(gesture_detected);
            std::cout << "User: " << skeleton.user_id << " Right hand rised" << std::endl;
        }
        else
            std::cout << "User: " << skeleton.user_id << " Can not compute the gesture Right hand rised" << std::endl;

        if(foundLeftHand && foundNeck && foundTorso &&
                skeleton.joints[indexLeftHand].position.z  > skeleton.joints[indexNeck].position.z)
        {
            vision_msgs::GestureSkeleton gesture_detected;

            gesture_detected.id = skeleton.user_id;
            gesture_detected.gesture = "left_hand_rised";
            gesture_detected.gesture_centroid.x = skeleton.joints[indexTorso].position.x;
            gesture_detected.gesture_centroid.y = skeleton.joints[indexTorso].position.y;
            gesture_detected.gesture_centroid.z = skeleton.joints[indexTorso].position.z;
            //pubGesture.publish(gesture_detected);
            gestures_detected.recog_gestures.push_back(gesture_detected);
            std::cout << "User: " << skeleton.user_id << " Left hand rised" << std::endl;
        }
        else
            std::cout << "User: " << skeleton.user_id << " Can not compute the gesture Left hand rised" << std::endl;

        skeletons.skeletons.pop_back();
    }
    pubGestures.publish(gestures_detected);
}

void callbackGetRHandPosSkeletonFinder(const vision_msgs::Skeletons& msg)
{
    vision_msgs::Skeletons skeletons;

    skeletons = msg;
    geometry_msgs::Point handCentroid;

    vision_msgs::HandSkeletonPos hands_pos;

    for(int i = 0; i < skeletons.skeletons.size(); i++){
        vision_msgs::Skeleton skeleton = skeletons.skeletons[i];
        int indexRightHand;
        bool foundRightHand;
        foundRightHand = findIndexJoint(skeleton, "right_hand", indexRightHand);
        if(foundRightHand){
            handCentroid.x = skeleton.joints[indexRightHand].position.x;
            handCentroid.y = skeleton.joints[indexRightHand].position.y;
            handCentroid.z = skeleton.joints[indexRightHand].position.z;
            hands_pos.hands_position.push_back(handCentroid);
        }
        else
            std::cout << "User: " << skeleton.user_id << " Can not get right hand position." << std::endl;
    }

    pubRHnadPos.publish(hands_pos);
}

void callbackGetLHandPosSkeletonFinder(const vision_msgs::Skeletons& msg)
{
    vision_msgs::Skeletons skeletons;

    skeletons = msg;

    geometry_msgs::Point handCentroid;

    vision_msgs::HandSkeletonPos hands_pos;

    for(int i = 0; i < skeletons.skeletons.size(); i++){
        vision_msgs::Skeleton skeleton = skeletons.skeletons[i];
        int indexLeftHand;
        bool foundLeftHand;
        foundLeftHand = findIndexJoint(skeleton, "left_hand", indexLeftHand);
        if(foundLeftHand){
            handCentroid.x = skeleton.joints[indexLeftHand].position.x;
            handCentroid.y = skeleton.joints[indexLeftHand].position.y;
            handCentroid.z = skeleton.joints[indexLeftHand].position.z;
            hands_pos.hands_position.push_back(handCentroid);
        }
        else
            std::cout << "User: " << skeleton.user_id << " Can not get left hand position." << std::endl;
    }
    pubLHnadPos.publish(hands_pos);
}

void callbackGetTorsoPosSkeletonFinder(const vision_msgs::Skeletons& msg)
{
    vision_msgs::Skeletons skeletons;

    skeletons = msg;

    geometry_msgs::Point torsoCentroid;

    vision_msgs::HandSkeletonPos torso_pos;

    for(int i = 0; i < skeletons.skeletons.size(); i++){
        vision_msgs::Skeleton skeleton = skeletons.skeletons[i];
        int indexTorso;
        bool foundTorso;
        foundTorso = findIndexJoint(skeleton, "torso", indexTorso);
        if(foundTorso){
            torsoCentroid.x = skeleton.joints[indexTorso].position.x;
            torsoCentroid.y = skeleton.joints[indexTorso].position.y;
            torsoCentroid.z = skeleton.joints[indexTorso].position.z;
            torso_pos.hands_position.push_back(torsoCentroid);
        }
        else
            std::cout << "User: " << skeleton.user_id << " Can not get torso position." << std::endl;
    }
    pubTorsoPos.publish(torso_pos);
}


void callbackGetGestureOpenPose3D(const vision_msgs::Skeletons& msg){
    vision_msgs::Skeletons skeletons = msg;
    vision_msgs::GestureSkeletons gestures_detected;
    for(int i = 0; i < skeletons.skeletons.size(); i++){
        vision_msgs::Skeleton skeleton = skeletons.skeletons[i];
        int indexRightWrist, indexRightHip, indexLeftWrist, indexLeftHip, indexNeck;
        bool foundRightWrist, foundRightHip, foundLeftWrist, foundLeftHip, foundNeck;
        foundRightWrist = findIndexJoint(skeleton, "right_wrist", indexRightWrist);
        foundRightHip = findIndexJoint(skeleton, "right_hip", indexRightHip);
        foundLeftWrist = findIndexJoint(skeleton, "left_wrist", indexLeftWrist);
        foundLeftHip = findIndexJoint(skeleton, "left_hip", indexLeftHip);
        foundNeck = findIndexJoint(skeleton, "neck", indexNeck);
        if(foundRightWrist && foundRightHip && foundNeck && 
                skeleton.joints[indexRightWrist].position.y > (skeleton.joints[indexRightHip].position.y + 0.20) && 
                skeleton.joints[indexRightWrist].position.z > skeleton.joints[indexRightHip].position.z && 
                skeleton.joints[indexRightWrist].position.z < skeleton.joints[indexNeck].position.z){
            vision_msgs::GestureSkeleton gesture_detected;

            gesture_detected.id = skeleton.user_id;
            gesture_detected.gesture = "pointing_right";
            gesture_detected.gesture_centroid.x = skeleton.joints[indexNeck].position.x;
            gesture_detected.gesture_centroid.y = skeleton.joints[indexNeck].position.y;
            gesture_detected.gesture_centroid.z = skeleton.joints[indexNeck].position.z;
            //pubGesture.publish(gesture_detected);
            gestures_detected.recog_gestures.push_back(gesture_detected);
            std::cout << "User: " << skeleton.user_id << " Pointing right" << std::endl;
        }
        else if(!foundRightWrist || !foundRightHip || !foundNeck) 
            std::cout << "User: " << skeleton.user_id << " Can not compute the gesture pointing right" << std::endl;
    }
    pubGestures.publish(gestures_detected);
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING GESTURE RECOGNIZER SKELETONS..." << std::endl;
    ros::init(argc, argv, "gesture_recognizer");
    ros::NodeHandle n;
    int method = 0;
    if(ros::param::has("~gesture_method"))
        ros::param::get("~gesture_method", method);

    gestureMethod = GestureMethod(method);

    ros::Subscriber subGetGesture;
    ros::Subscriber subGetRHandPos;
    ros::Subscriber subGetLHandPos;
    ros::Subscriber subGetTorsoPos;

    switch(gestureMethod){
        case NITE:
            subGetGesture = n.subscribe("/vision/skeleton_finder/skeleton_recog", 1, callbackGetGestureSkeletonFinder);
            subGetRHandPos = n.subscribe("/vision/skeleton_finder/skeleton_recog", 1, callbackGetRHandPosSkeletonFinder);
            subGetLHandPos = n.subscribe("/vision/skeleton_finder/skeleton_recog", 1, callbackGetLHandPosSkeletonFinder);
            subGetTorsoPos = n.subscribe("/vision/skeleton_finder/skeleton_recog", 1, callbackGetTorsoPosSkeletonFinder);
            break;
        case OPENPOSE_3D:
            subGetGesture = n.subscribe("/vision/openpose/skeleton_recog", 1, callbackGetGestureOpenPose3D);
            break;
        default:
            break;
    }

    pubGestures = n.advertise<vision_msgs::GestureSkeletons> ("/vision/gesture_recog_skeleton/gesture_recog", 1);
    pubRHnadPos = n.advertise<vision_msgs::HandSkeletonPos> ("/vision/gesture_recog_skeleton/right_hand_pos", 1);
    pubLHnadPos = n.advertise<vision_msgs::HandSkeletonPos> ("/vision/gesture_recog_skeleton/left_hand_pos", 1);
    pubTorsoPos = n.advertise<vision_msgs::HandSkeletonPos> ("/vision/gesture_recog_skeleton/torso_pos", 1);

    ros::Rate loop(30);

    std::cout << "GestureRecognizer.->Running..." << std::endl;

    while(ros::ok())
    {
        loop.sleep();
        ros::spinOnce();
    }

    return 0;
}
