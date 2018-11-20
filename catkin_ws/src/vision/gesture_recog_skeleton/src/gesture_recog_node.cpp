#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <iostream>
#include <sstream>
#include <string>
#include <tuple>
#include "vision_msgs/Skeletons.h"
#include "vision_msgs/GestureSkeleton.h"
#include "vision_msgs/GestureSkeletons.h"
#include "geometry_msgs/Point.h"
#include "vision_msgs/HandSkeletonPos.h"
#include <visualization_msgs/Marker.h>
#include "justina_tools/JustinaVision.h"
#include "opencv2/opencv.hpp"
#include "justina_tools/JustinaTools.h"

using namespace cv;

enum GestureMethod{
    NITE, OPENPOSE_3D, OPENPOSE_2D
};

ros::Publisher pubGestures;
ros::Publisher pubRHnadPos;
ros::Publisher pubLHnadPos;
ros::Publisher pubTorsoPos;

GestureMethod gestureMethod;
    
vision_msgs::GestureSkeletons gestures_detected;
vision_msgs::Skeletons skeletons;

std::vector<std::pair<geometry_msgs::Point32, cv::Mat> > dataReco;
// std::vector<std::pair<geometry_msgs::Point32, std::vector<geometry_msgs::Point32> > > dataRecoRightWrist;
std::vector<std::tuple<geometry_msgs::Point32, std::vector<geometry_msgs::Point32>, ros::Time > > dataRecoRightWrist;
// std::vector<std::pair<geometry_msgs::Point32, std::vector<geometry_msgs::Point32> > > dataRecoLeftWrist;
std::vector<std::tuple<geometry_msgs::Point32, std::vector<geometry_msgs::Point32>, ros::Time > > dataRecoLeftWrist;

//This is only for the waving recognize gesture
ros::ServiceClient cltRgbdRobot;

bool findIndexJoint(const vision_msgs::Skeleton skeleton, std::string name_joint, int &indexJoint){
    for(int i = 0; i < skeleton.joints.size(); i++){
        if(skeleton.joints[i].name_joint.data.compare(name_joint) == 0){
            indexJoint = i;
            return true;
        }
    }
    return false;
}

bool GetImagesFromJustina( cv::Mat& imaBGR, cv::Mat& imaPCL)
{
    point_cloud_manager::GetRgbd srv;
    if(!cltRgbdRobot.call(srv))
    {
        std::cout << "CubesSegmentation.->Cannot get point cloud" << std::endl;
        return false;
    }
    JustinaTools::PointCloud2Msg_ToCvMat(srv.response.point_cloud, imaBGR, imaPCL);
    return true; 
}

float getEuclideanDistance(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2){
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
}

void callbackGetGestureSkeletonFinder(const vision_msgs::Skeletons& msg)
{
    vision_msgs::Skeleton skeleton;

    skeletons = msg;
    gestures_detected.recog_gestures.clear();

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
            gesture_detected.time = ros::Time::now(); 
            gesture_detected.gesture = "pointing_right";
            gesture_detected.gesture_centroid.x = skeleton.joints[indexTorso].position.x;
            gesture_detected.gesture_centroid.y = skeleton.joints[indexTorso].position.y;
            gesture_detected.gesture_centroid.z = skeleton.joints[indexTorso].position.z;
            //pubGesture.publish(gesture_detected);
            gestures_detected.recog_gestures.push_back(gesture_detected);
            std::cout << "User: " << skeleton.user_id << " Pointing right" << std::endl;
        }
        else if(!foundRightHand && !foundRightHip && !foundNeck && !foundTorso)
            std::cout << "User: " << skeleton.user_id << " Can not compute the gesture pointing right" << std::endl;

        if(foundLeftHand && foundLeftHip && foundNeck && foundTorso &&
                skeleton.joints[indexLeftHand].position.y < (skeleton.joints[indexLeftHip].position.y - 0.20) && 
                skeleton.joints[indexLeftHand].position.z > skeleton.joints[indexLeftHip].position.z && 
                skeleton.joints[indexLeftHand].position.z < skeleton.joints[indexNeck].position.z)
        {
            vision_msgs::GestureSkeleton gesture_detected;

            gesture_detected.id = skeleton.user_id;
            gesture_detected.time = ros::Time::now(); 
            gesture_detected.gesture = "pointing_left";
            gesture_detected.gesture_centroid.x = skeleton.joints[indexTorso].position.x;
            gesture_detected.gesture_centroid.y = skeleton.joints[indexTorso].position.y;
            gesture_detected.gesture_centroid.z = skeleton.joints[indexTorso].position.z;
            //pubGesture.publish(gesture_detected);
            gestures_detected.recog_gestures.push_back(gesture_detected);
            std::cout << "User: " << skeleton.user_id << " Pointing left" << std::endl;
        }
        else if(!foundLeftHand && !foundLeftHip && !foundNeck && !foundTorso)
            std::cout << "User: " << skeleton.user_id << " Can not compute the gesture pointing left" << std::endl;

        if(foundRightHand && foundNeck && foundTorso && 
                skeleton.joints[indexRightHand].position.z > skeleton.joints[indexNeck].position.z)
        {
            vision_msgs::GestureSkeleton gesture_detected;

            gesture_detected.id = skeleton.user_id;
            gesture_detected.time = ros::Time::now(); 
            gesture_detected.gesture = "right_hand_rised";
            gesture_detected.gesture_centroid.x = skeleton.joints[indexTorso].position.x;
            gesture_detected.gesture_centroid.y = skeleton.joints[indexTorso].position.y;
            gesture_detected.gesture_centroid.z = skeleton.joints[indexTorso].position.z;
            //pubGesture.publish(gesture_detected);
            gestures_detected.recog_gestures.push_back(gesture_detected);
            std::cout << "User: " << skeleton.user_id << " Right hand rised" << std::endl;
        }
        else if(!foundRightHand && !foundNeck && !foundTorso)
            std::cout << "User: " << skeleton.user_id << " Can not compute the gesture Right hand rised" << std::endl;

        if(foundLeftHand && foundNeck && foundTorso &&
                skeleton.joints[indexLeftHand].position.z  > skeleton.joints[indexNeck].position.z)
        {
            vision_msgs::GestureSkeleton gesture_detected;

            gesture_detected.id = skeleton.user_id;
            gesture_detected.time = ros::Time::now(); 
            gesture_detected.gesture = "left_hand_rised";
            gesture_detected.gesture_centroid.x = skeleton.joints[indexTorso].position.x;
            gesture_detected.gesture_centroid.y = skeleton.joints[indexTorso].position.y;
            gesture_detected.gesture_centroid.z = skeleton.joints[indexTorso].position.z;
            //pubGesture.publish(gesture_detected);
            gestures_detected.recog_gestures.push_back(gesture_detected);
            std::cout << "User: " << skeleton.user_id << " Left hand rised" << std::endl;
        }
        else if(!foundLeftHand && !foundNeck && !foundTorso)
            std::cout << "User: " << skeleton.user_id << " Can not compute the gesture Left hand rised" << std::endl;

        skeletons.skeletons.pop_back();
    }
}

void callbackGetRHandPosSkeletonFinder(const vision_msgs::Skeletons& msg)
{
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
    gestures_detected.recog_gestures.clear();
    skeletons = msg;
    for(int i = 0; i < skeletons.skeletons.size(); i++){
        vision_msgs::Skeleton skeleton = skeletons.skeletons[i];
        int indexRightWrist, indexHip, indexRightShoulder, indexRightElbow, indexLeftWrist, indexLeftShoulder, indexLeftElbow, indexNeck;
        bool foundRightWrist, foundHip, foundRightShoulder, foundRightElbow, foundLeftWrist, foundLeftShoulder, foundLeftElbow, foundNeck;
        foundRightWrist = findIndexJoint(skeleton, "right_wrist", indexRightWrist);
        foundHip = findIndexJoint(skeleton, "hip", indexHip);
        foundRightShoulder = findIndexJoint(skeleton, "right_shoulder", indexRightShoulder);
        foundRightElbow = findIndexJoint(skeleton, "right_elbow", indexRightElbow);
        foundLeftWrist = findIndexJoint(skeleton, "left_wrist", indexLeftWrist);
        foundLeftShoulder = findIndexJoint(skeleton, "left_shoulder", indexLeftShoulder);
        foundLeftElbow = findIndexJoint(skeleton, "left_elbow", indexLeftElbow);
        foundNeck = findIndexJoint(skeleton, "neck", indexNeck);
        /*if(foundRightWrist && foundRightHip && foundNeck && 
                skeleton.joints[indexRightWrist].position.y > (skeleton.joints[indexRightHip].position.y + 0.20) && 
                skeleton.joints[indexRightWrist].position.z > skeleton.joints[indexRightHip].position.z && 
                skeleton.joints[indexRightWrist].position.z < (skeleton.joints[indexNeck].position.z + 0.15)){
            vision_msgs::GestureSkeleton gesture_detected;

            gesture_detected.id = skeleton.user_id;
            gesture_detected.time = ros::Time::now(); 
            gesture_detected.gesture = "pointing_right";
            gesture_detected.gesture_centroid.x = skeleton.joints[indexNeck].position.x;
            gesture_detected.gesture_centroid.y = skeleton.joints[indexNeck].position.y;
            gesture_detected.gesture_centroid.z = skeleton.joints[indexNeck].position.z;
            //pubGesture.publish(gesture_detected);
            gestures_detected.recog_gestures.push_back(gesture_detected);
            //std::cout << "User: " << skeleton.user_id << " Pointing right" << std::endl;
        }*/
        /*else if(!foundRightWrist || !foundRightHip || !foundNeck) 
            std::cout << "User: " << skeleton.user_id << " Can not compute the gesture pointing right" << std::endl;*/
        if(foundRightWrist && foundRightShoulder && foundRightElbow && foundNeck){
            float dx_neck_shoulder = skeleton.joints[indexRightShoulder].position.x - skeleton.joints[indexNeck].position.x;
            float dy_neck_shoulder = skeleton.joints[indexRightShoulder].position.y - skeleton.joints[indexNeck].position.y;
            float dz_neck_shoulder = skeleton.joints[indexRightShoulder].position.z - skeleton.joints[indexNeck].position.z;
            float dx_shoulder_elbow = skeleton.joints[indexRightElbow].position.x - skeleton.joints[indexRightShoulder].position.x;
            float dy_shoulder_elbow = skeleton.joints[indexRightElbow].position.y - skeleton.joints[indexRightShoulder].position.y;
            float dz_shoulder_elbow = skeleton.joints[indexRightElbow].position.z - skeleton.joints[indexRightShoulder].position.z;
            float dx_elbow_wrist = skeleton.joints[indexRightWrist].position.x - skeleton.joints[indexRightElbow].position.x;
            float dy_elbow_wrist = skeleton.joints[indexRightWrist].position.y - skeleton.joints[indexRightElbow].position.y;
            float dz_elbow_wrist = skeleton.joints[indexRightWrist].position.z - skeleton.joints[indexRightElbow].position.z;
            
            float dot = - dx_neck_shoulder * dx_shoulder_elbow - dy_neck_shoulder * dy_shoulder_elbow - dz_neck_shoulder *  dz_shoulder_elbow;
            float mod1 = sqrt(pow(dx_neck_shoulder, 2) + pow(dy_neck_shoulder, 2) + pow(dz_neck_shoulder, 2)); 
            float mod2 = sqrt(pow(dx_shoulder_elbow, 2) + pow(dy_shoulder_elbow, 2) + pow(dz_shoulder_elbow, 2));
            float angle1 = acos(dot / (mod1 * mod2));
            //std::cout << "User:" << skeleton.user_id << ", gesture_recog_node.->Angle1:" << angle1 << std::endl;
            dot = -dx_shoulder_elbow * dx_elbow_wrist - dy_shoulder_elbow * dy_elbow_wrist - dz_shoulder_elbow * dz_elbow_wrist;
            mod1 = sqrt(pow(dx_shoulder_elbow, 2) + pow(dy_shoulder_elbow, 2) + pow(dz_shoulder_elbow, 2)); 
            mod2 = sqrt(pow(dx_elbow_wrist, 2) + pow(dy_elbow_wrist, 2) + pow(dz_elbow_wrist, 2));
            float angle2 = acos(dot / (mod1 * mod2));
            //std::cout << "User:" << skeleton.user_id << ", gesture_recog_node.->Angle2:" << angle2 << std::endl;
            //if(angle1 >= 2.5 && angle1 <= M_PI && angle2 >= 2.6 && angle2 <= 2.9){
            if(angle1 >= 2.4 && angle1 <= 2.7 && angle2 >= 2.3 && angle2 <= M_PI){
                vision_msgs::GestureSkeleton gesture_detected;
                gesture_detected.id = skeleton.user_id;
                gesture_detected.time = ros::Time::now(); 
                gesture_detected.gesture = "pointing_right";
                gesture_detected.gesture_centroid.x = skeleton.joints[indexNeck].position.x;
                gesture_detected.gesture_centroid.y = skeleton.joints[indexNeck].position.y;
                gesture_detected.gesture_centroid.z = skeleton.joints[indexNeck].position.z;
                //pubGesture.publish(gesture_detected);
                gestures_detected.recog_gestures.push_back(gesture_detected);
                //std::cout << "User: " << skeleton.user_id << " Pointing right" << std::endl;
            }
        }
        /*else if(!foundRightWrist || !foundRightHip || !foundNeck) 
          std::cout << "User: " << skeleton.user_id << " Can not compute the gesture pointing right" << std::endl;*/

        /*if(foundLeftWrist && foundLeftHip && foundNeck &&
                skeleton.joints[indexLeftWrist].position.y < (skeleton.joints[indexLeftHip].position.y - 0.20) && 
                skeleton.joints[indexLeftWrist].position.z > skeleton.joints[indexLeftHip].position.z &&
                skeleton.joints[indexLeftWrist].position.z < (skeleton.joints[indexNeck].position.z + 0.15)){
            vision_msgs::GestureSkeleton gesture_detected;

            gesture_detected.id = skeleton.user_id;
            gesture_detected.time = ros::Time::now(); 
            gesture_detected.gesture = "pointing_left";
            gesture_detected.gesture_centroid.x = skeleton.joints[indexNeck].position.x;
            gesture_detected.gesture_centroid.y = skeleton.joints[indexNeck].position.y;
            gesture_detected.gesture_centroid.z = skeleton.joints[indexNeck].position.z;
            //pubGesture.publish(gesture_detected);
            gestures_detected.recog_gestures.push_back(gesture_detected);
            //std::cout << "User: " << skeleton.user_id << " Pointing left" << std::endl;
        }*/
        if(foundLeftWrist && foundLeftShoulder && foundLeftElbow && foundNeck){
            float dx_neck_shoulder = skeleton.joints[indexLeftShoulder].position.x - skeleton.joints[indexNeck].position.x;
            float dy_neck_shoulder = skeleton.joints[indexLeftShoulder].position.y - skeleton.joints[indexNeck].position.y;
            float dz_neck_shoulder = skeleton.joints[indexLeftShoulder].position.z - skeleton.joints[indexNeck].position.z;
            float dx_shoulder_elbow = skeleton.joints[indexLeftElbow].position.x - skeleton.joints[indexLeftShoulder].position.x;
            float dy_shoulder_elbow = skeleton.joints[indexLeftElbow].position.y - skeleton.joints[indexLeftShoulder].position.y;
            float dz_shoulder_elbow = skeleton.joints[indexLeftElbow].position.z - skeleton.joints[indexLeftShoulder].position.z;
            float dx_elbow_wrist = skeleton.joints[indexLeftWrist].position.x - skeleton.joints[indexLeftElbow].position.x;
            float dy_elbow_wrist = skeleton.joints[indexLeftWrist].position.y - skeleton.joints[indexLeftElbow].position.y;
            float dz_elbow_wrist = skeleton.joints[indexLeftWrist].position.z - skeleton.joints[indexLeftElbow].position.z;
            
            float dot = - dx_neck_shoulder * dx_shoulder_elbow - dy_neck_shoulder * dy_shoulder_elbow - dz_neck_shoulder *  dz_shoulder_elbow;
            float mod1 = sqrt(pow(dx_neck_shoulder, 2) + pow(dy_neck_shoulder, 2) + pow(dz_neck_shoulder, 2)); 
            float mod2 = sqrt(pow(dx_shoulder_elbow, 2) + pow(dy_shoulder_elbow, 2) + pow(dz_shoulder_elbow, 2));
            float angle1 = acos(dot / (mod1 * mod2));
            //std::cout << "User:" << skeleton.user_id << ", gesture_recog_node.->Angle1:" << angle1 << std::endl;
            dot = -dx_shoulder_elbow * dx_elbow_wrist - dy_shoulder_elbow * dy_elbow_wrist - dz_shoulder_elbow * dz_elbow_wrist;
            mod1 = sqrt(pow(dx_shoulder_elbow, 2) + pow(dy_shoulder_elbow, 2) + pow(dz_shoulder_elbow, 2)); 
            mod2 = sqrt(pow(dx_elbow_wrist, 2) + pow(dy_elbow_wrist, 2) + pow(dz_elbow_wrist, 2));
            float angle2 = acos(dot / (mod1 * mod2));
            //std::cout << "User:" << skeleton.user_id << ", gesture_recog_node.->Angle2:" << angle2 << std::endl;
            //if(angle1 >= 2.5 && angle1 <= M_PI && angle2 >= 2.6 && angle2 <= 2.9){
            if(angle1 >= 2.4 && angle1 <= 2.7 && angle2 >= 2.3 && angle2 <= M_PI){
                vision_msgs::GestureSkeleton gesture_detected;
                gesture_detected.id = skeleton.user_id;
                gesture_detected.time = ros::Time::now(); 
                gesture_detected.gesture = "pointing_left";
                gesture_detected.gesture_centroid.x = skeleton.joints[indexNeck].position.x;
                gesture_detected.gesture_centroid.y = skeleton.joints[indexNeck].position.y;
                gesture_detected.gesture_centroid.z = skeleton.joints[indexNeck].position.z;
                //pubGesture.publish(gesture_detected);
                gestures_detected.recog_gestures.push_back(gesture_detected);
                //std::cout << "User: " << skeleton.user_id << " Pointing right" << std::endl;
            }
        }
        /*else if(!foundLeftWrist && !foundLeftHip && !foundNeck)
            std::cout << "User: " << skeleton.user_id << " Can not compute the gesture pointing left" << std::endl;*/

        if(foundRightWrist && foundNeck &&
                skeleton.joints[indexRightWrist].position.z > (skeleton.joints[indexNeck].position.z + 0.10)){
            vision_msgs::GestureSkeleton gesture_detected;

            gesture_detected.id = skeleton.user_id;
            gesture_detected.time = ros::Time::now(); 
            gesture_detected.gesture = "right_hand_rised";
            gesture_detected.gesture_centroid.x = skeleton.joints[indexNeck].position.x;
            gesture_detected.gesture_centroid.y = skeleton.joints[indexNeck].position.y;
            gesture_detected.gesture_centroid.z = skeleton.joints[indexNeck].position.z;
            //pubGesture.publish(gesture_detected);
            gestures_detected.recog_gestures.push_back(gesture_detected);
            //std::cout << "User: " << skeleton.user_id << " Right hand rised" << std::endl;
        }
        /*else if(!foundRightWrist && !foundNeck)
            std::cout << "User: " << skeleton.user_id << " Can not compute the gesture Right hand rised" << std::endl;*/

        if(foundLeftWrist && foundNeck && 
                skeleton.joints[indexLeftWrist].position.z  > (skeleton.joints[indexNeck].position.z + 0.10)){
            vision_msgs::GestureSkeleton gesture_detected;

            gesture_detected.id = skeleton.user_id;
            gesture_detected.time = ros::Time::now(); 
            gesture_detected.gesture = "left_hand_rised";
            gesture_detected.gesture_centroid.x = skeleton.joints[indexNeck].position.x;
            gesture_detected.gesture_centroid.y = skeleton.joints[indexNeck].position.y;
            gesture_detected.gesture_centroid.z = skeleton.joints[indexNeck].position.z;
            //pubGesture.publish(gesture_detected);
            gestures_detected.recog_gestures.push_back(gesture_detected);
            //std::cout << "User: " << skeleton.user_id << " Left hand rised" << std::endl;
        }
        /*else if(!foundLeftWrist && !foundNeck)
            std::cout << "User: " << skeleton.user_id << " Can not compute the gesture Left hand rised" << std::endl;*/

        if(foundRightWrist && foundNeck &&
            skeleton.joints[indexRightWrist].position.z < skeleton.joints[indexNeck].position.z &&
            skeleton.joints[indexRightWrist].position.x < skeleton.joints[indexNeck].position.x - 0.10){
            vision_msgs::GestureSkeleton gesture_detected;

            gesture_detected.id = skeleton.user_id;
            gesture_detected.time = ros::Time::now(); 
            gesture_detected.gesture = "pointing_right_to_robot";
            gesture_detected.gesture_centroid.x = skeleton.joints[indexRightWrist].position.x;
            gesture_detected.gesture_centroid.y = skeleton.joints[indexRightWrist].position.y;
            gesture_detected.gesture_centroid.z = skeleton.joints[indexRightWrist].position.z;

            gestures_detected.recog_gestures.push_back(gesture_detected);
            //std::cout << "User: " << skeleton.user_id << " Pointing right to robot" << std::endl;
        }
        /*else if(!foundRightWrist && !foundNeck)
            std::cout << "User: " << skeleton.user_id << " Cannot compute the gesture Pointing right to robot" << std::endl;*/

        if(foundLeftWrist && foundNeck &&
            skeleton.joints[indexLeftWrist].position.z < skeleton.joints[indexNeck].position.z &&
            skeleton.joints[indexLeftWrist].position.x < skeleton.joints[indexNeck].position.x - 0.10){
            vision_msgs::GestureSkeleton gesture_detected;

            gesture_detected.id = skeleton.user_id;
            gesture_detected.time = ros::Time::now(); 
            gesture_detected.gesture = "pointing_left_to_robot";
            gesture_detected.gesture_centroid.x = skeleton.joints[indexLeftWrist].position.x;
            gesture_detected.gesture_centroid.y = skeleton.joints[indexLeftWrist].position.y;
            gesture_detected.gesture_centroid.z = skeleton.joints[indexLeftWrist].position.z;

            gestures_detected.recog_gestures.push_back(gesture_detected);
            //std::cout << "User: " << skeleton.user_id << " Pointing left to robot" << std::endl;
        }
        /*else if(!foundLeftWrist && !foundNeck)
            std::cout << "User: " << skeleton.user_id << " Cannot compute the gesture Pointing left to robot" << std::endl;*/

    }
}

void callbackGetGestureOpenPose2D(const vision_msgs::Skeletons& msg){
    gestures_detected.recog_gestures.clear();
    skeletons = msg;
    for(int i = 0; i < skeletons.skeletons.size(); i++){
        vision_msgs::Skeleton skeleton = skeletons.skeletons[i];
        int indexRightWrist, indexHip, indexRightShoulder, indexRightElbow, indexLeftWrist, indexLeftShoulder, indexLeftElbow, indexNeck, indexNose;
        bool foundRightWrist, foundHip, foundRightShoulder, foundRightElbow, foundLeftWrist, foundLeftShoulder, foundLeftElbow, foundNeck, foundNose;
        foundRightWrist = findIndexJoint(skeleton, "right_wrist", indexRightWrist);
        foundHip = findIndexJoint(skeleton, "hip", indexHip);
        foundNose = findIndexJoint(skeleton, "nose", indexNose);
        foundRightShoulder = findIndexJoint(skeleton, "right_shoulder", indexRightShoulder);
        foundRightElbow = findIndexJoint(skeleton, "right_elbow", indexRightElbow);
        foundLeftWrist = findIndexJoint(skeleton, "left_wrist", indexLeftWrist);
        foundLeftShoulder = findIndexJoint(skeleton, "left_shoulder", indexLeftShoulder);
        foundLeftElbow = findIndexJoint(skeleton, "left_elbow", indexLeftElbow);
        foundNeck = findIndexJoint(skeleton, "neck", indexNeck);
        if(foundRightWrist && (foundHip || foundNose) && foundRightShoulder && foundRightElbow && foundNeck){
            int dx_neck_wrist = skeleton.joints[indexRightWrist].position.x - skeleton.joints[indexNeck].position.x;
            int dy_neck_wrist = skeleton.joints[indexRightWrist].position.y - skeleton.joints[indexNeck].position.y;
            int dx_neck_ref;
            int dy_neck_ref;
            if(foundHip){
                dx_neck_ref = skeleton.joints[indexHip].position.x - skeleton.joints[indexNeck].position.x;
                dx_neck_ref = skeleton.joints[indexHip].position.x - skeleton.joints[indexNeck].position.x;
            }
            else{
                dy_neck_ref = skeleton.joints[indexNose].position.y - skeleton.joints[indexNeck].position.y;
                dy_neck_ref = skeleton.joints[indexNose].position.y - skeleton.joints[indexNeck].position.y;
            }
            int dx_elbow_wrist = skeleton.joints[indexRightWrist].position.x - skeleton.joints[indexRightElbow].position.x;
            int dy_elbow_wrist = skeleton.joints[indexRightWrist].position.y - skeleton.joints[indexRightElbow].position.y;
            int dx_elbow_shoulder = skeleton.joints[indexRightShoulder].position.x - skeleton.joints[indexRightElbow].position.x;
            int dy_elbow_shoulder = skeleton.joints[indexRightShoulder].position.y - skeleton.joints[indexRightElbow].position.y;
            float dot = dx_neck_wrist * dx_neck_ref + dy_neck_wrist * dy_neck_ref;
            float mod1 = sqrt(pow(dx_neck_wrist, 2) + pow(dy_neck_wrist, 2)); 
            float mod2 = sqrt(pow(dx_neck_ref, 2) + pow(dy_neck_ref, 2));
            float angle1 = acos(dot / (mod1 * mod2));
            //std::cout << "User:" << skeleton.user_id << ", gesture_recog_node.->Angle1:" << angle1 << std::endl;
            dot = dx_elbow_wrist * dx_elbow_shoulder + dy_elbow_wrist * dy_elbow_shoulder;
            mod1 = sqrt(pow(dx_elbow_wrist, 2) + pow(dy_elbow_wrist, 2)); 
            mod2 = sqrt(pow(dx_elbow_shoulder, 2) + pow(dy_elbow_shoulder, 2));
            float angle2 = acos(dot / (mod1 * mod2));
            //std::cout << "User:" << skeleton.user_id << ", gesture_recog_node.->Angle2:" << angle2 << std::endl;
            if(angle1 >= 1.0 && angle1 <= 1.5708 && angle2 >= 1.9 && angle2 <= 3.1416){
                vision_msgs::GestureSkeleton gesture_detected;
                gesture_detected.id = skeleton.user_id;
                gesture_detected.time = ros::Time::now(); 
                gesture_detected.gesture = "pointing_right";
                gesture_detected.gesture_centroid.x = skeleton.ref_point.x;
                gesture_detected.gesture_centroid.y = skeleton.ref_point.y;
                gesture_detected.gesture_centroid.z = skeleton.ref_point.z;
                //pubGesture.publish(gesture_detected);
                gestures_detected.recog_gestures.push_back(gesture_detected);
                std::cout << "User: " << skeleton.user_id << " Pointing right" << std::endl;
            }
        }
        else if(!foundRightWrist || !foundHip || !foundNose || !foundRightShoulder || !foundRightElbow || !foundNeck) 
            std::cout << "User: " << skeleton.user_id << " Can not compute the gesture pointing right" << std::endl;

        if(foundLeftWrist && (foundHip || foundNose) && foundLeftShoulder && foundLeftElbow && foundNeck){
            int dx_neck_wrist = skeleton.joints[indexLeftWrist].position.x - skeleton.joints[indexNeck].position.x;
            int dy_neck_wrist = skeleton.joints[indexLeftWrist].position.y - skeleton.joints[indexNeck].position.y;
            int dx_neck_ref;
            int dy_neck_ref;
            if(foundHip){
                dx_neck_ref = skeleton.joints[indexHip].position.x - skeleton.joints[indexNeck].position.x;
                dx_neck_ref = skeleton.joints[indexHip].position.x - skeleton.joints[indexNeck].position.x;
            }
            else{
                dy_neck_ref = skeleton.joints[indexNose].position.y - skeleton.joints[indexNeck].position.y;
                dy_neck_ref = skeleton.joints[indexNose].position.y - skeleton.joints[indexNeck].position.y;
            }
            int dx_elbow_wrist = skeleton.joints[indexLeftWrist].position.x - skeleton.joints[indexLeftElbow].position.x;
            int dy_elbow_wrist = skeleton.joints[indexLeftWrist].position.y - skeleton.joints[indexLeftElbow].position.y;
            int dx_elbow_shoulder = skeleton.joints[indexLeftShoulder].position.x - skeleton.joints[indexLeftElbow].position.x;
            int dy_elbow_shoulder = skeleton.joints[indexLeftShoulder].position.y - skeleton.joints[indexLeftElbow].position.y;
            float dot = dx_neck_wrist * dx_neck_ref + dy_neck_wrist * dy_neck_ref;
            float mod1 = sqrt(pow(dx_neck_wrist, 2) + pow(dy_neck_wrist, 2)); 
            float mod2 = sqrt(pow(dx_neck_ref, 2) + pow(dy_neck_ref, 2));
            float angle1 = acos(dot / (mod1 * mod2));
            //std::cout << "User:" << skeleton.user_id << ", gesture_recog_node.->Angle1:" << angle1 << std::endl;
            dot = dx_elbow_wrist * dx_elbow_shoulder + dy_elbow_wrist * dy_elbow_shoulder;
            mod1 = sqrt(pow(dx_elbow_wrist, 2) + pow(dy_elbow_wrist, 2)); 
            mod2 = sqrt(pow(dx_elbow_shoulder, 2) + pow(dy_elbow_shoulder, 2));
            float angle2 = acos(dot / (mod1 * mod2));
            //std::cout << "User:" << skeleton.user_id << ", gesture_recog_node.->Angle2:" << angle2 << std::endl;
            if(angle1 >= 1.0 && angle1 <= 1.5708 && angle2 >= 1.9 && angle2 <= 3.1416){
                vision_msgs::GestureSkeleton gesture_detected;
                gesture_detected.id = skeleton.user_id;
                gesture_detected.time = ros::Time::now(); 
                gesture_detected.gesture = "pointing_left";
                gesture_detected.gesture_centroid.x = skeleton.ref_point.x;
                gesture_detected.gesture_centroid.y = skeleton.ref_point.y;
                gesture_detected.gesture_centroid.z = skeleton.ref_point.z;
                //pubGesture.publish(gesture_detected);
                gestures_detected.recog_gestures.push_back(gesture_detected);
                std::cout << "User: " << skeleton.user_id << " Pointing left" << std::endl;
            }
        }
        else if(!foundLeftWrist || !foundHip || !foundNose || !foundLeftShoulder || !foundLeftElbow || !foundNeck) 
            std::cout << "User: " << skeleton.user_id << " Can not compute the gesture pointing left" << std::endl;

        if(foundRightWrist && foundNeck &&
                skeleton.joints[indexRightWrist].position.y < skeleton.joints[indexNeck].position.y){
            vision_msgs::GestureSkeleton gesture_detected;

            gesture_detected.id = skeleton.user_id;
            gesture_detected.time = ros::Time::now(); 
            gesture_detected.gesture = "right_hand_rised";
            gesture_detected.gesture_centroid.x = skeleton.ref_point.x;
            gesture_detected.gesture_centroid.y = skeleton.ref_point.y;
            gesture_detected.gesture_centroid.z = skeleton.ref_point.z;
            //pubGesture.publish(gesture_detected);
            gestures_detected.recog_gestures.push_back(gesture_detected);
            std::cout << "User: " << skeleton.user_id << " Right hand rised" << std::endl;
        }
        else if(!foundRightWrist && !foundNeck)
            std::cout << "User: " << skeleton.user_id << " Can not compute the gesture Right hand rised" << std::endl;

        if(foundLeftWrist && foundNeck && 
                skeleton.joints[indexLeftWrist].position.y  < skeleton.joints[indexNeck].position.y){
            vision_msgs::GestureSkeleton gesture_detected;

            gesture_detected.id = skeleton.user_id;
            gesture_detected.time = ros::Time::now(); 
            gesture_detected.gesture = "left_hand_rised";
            gesture_detected.gesture_centroid.x = skeleton.ref_point.x;
            gesture_detected.gesture_centroid.y = skeleton.ref_point.y;
            gesture_detected.gesture_centroid.z = skeleton.ref_point.z;
            //pubGesture.publish(gesture_detected);
            gestures_detected.recog_gestures.push_back(gesture_detected);
            std::cout << "User: " << skeleton.user_id << " Left hand rised" << std::endl;
        }
        else if(!foundLeftWrist && !foundNeck)
            std::cout << "User: " << skeleton.user_id << " Can not compute the gesture Left hand rised" << std::endl;
    }
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING GESTURE RECOGNIZER SKELETONS..." << std::endl;
    ros::init(argc, argv, "gesture_recognizer");
    ros::NodeHandle n;
    int method = 0;
    int numFrames; // 20
    float thrWavingMotion; // 0.8 // 0.5
    float thrToPerson;
    double timeElapsedRemove;
    if(ros::param::has("~gesture_method")){
        ros::param::get("~gesture_method", method);
        gestureMethod = GestureMethod(method);
    }
    else
        gestureMethod = GestureMethod(0);
    if(ros::param::has("~num_frames"))
        ros::param::get("~num_frames", numFrames);
    if(ros::param::has("~thr_waving_motion"))
        ros::param::get("~thr_waving_motion", thrWavingMotion);
    if(ros::param::has("~thr_to_person"))
        ros::param::get("~thr_to_person", thrToPerson);
    if(ros::param::has("~time_elapsed_remove"))
        ros::param::get("~time_elapsed_remove", timeElapsedRemove);

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
        case OPENPOSE_2D:
            subGetGesture = n.subscribe("/vision/openpose/skeleton_recog_2D", 1, callbackGetGestureOpenPose2D);
            break;
        default:
            std::cout << "gesture_recog_node.->Method is not available for gesture recog node.";
            break;
    }

    pubGestures = n.advertise<vision_msgs::GestureSkeletons> ("/vision/gesture_recog_skeleton/gesture_recog", 1);
    pubRHnadPos = n.advertise<vision_msgs::HandSkeletonPos> ("/vision/gesture_recog_skeleton/right_hand_pos", 1);
    pubLHnadPos = n.advertise<vision_msgs::HandSkeletonPos> ("/vision/gesture_recog_skeleton/left_hand_pos", 1);
    pubTorsoPos = n.advertise<vision_msgs::HandSkeletonPos> ("/vision/gesture_recog_skeleton/torso_pos", 1);

    cltRgbdRobot = n.serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot");

    ros::Rate loop(30);

    std::cout << "GestureRecognizer.->Running..." << std::endl;

    while(ros::ok() && cv::waitKey(1) != 'q')
    {
        /*if(gestures_detected.recog_gestures.size() > 0){
            cv::Mat bgrImg;
            cv::Mat xyzCloud;
            cv::Mat globalmask;
            GetImagesFromJustina(bgrImg,xyzCloud);
            for(int i = 0; i < gestures_detected.recog_gestures.size(); i++){
                if(gestures_detected.recog_gestures[i].gesture.compare("right_hand_rised") == 0 ||  gestures_detected.recog_gestures[i].gesture.compare("left_hand_rised") == 0){
                    int indexUser = -1;
                    for(int j = 0; j < skeletons.skeletons.size(); j++){
                        vision_msgs::Skeleton ske = skeletons.skeletons[j];
                        if(ske.user_id == gestures_detected.recog_gestures[i].id){
                            indexUser = j;
                            break;
                        }
                    }
                    if(indexUser < 0)
                        continue;
                         
                    if(gestures_detected.recog_gestures[i].gesture.compare("right_hand_rised") == 0){
                        vision_msgs::Skeleton ske = skeletons.skeletons[indexUser];
                        int indexRightWrist = 0;
                        bool foundRightWrist = findIndexJoint(ske, "right_wrist", indexRightWrist);
                        if(!foundRightWrist)
                            continue;
                        float minX = ske.joints[indexRightWrist].position.x - 0.125;
                        float maxX = ske.joints[indexRightWrist].position.x + 0.125;
                        float minY = ske.joints[indexRightWrist].position.y - 0.15;
                        float maxY = ske.joints[indexRightWrist].position.y + 0.15;
                        float minZ = ske.joints[indexRightWrist].position.z - 0.1;
                        float maxZ = ske.joints[indexRightWrist].position.z + 0.1;
                        
                        cv::Mat maskXYZ;
                        cv::Mat validImage;
                        cv::Mat validImageGray;
                        cv::Mat bgrImgCopy;
                        cv::inRange(xyzCloud,cv::Scalar(minX, minY,minZ),cv::Scalar(maxX,maxY,maxZ),maskXYZ);
                        bgrImg.copyTo(validImage, maskXYZ);
                        cv::cvtColor(validImage, validImageGray, CV_BGR2GRAY);
                        
                        bool foundDataReco = false;
                        int indexFound = 0;
                        for(int k = 0; k < dataReco.size() && !foundDataReco; k++){
                            float distance = getEuclideanDistance(dataReco[k].first, ske.ref_point); 
                            if(distance <= 0.3){
                                indexFound = k;
                                foundDataReco = true;
                            }
                        }
                        if(!foundDataReco){
                            std::pair<geometry_msgs::Point32, cv::Mat> pair = std::make_pair(ske.ref_point, validImageGray);
                            dataReco.push_back(pair);
                        }
                        else{
                            cv::UMat flowUMAT;
                            cv::Mat flow; 
                            calcOpticalFlowFarneback(dataReco[indexFound].second, validImageGray, flowUMAT, 0.4, 1, 12, 2, 8, 1.5, 0);
                            flowUMAT.copyTo(flow);
                            bgrImg.copyTo(bgrImgCopy);
                            int countHorizontalFlow = 0;
                            for (int y = 0; y < bgrImgCopy.rows; y += 5) {
                                for (int x = 0; x < bgrImgCopy.cols; x += 5)
                                {
                                    // get the flow from y, x position * 10 for better visibility
                                    const Point2f flowatxy = flow.at<Point2f>(y, x) * 10;
                                    // draw line at flow direction
                                    // draw initial point
                                    // circle(bgrImgCopy, Point(x, y), 1, Scalar(0, 0, 0), -1);
                                    float lenght = sqrt(flowatxy.x * flowatxy.x + flowatxy.y * flowatxy.y);
                                    if(lenght > 15.0f){ // 0.01
                                        float angle = atan2(flowatxy.y, flowatxy.x);
                                        if(fabs(angle) < 0.79){
                                            // std::cout << "gesture_recog_node.->lenght:" << lenght << std::endl; 
                                            line(bgrImgCopy, Point(x, y), Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), Scalar(255,0,0));
                                            countHorizontalFlow++;
                                        }
                                    }
                                }

                            }
                            if(countHorizontalFlow > 30)
                                std::cout << "Waving user:" << indexUser << ", with:"<< countHorizontalFlow << std::endl;
                            std::stringstream ss;
                            ss.str("");
                            ss << "Image user index " << indexUser << std::endl;
                            cv::imshow(ss.str().c_str(), validImageGray);
                            ss.str("");
                            ss << "Image user index flow " << indexUser << std::endl;
                            cv::imshow(ss.str().c_str(), bgrImgCopy);
                            dataReco[indexFound].first = ske.ref_point;  
                            dataReco[indexFound].second = validImageGray;
                        }
                    }                 
                }
            }
        }*/
       
        for(int i = 0; i < dataRecoRightWrist.size(); i++){
            ros::Time lastTimeRecoData = std::get<2>(dataRecoRightWrist[i]);
            ros::Time currTime = ros::Time::now();
            ros::Duration dt = currTime - lastTimeRecoData;
            if(dt.toSec() > timeElapsedRemove)
                dataRecoRightWrist.erase(dataRecoRightWrist.begin() + i);
        }
        for(int i = 0; i < dataRecoLeftWrist.size(); i++){
            ros::Time lastTimeRecoData = std::get<2>(dataRecoLeftWrist[i]);
            ros::Time currTime = ros::Time::now();
            ros::Duration dt = currTime - lastTimeRecoData;
            if(dt.toSec() > timeElapsedRemove)
                dataRecoLeftWrist.erase(dataRecoLeftWrist.begin() + i);
        }

        if(gestures_detected.recog_gestures.size() > 0){
            for(int i = 0; i < gestures_detected.recog_gestures.size(); i++){
                if(gestures_detected.recog_gestures[i].gesture.compare("right_hand_rised") == 0 ||  gestures_detected.recog_gestures[i].gesture.compare("left_hand_rised") == 0){
                    int indexUser = -1;
                    for(int j = 0; j < skeletons.skeletons.size(); j++){
                        vision_msgs::Skeleton ske = skeletons.skeletons[j];
                        if(ske.user_id == gestures_detected.recog_gestures[i].id){
                            indexUser = j;
                            break;
                        }
                    }
                    if(indexUser < 0)
                        continue;
                         
                    if(gestures_detected.recog_gestures[i].gesture.compare("right_hand_rised") == 0){
                        vision_msgs::Skeleton ske = skeletons.skeletons[indexUser];
                        int indexRightWrist = 0;
                        bool foundRightWrist = findIndexJoint(ske, "right_wrist", indexRightWrist);
                        if(!foundRightWrist)
                            continue;
                        geometry_msgs::Point32 rightWrist;
                        rightWrist.x = ske.joints[indexRightWrist].position.x;
                        rightWrist.y = ske.joints[indexRightWrist].position.y;
                        rightWrist.z = ske.joints[indexRightWrist].position.z;
                        float wristX = rightWrist.x;
                        float wristY = rightWrist.y;
                        float wristZ = rightWrist.z;
                        
                        bool foundDataReco = false;
                        int indexFound = 0;
                        for(int k = 0; k < dataRecoRightWrist.size() && !foundDataReco; k++){
                            std::tuple<geometry_msgs::Point32, std::vector<geometry_msgs::Point32>, ros::Time > tupla = dataRecoRightWrist[k];
                            float distance = getEuclideanDistance(std::get<0>(tupla), ske.ref_point); 
                            if(distance <= thrToPerson){
                                indexFound = k;
                                foundDataReco = true;
                            }
                        }
                        if(!foundDataReco){
                            std::vector<geometry_msgs::Point32> wristData;
                            for(int k = 0; k < numFrames; k++)
                                wristData.push_back(rightWrist);
                            std::tuple<geometry_msgs::Point32, std::vector<geometry_msgs::Point32>, ros::Time > tupla;
                            std::get<0>(tupla) = ske.ref_point;
                            std::get<1>(tupla) = wristData;
                            std::get<2>(tupla) = ros::Time::now();
                            dataRecoRightWrist.push_back(tupla);
                        }
                        else{
                            std::vector<geometry_msgs::Point32> wristData = std::get<1>(dataRecoRightWrist[indexFound]);
                            wristData.pop_back();
                            wristData.insert(wristData.begin(), rightWrist);
                            std::get<0>(dataRecoRightWrist[indexFound]) = ske.ref_point;
                            std::get<1>(dataRecoRightWrist[indexFound]) = wristData;
                            std::get<2>(dataRecoRightWrist[indexFound]) = ros::Time::now();
                            float motion = 0.0;
                            for(int k = std::get<1>(dataRecoRightWrist[indexFound]).size() - 1; k > 0; k--)
                                motion += getEuclideanDistance(std::get<1>(dataRecoRightWrist[indexFound])[k], std::get<1>(dataRecoRightWrist[indexFound])[k - 1]);
                            if(motion > thrWavingMotion){
                                vision_msgs::GestureSkeleton gesture_detected = gestures_detected.recog_gestures[i];
                                gesture_detected.gesture = "right_waving";
                                gestures_detected.recog_gestures.push_back(gesture_detected);
                                std::cout << "gesture_recog_node.->Right waving detect user id:" << indexUser << std::endl;
                            }
                        }
                    }
                    
                    if(gestures_detected.recog_gestures[i].gesture.compare("left_hand_rised") == 0){
                        vision_msgs::Skeleton ske = skeletons.skeletons[indexUser];
                        int indexLeftWrist = 0;
                        bool foundLeftWrist = findIndexJoint(ske, "left_wrist", indexLeftWrist);
                        if(!foundLeftWrist)
                            continue;
                        geometry_msgs::Point32 leftWrist;
                        leftWrist.x = ske.joints[indexLeftWrist].position.x;
                        leftWrist.y = ske.joints[indexLeftWrist].position.y;
                        leftWrist.z = ske.joints[indexLeftWrist].position.z;
                        float wristX = leftWrist.x;
                        float wristY = leftWrist.y;
                        float wristZ = leftWrist.z;
                        
                        bool foundDataReco = false;
                        int indexFound = 0;
                        for(int k = 0; k < dataRecoLeftWrist.size() && !foundDataReco; k++){
                            std::tuple<geometry_msgs::Point32, std::vector<geometry_msgs::Point32>, ros::Time > tupla = dataRecoLeftWrist[k];
                            float distance = getEuclideanDistance(std::get<0>(tupla), ske.ref_point); 
                            if(distance <= thrToPerson){
                                indexFound = k;
                                foundDataReco = true;
                            }
                        }
                        if(!foundDataReco){
                            std::vector<geometry_msgs::Point32> wristData;
                            for(int k = 0; k < numFrames; k++)
                                wristData.push_back(leftWrist);
                            std::tuple<geometry_msgs::Point32, std::vector<geometry_msgs::Point32>, ros::Time > tupla;
                            std::get<0>(tupla) = ske.ref_point;
                            std::get<1>(tupla) = wristData;
                            std::get<2>(tupla) = ros::Time::now();
                            dataRecoLeftWrist.push_back(tupla);
                        }
                        else{
                            std::vector<geometry_msgs::Point32> wristData = std::get<1>(dataRecoLeftWrist[indexFound]);
                            wristData.pop_back();
                            wristData.insert(wristData.begin(), leftWrist);
                            std::get<0>(dataRecoLeftWrist[indexFound]) = ske.ref_point;
                            std::get<1>(dataRecoLeftWrist[indexFound]) = wristData;
                            std::get<2>(dataRecoLeftWrist[indexFound]) = ros::Time::now();;
                            float motion = 0.0;
                            for(int k = std::get<1>(dataRecoLeftWrist[indexFound]).size() - 1; k > 0; k--)
                                motion += getEuclideanDistance(std::get<1>(dataRecoLeftWrist[indexFound])[k], std::get<1>(dataRecoLeftWrist[indexFound])[k - 1]);
                            if(motion > thrWavingMotion){
                                vision_msgs::GestureSkeleton gesture_detected = gestures_detected.recog_gestures[i];
                                gesture_detected.gesture = "left_waving";
                                gestures_detected.recog_gestures.push_back(gesture_detected);
                                std::cout << "gesture_recog_node.->Left waving detect user id:" << indexUser << std::endl;
                            }
                        }
                    }
                }
            }
        }
        
        if(gestures_detected.recog_gestures.size() > 0)
            pubGestures.publish(gestures_detected);
        
        gestures_detected.recog_gestures.clear();
        
        loop.sleep();
        ros::spinOnce();
    }

    return 0;
}
