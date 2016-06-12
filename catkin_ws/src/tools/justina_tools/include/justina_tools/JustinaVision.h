#pragma once
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "bbros_bridge/RecognizedSpeech.h"
#include "bbros_bridge/Default_ROS_BB_Bridge.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/PointCloud2.h"
#include "point_cloud_manager/GetRgbd.h"
#include "vision_msgs/VisionObject.h"
#include "vision_msgs/DetectObjects.h"
#include "vision_msgs/VisionFaceTrainObject.h"
#include "vision_msgs/VisionFaceObjects.h"

class JustinaVision
{
private:
    static bool is_node_set;
    //Members for operating skeleton finder
    static ros::Publisher pubSktStartRecog;
    static ros::Publisher pubSktStopRecog;
    //Members for operating face recognizer
    static ros::Publisher pubFacStartRecog;
    static ros::Publisher pubFacStopRecog;
    static ros::Publisher pubTrainFace;
    static ros::Publisher pubTrainFaceNum;
    static ros::Publisher pubRecFace;
    static ros::Publisher pubRecFaceByID;
    static ros::Publisher pubClearFacesDB;
    static ros::Publisher pubClearFacesDBByID;
    static ros::Subscriber subFaces;
    static ros::Subscriber subTrainer;
    static vision_msgs::VisionFaceObject lastRecognizedFace;
    static int lastFaceRecogResult;
    //Recog objects
    static ros::ServiceClient cltDetectObjects;

public:
    static bool setNodeHandle(ros::NodeHandle* nh);
    //Methods for operating skeleton finder
    static void startSkeletonFinding();
    static void stopSkeletonFinding();
    //Methods for operating face recognizer
    static void startFaceRecognition();
    static void startFaceRecognition(std::string id);
    static void stopFaceRecognition();
    static void facTrain(std::string id);
    static void facTrain(std::string id, int numOfFrames);
    static void facClearByID(std::string id);
    static void facClearAll();
    static void getLastRecognizedFace(std::string& id, float& posX, float& posY, float& posZ, float& confidence, int& gender, bool& isSmiling);
    static int getLastTrainingResult();
    //Methods for object detector and recognizer   
    static bool detectObjects(std::vector<vision_msgs::VisionObject>& recoObjList);

private:
    //callbacks for face recognition
    static void callbackFaces(const vision_msgs::VisionFaceObjects::ConstPtr& msg);
    static void callbackTrainer(const std_msgs::Int32::ConstPtr& msg);
};
