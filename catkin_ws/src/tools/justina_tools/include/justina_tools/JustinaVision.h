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
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/PointCloud2.h"
#include "point_cloud_manager/GetRgbd.h"
#include "sensor_msgs/Image.h"
#include "vision_msgs/VisionObject.h"
#include "vision_msgs/VisionObjectList.h"
#include "vision_msgs/DetectObjects.h"
#include "vision_msgs/VisionFaceTrainObject.h"
#include "vision_msgs/VisionFaceObjects.h"
#include "vision_msgs/FindLines.h"
#include "vision_msgs/FindPlane.h"
#include "vision_msgs/GetThermalAngle.h"
#include "vision_msgs/GestureSkeletons.h"
#include "vision_msgs/Skeletons.h"
#include "vision_msgs/HandSkeletonPos.h"
#include "vision_msgs/TrainObject.h"
#include "vision_msgs/GetFacesFromImage.h"
#include "vision_msgs/DetectGripper.h"
#include "vision_msgs/FaceRecognition.h"
#include "vision_msgs/FindWaving.h"
#include "vision_msgs/GetCubes.h"
#include "vision_msgs/SRV_DetectPlasticTrayZones.h"
#include "vision_msgs/SRV_FindDishwasher.h"

class JustinaVision
{
private:
    static bool is_node_set;
    //Members for operating pano maker
    static ros::Publisher pubTakePanoMaker;
    static ros::Publisher pubClearPanoMaker;
    static ros::Publisher pubMakePanoMaker;
    static ros::Subscriber subPanoImage;
    static sensor_msgs::Image lastImage;
    static bool panoImageRecived;
    //Members for operating skeleton finder
    static ros::Publisher pubSktEnableRecog;
    static ros::Subscriber subGestures;
    static ros::Subscriber subSkeletons;
    static ros::Subscriber subLeftHandPositions;
    static ros::Subscriber subRightHandPositions;
    static std::vector<vision_msgs::Skeleton> lastSkeletons;
    static std::vector<vision_msgs::GestureSkeleton> lastGestureRecog;
    static std::vector<geometry_msgs::Point> lastLeftHandPos;
    static std::vector<geometry_msgs::Point> lastRightHandPos;
    //Members for operating face recognizer
    static ros::Publisher pubStartFaceDetection;
    static ros::Publisher pubStartFaceRecognition;
    static ros::Publisher pubStartFaceRecognition2D;
    static ros::Publisher pubSetIdFaceRecognition;
    static ros::Publisher pubTrainerFaces;
    static ros::Publisher pubClearFacesDB;
    static ros::Publisher pubClearFacesDBByID;
    /* 
    static ros::Publisher pubTrainFace;
    static ros::Publisher pubRecFace;
    static ros::Publisher pubRecFaceByID;
    static ros::Subscriber subTrainer;*/
    static ros::Subscriber subFaces;
    static ros::ServiceClient cltPanoFaceReco;
    static std::vector<vision_msgs::VisionFaceObject> lastRecognizedFaces;
    static int lastFaceRecogResult;
    //Service for face recognition
    static ros::ServiceClient cltDetectPanoFaces;
    static ros::ServiceClient cltDetectFaces;
    static ros::ServiceClient cltDetectWaving;
    static ros::ServiceClient cltFaceRecognition;
    static ros::ServiceClient cltFaceRecognition2D;
    //Members for thermal camera
    static ros::Publisher pubStartThermalCamera;
    static ros::Publisher pubStopThermalCamera;
    //Services for getting point cloud
    static ros::ServiceClient cltGetRgbdWrtKinect;
    static ros::ServiceClient cltGetRgbdWrtRobot;
    //Recog objects
    static ros::ServiceClient cltDetectObjects;
    static ros::ServiceClient cltDetectAllObjects;
    static ros::ServiceClient cltDetectAllObjectsVot;
    static ros::Publisher pubObjStartRecog;
    static ros::Publisher pubObjStopRecog;
    static ros::Publisher pubObjStartWin;
    static ros::Publisher pubObjStopWin;
    static ros::Publisher pubEnableObjsDetectYOLO;
    static ros::ServiceClient srvTrainObject;
    static ros::ServiceClient srvTrainObjectByHeight;
    static ros::Publisher pubMove_base_train_vision;
    static ros::ServiceClient cltDetecObjectsYOLO;
    static ros::Subscriber subGetRecoObjYOLO;
    static std::vector<vision_msgs::VisionObject> lastObjRecoYOLO;
    //Sevices for line finding
    static ros::ServiceClient cltFindLines;
    //Service for find plane
    static ros::ServiceClient cltFindPlane;
    static ros::ServiceClient cltFindTable;
    //Service for find vacant plane
    static ros::ServiceClient cltFindVacantPlane;
    //Members for operation of qr reader
    static ros::Publisher pubQRReaderStart;
    //Services for thermal camera
    static ros::ServiceClient cltGetAngle;
    //Members for detect hand in front of gripper
    static ros::Publisher pubStartHandFrontDetectBB;
    static ros::Publisher pubStopHandFrontDetectBB;
    static ros::Subscriber subHandFrontDetectBB;
    static bool isHandFrontDetectedBB;
    static ros::Publisher pubStartHandNearestDetectBB;
    static ros::Publisher pubStopHandNearestDetectBB;
    static ros::Subscriber subHandNearestDetectBB;
    static geometry_msgs::Point32 lastHandNearestDetectedBB;
    static bool isHandNearestDetectedBB;
    //Members for detect gripper
    static ros::ServiceClient cltGripperPos;
    //Members to segment objects by color
    static ros::ServiceClient cltCubesSeg;
    static ros::ServiceClient cltCutlerySeg;
    static ros::ServiceClient cltGetTray;
    static ros::ServiceClient cltGetDishwasher;

public:
    static bool setNodeHandle(ros::NodeHandle* nh);
    //Methods for pano maker
    static void takePano();
    static void clearPano();
    static void makePano();
    static bool isPanoImageRecived();
    static sensor_msgs::Image getLastPanoImage();
    //Methods for operating skeleton finder
    static void startSkeletonFinding();
    static void stopSkeletonFinding();
    static void getLastSkeletons(std::vector<vision_msgs::Skeleton> &skeletons);
    static void getLastGesturesRecognize(std::vector<vision_msgs::GestureSkeleton> &gestures);
    static void getLastLeftHandPositions(std::vector<geometry_msgs::Point> &leftHandPositions);
    static void getLastRightHandPositions(std::vector<geometry_msgs::Point> &rightHandPositions);
    //Methods for operating face recognizer
    static void startFaceDetection(bool enable);
    static void startFaceRecognition(bool enable);
    static void startFaceRecognition2D(bool enable);
    static void setIdFaceRecognition(std::string id);
    static void faceTrain(std::string id, int numOfFrames);
    static void facClearByID(std::string id);
    static void facClearAll();
    /*static void facRecognize();
    static void facRecognize(std::string id);
    static void facTrain(std::string id);*/
    static bool getMostConfidentFace(std::string& id, float& posX, float& posY, float& posZ, float& confidence, int& gender, bool& isSmiling);
    static bool getLastRecognizedFaces(std::vector<vision_msgs::VisionFaceObject>& faces);
    // static int getLastTrainingResult();
    static vision_msgs::VisionFaceObjects getRecogFromPano(sensor_msgs::Image image);
    static vision_msgs::VisionFaceObjects getFaces();
    static vision_msgs::VisionFaceObjects getFaceRecognition(std::string id = "");
    static vision_msgs::VisionFaceObjects getFaceRecognition2D(std::string id = "");
    static std::vector<vision_msgs::VisionRect> detectWaving();
    //Methods for object detector and recognizer
    static void startObjectFinding();
    static void stopObjectFinding();
    static void startObjectFindingWindow();
    static void stopObjectFindingWindow();
    static bool detectObjects(std::vector<vision_msgs::VisionObject>& recoObjList, bool saveFiles = false);
    static bool detectAllObjects(std::vector<vision_msgs::VisionObject>& recoObjList, bool saveFiles = false);
    static bool detectAllObjectsVot(std::vector<vision_msgs::VisionObject>& recoObjList, sensor_msgs::Image &image, int iterations = 1);
    //Action client for YOLO object recog
    static void enableDetectObjsYOLO(bool enable);
    static bool detectObjectsYOLO(std::vector<vision_msgs::VisionObject>& yoloObjects);
    static void getObjectsYOLO(std::vector<vision_msgs::VisionObject>& yoloObjects);
    static void moveBaseTrainVision(const std_msgs::String& msg);
    //Methods for line finding
    static bool findLine(float& x1, float& y1, float& z1, float& x2, float& y2, float& z2);
    //Methods for plane findinig
    static bool findPlane();
    static bool findTable(std::vector<float>& nearestPoint);
    //Methods for plan vacant finding
    static bool findVacantPlane(std::vector<float>& vacantPlane, std::vector<int>& inliersOnPlane);
    //Methods for the qr reader
    static void startQRReader();
    static void stopQRReader();
    //Methods for the thermal camera
    static void startThermalCamera();
    static void stopThermalCamera();
    static float getAngleTC();
    //Methods for the hand detect in front of gripper
    static void startHandFrontDetectBB(float x, float y, float z);
    static void stopHandFrontDetectBB();
    static bool getDetectionHandFrontBB();
    static bool getDetectionHandNearestBB(geometry_msgs::Point32 &nearestPoint);
    static void trainObject(const std::string name);
    static void trainObjectByHeight(const std::string name);
    //Methods for gripper detect
    static bool getGripperPos(geometry_msgs::Point& gripperPos);
    //Methods for segment objects by color
    static bool getCubesSeg(vision_msgs::CubesSegmented& cubes);
    static bool getCutlerySeg(vision_msgs::CubesSegmented& cutleries);
    static bool isStillOnTable(vision_msgs::Cube my_cube);
    static bool getTray(vision_msgs::MSG_VisionPlasticTray &tray);
    static bool getDishwasher(vision_msgs::MSG_VisionDishwasher &dishwasher);

private:
    //callbacks for obj recog
    static void callbackGetRecoObjYOLO(const vision_msgs::VisionObjectList::ConstPtr& msg);
    //callbacks for pano maker
    static void callbackPanoRecived(const sensor_msgs::Image msg);
    //callbacks for skeleton recognition
    static void callbackSkeletons(const vision_msgs::Skeletons::ConstPtr& msg);
    static void callbackGestures(const vision_msgs::GestureSkeletons::ConstPtr& msg);
    static void callbackLeftHandPositions(const vision_msgs::HandSkeletonPos leftHandPositions);
    static void callbackRightHandPositions(const vision_msgs::HandSkeletonPos rightHandPositions);
    //callbacks for face recognition
    static void callbackFaces(const vision_msgs::VisionFaceObjects::ConstPtr& msg);
    static void callbackTrainer(const std_msgs::Int32::ConstPtr& msg);
    //callbacks for the hand detect in front of gripper
    static void callbackHandFrontDetectBB(const std_msgs::Bool::ConstPtr& msg);
    static void callbackHandNearestDetectBB(const geometry_msgs::Point32 msg);
};
