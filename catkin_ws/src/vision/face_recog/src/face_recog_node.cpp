#include <iostream>

#include <sstream>
#include <string>
#include <algorithm>    // std::sort
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"
#include "vision_msgs/VisionFaceObjects.h"
#include "vision_msgs/VisionFaceObject.h"
#include "vision_msgs/VisionFaceTrainObject.h"
#include "vision_msgs/GetFacesFromImage.h"
#include "vision_msgs/FindWaving.h"
#include "vision_msgs/VisionRect.h"
#include "vision_msgs/FaceRecognition.h"
#include "justina_tools/JustinaTools.h"
#include "webcam_man/GetRgb.h"
#include "geometry_msgs/Point.h"

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include "facerecog/facerecog.h"
#include "facerecog/faceobj.h"

using namespace std;
using namespace cv;

ros::NodeHandle* node;
ros::ServiceClient cltRgbdRobot;
ros::ServiceClient cltRgbWebCam;
ros::ServiceClient cltFaceRecognition;
ros::ServiceClient cltFacesAgeGender;
ros::ServiceClient cltFaceTrain;
ros::ServiceClient cltFaceTrainFlush;

// Face recognizer
facerecog facerecognizer;

/*
   bool recFace = false;
   int trainFailed = 0;
   int maxNumFailedTrain = 5;*/
ros::Publisher pubTrainer;
bool trainNewFace = false;
string trainID = "unknown";
bool clearFaces = false;
bool clearFaceID = false;
int numTrain = 1;
int trainedcount = 0;

ros::Publisher pubFaces;
string faceID = "";
bool enableFaceDetection = false;
bool enableFaceRecognition = false;
bool enableFaceRecognition2D = false;
bool enableFaceAgeGender = false;
bool enableFaceAgeGender2D = false;

// Services
ros::ServiceServer srvDetectPanoFaces;
ros::ServiceServer srvDetectFaces;
ros::ServiceServer srvDetectWave;
ros::ServiceServer srvFaceRecognition;
ros::ServiceServer srvFaceRecognition2D;
ros::ServiceServer srvFaceAgeGender;
ros::ServiceServer srvFaceAgeGender2D;

bool GetImagesFromJustina( cv::Mat& imaBGR, cv::Mat& imaPCL)
{
    point_cloud_manager::GetRgbd srv;
    if(!cltRgbdRobot.call(srv))
    {
        std::cout << "ObjDetector.->Cannot get point cloud" << std::endl;
        return false;
    }
    JustinaTools::PointCloud2Msg_ToCvMat(srv.response.point_cloud, imaBGR, imaPCL);
    return true; 
}

bool GetImagesFromJustina(cv::Mat& imaBGR)
{
    webcam_man::GetRgb srv;
    if(!cltRgbWebCam.call(srv))
    {
        std::cout << "ObjDetector.->Cannot get image from webcam" << std::endl;
        return false;
    }
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(srv.response.imageBGR, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception &e)
    {
        ROS_ERROR("face_recog_node.->cv_bridge exception: %s", e.what());
        return false;
    }
    imaBGR = cv_ptr->image;
    return true;
}


bool faceobjSortFunction (faceobj i,faceobj j) { 
    return (i.boundingbox.x < j.boundingbox.x); 
}

bool RectSortFunction (Rect i,Rect j) { 
    return (i.x < j.x); 
}

bool RectSortFunctionSize (Rect i,Rect j) { 
    return ((i.height * i.width) > (j.height * j.width)); 
}

void faceAddOverlays(vision_msgs::VisionFaceObjects faceObjects, cv::Mat &bgrImg)
{
    for(int i = 0; i < faceObjects.recog_faces.size(); i++){
        vision_msgs::VisionFaceObject faceObject = faceObjects.recog_faces[i];
        cv::rectangle(bgrImg, cv::Point(faceObject.bounding_box[0].x, faceObject.bounding_box[0].y),
                cv::Point(faceObject.bounding_box[1].x, faceObject.bounding_box[1].y), 
                cv::Scalar(255, 0, 0), 2);
        if(faceObject.id.compare("") != 0){
            cv::putText(bgrImg, faceObject.id, cv::Point(faceObject.bounding_box[0].x, faceObject.bounding_box[1].y - 30),
                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 1);
            cv::putText(bgrImg, std::to_string(faceObject.confidence), cv::Point(faceObject.bounding_box[0].x, faceObject.bounding_box[1].y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 1);
        }
    }
}

void faceAddLabel(vision_msgs::VisionFaceObjects faceObjects, cv::Mat &bgrImg)
{
    for(int i = 0; i < faceObjects.recog_faces.size(); i++)
    {
        vision_msgs::VisionFaceObject faceObject = faceObjects.recog_faces[i];
        std::stringstream ss;
        int baseline;
        ss << std::to_string(faceObject.ages) << ", ";
        faceObject.gender == 0 ? ss << "F" : ss << "M";
        cv::Size size = cv::getTextSize(ss.str(), cv::FONT_HERSHEY_SIMPLEX, 1.0, 2, &baseline);
        cv::rectangle(bgrImg, cv::Point(faceObject.bounding_box[0].x, faceObject.bounding_box[0].y - size.height), cv::Point(faceObject.bounding_box[0].x + size.width, faceObject.bounding_box[0].y), cv::Scalar(255, 0, 0), cv::FILLED);
        cv::rectangle(bgrImg, cv::Point(faceObject.bounding_box[0].x, faceObject.bounding_box[0].y), cv::Point(faceObject.bounding_box[1].x, faceObject.bounding_box[1].y), cv::Scalar(255, 0, 0), 2);
        cv::putText(bgrImg, ss.str(), cv::Point(faceObject.bounding_box[0].x, faceObject.bounding_box[0].y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2, 1);
    }
}

vision_msgs::VisionFaceObjects faceDetection(cv::Mat bgrImg, cv::Mat xyzCloud)
{
    vision_msgs::VisionFaceObjects faceObjects;
    std::vector<faceobj> facesdetected = facerecognizer.facialRecognitionForever(bgrImg, xyzCloud, "");
    vision_msgs::VisionFaceObjects faces_detected;
    if(facesdetected.size() > 0) {
        //Sort vector
        std::sort (facesdetected.begin(), facesdetected.end(), faceobjSortFunction);

        for (int x = 0; x < facesdetected.size(); x++) {
            vision_msgs::VisionFaceObject face;
            geometry_msgs::Point p; 
            face.id = facesdetected[x].id;
            face.confidence = facesdetected[x].confidence;
            face.face_centroid.x = facesdetected[x].pos3D.x;
            face.face_centroid.y = facesdetected[x].pos3D.y;
            face.face_centroid.z = facesdetected[x].pos3D.z;
            p.x = facesdetected[x].boundingbox.x;
            p.y = facesdetected[x].boundingbox.y;
            face.bounding_box.push_back(p);
            p.x = facesdetected[x].boundingbox.x + facesdetected[x].boundingbox.width;
            p.y = facesdetected[x].boundingbox.y + facesdetected[x].boundingbox.height;
            face.bounding_box.push_back(p);
            face.smile = facesdetected[x].smile;
            face.gender = facesdetected[x].gender;
            face.ages = facesdetected[x].ages;
            faceObjects.recog_faces.push_back(face);
        }
    }
    return faceObjects;
}

vision_msgs::VisionFaceObjects faceRecognition(std::string faceID, cv::Mat bgrImg, cv::Mat xyzCloud)
{
    vision_msgs::VisionFaceObjects faceObjects;
    vision_msgs::FaceRecognition srv;
    srv.request.id = faceID;
    sensor_msgs::Image container;
    cv_bridge::CvImage cvi_mat;
    cvi_mat.encoding = sensor_msgs::image_encodings::BGR8;
    cvi_mat.image = bgrImg;
    cvi_mat.toImageMsg(container);
    srv.request.imageBGR = container;	
    if(cltFaceRecognition.call(srv))
    {
        faceObjects = srv.response.faces;

        for(int i = 0; i < faceObjects.recog_faces.size(); i++){
            vision_msgs::VisionFaceObject faceObject = faceObjects.recog_faces[i];
            cv::Rect roi3D;
            roi3D.x = cvRound(faceObject.bounding_box[0].x);
            roi3D.y = cvRound(faceObject.bounding_box[0].y);
            roi3D.width = cvRound(fabs(faceObject.bounding_box[1].x - faceObject.bounding_box[0].x));
            roi3D.height = cvRound(fabs(faceObject.bounding_box[1].y - faceObject.bounding_box[0].y));
            cv::Mat facexyz = xyzCloud(roi3D).clone();
            Vec3f face3Dcenter = facexyz.at<cv::Vec3f>(facexyz.rows * 0.5, facexyz.cols * 0.5);
            faceObjects.recog_faces[i].face_centroid.x = face3Dcenter[0];
            faceObjects.recog_faces[i].face_centroid.y = face3Dcenter[1];
            faceObjects.recog_faces[i].face_centroid.z = face3Dcenter[2];
        }

        faceAddOverlays(faceObjects, bgrImg);
        cv::imshow("Face recognition", bgrImg);
    }
    else
        std::cout << "face_recog_node.->Error in face recognition service." << std::endl;

    return faceObjects;
}

vision_msgs::VisionFaceObjects faceRecognition2D(std::string faceID, cv::Mat bgrImg)
{
    vision_msgs::VisionFaceObjects faceObjects;
    vision_msgs::FaceRecognition srv;
    srv.request.id = faceID;
    sensor_msgs::Image container;
    cv_bridge::CvImage cvi_mat;
    cvi_mat.encoding = sensor_msgs::image_encodings::BGR8;
    cvi_mat.image = bgrImg;
    cvi_mat.toImageMsg(container);
    srv.request.imageBGR = container;	
    if(cltFaceRecognition.call(srv))
    {
        faceObjects = srv.response.faces;

        for(int i = 0; i < faceObjects.recog_faces.size(); i++)
            vision_msgs::VisionFaceObject faceObject = faceObjects.recog_faces[i];

        faceAddOverlays(faceObjects, bgrImg);
        cv::imshow("Face recognition", bgrImg);
    }
    else
        std::cout << "face_recog_node.->Error in face recognition service." << std::endl;

    return faceObjects;
}

void faceAgeGender(cv::Mat bgrImg, cv::Mat xyzCloud, vision_msgs::VisionFaceObjects &faceObjects)
{
    vision_msgs::FaceRecognition srv;
    sensor_msgs::Image container;
    cv_bridge::CvImage cvi_mat;
    cvi_mat.encoding = sensor_msgs::image_encodings::BGR8;
    cvi_mat.image = bgrImg;
    cvi_mat.toImageMsg(container);
    srv.request.imageBGR = container;
    srv.request.faces = faceObjects;
    if(cltFacesAgeGender.call(srv))
    {
        faceObjects = srv.response.faces;
        for(int i = 0; i < faceObjects.recog_faces.size(); i++){
            vision_msgs::VisionFaceObject faceObject = faceObjects.recog_faces[i];
            cv::Rect roi3D;
            roi3D.x = cvRound(faceObject.bounding_box[0].x);
            roi3D.y = cvRound(faceObject.bounding_box[0].y);
            roi3D.width = cvRound(fabs(faceObject.bounding_box[1].x - faceObject.bounding_box[0].x));
            roi3D.height = cvRound(fabs(faceObject.bounding_box[1].y - faceObject.bounding_box[0].y));
            cv::Mat facexyz = xyzCloud(roi3D).clone();
            Vec3f face3Dcenter = facexyz.at<cv::Vec3f>(facexyz.rows * 0.5, facexyz.cols * 0.5);
            faceObjects.recog_faces[i].face_centroid.x = face3Dcenter[0];
            faceObjects.recog_faces[i].face_centroid.y = face3Dcenter[1];
            faceObjects.recog_faces[i].face_centroid.z = face3Dcenter[2];
        }
        faceAddLabel(faceObjects, bgrImg);
        cv::imshow("Face recognition", bgrImg);
    }
    else
        std::cout << "face_recog_node.->Error in face recognition service." << std::endl;
}

void faceAgeGender2D(cv::Mat bgrImg, vision_msgs::VisionFaceObjects &faceObjects)
{
    vision_msgs::FaceRecognition srv;
    sensor_msgs::Image container;
    cv_bridge::CvImage cvi_mat;
    cvi_mat.encoding = sensor_msgs::image_encodings::BGR8;
    cvi_mat.image = bgrImg;
    cvi_mat.toImageMsg(container);
    srv.request.imageBGR = container;
    srv.request.faces = faceObjects;
    if(cltFacesAgeGender.call(srv))
    {
        faceObjects = srv.response.faces;
        faceAddLabel(faceObjects, bgrImg);
        cv::imshow("Face recognition", bgrImg);
    }
    else
        std::cout << "face_recog_node.->Error in face recognition service." << std::endl;
}

/*void callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
  cv::Mat bgrImg;
  cv::Mat xyzCloud;

  if (clearDB) {
  clearDB = false;
  facerecognizer.clearFaceDB();
  cout << "Faces Data Base Cleared!! =(" << endl;
  }

  if (clearDBByID) {
  clearDBByID = false;
  facerecognizer.clearFaceDB(trainID);
  cout << trainID << " was destroyed!!" << endl;
  }

  if (trainNewFace) {

  if(trainedcount < numTrain) {
  JustinaTools::PointCloud2Msg_ToCvMat(msg, bgrImg, xyzCloud);
  if(facerecognizer.faceTrainer(bgrImg, xyzCloud, trainID)) {
  trainedcount++;
  } else {
  trainFailed++; //train failed!!
  }	

  if(trainFailed > maxNumFailedTrain) {
  trainNewFace = false;
  std_msgs::Int32 numTrainmsg;
  numTrainmsg.data = trainedcount;
  pubTrainer.publish(numTrainmsg);
  }
  } 
  else {
  trainNewFace = false;
  std_msgs::Int32 numTrainmsg;
  numTrainmsg.data = trainedcount;
  pubTrainer.publish(numTrainmsg);

  }		
  } else {

  trainFailed = 0;
  }



  if (recFace) {
  recFace = false;
  JustinaTools::PointCloud2Msg_ToCvMat(msg, bgrImg, xyzCloud);
  if(faceID.compare("") == 0){

  std::vector<faceobj> facesdetected = facerecognizer.facialRecognitionForever(bgrImg, xyzCloud, faceID);

  vision_msgs::VisionFaceObjects faces_detected;

  if(facesdetected.size() > 0) {
//Sort vector
std::sort (facesdetected.begin(), facesdetected.end(), faceobjSortFunction);

for (int x = 0; x < facesdetected.size(); x++) {
vision_msgs::VisionFaceObject face;
geometry_msgs::Point p; 
face.id = facesdetected[x].id;
face.confidence = facesdetected[x].confidence;
face.face_centroid.x = facesdetected[x].pos3D.x;
face.face_centroid.y = facesdetected[x].pos3D.y;
face.face_centroid.z = facesdetected[x].pos3D.z;
p.x = facesdetected[x].boundingbox.x;
p.y = facesdetected[x].boundingbox.y;
face.bounding_box.push_back(p);
p.x = facesdetected[x].boundingbox.x + facesdetected[x].boundingbox.width;
p.y = facesdetected[x].boundingbox.y + facesdetected[x].boundingbox.height;
face.bounding_box.push_back(p);
face.smile = facesdetected[x].smile;
face.gender = facesdetected[x].gender;

faces_detected.recog_faces.push_back(face);
}

}
pubFaces.publish(faces_detected);
}
else{
    vision_msgs::VisionFaceObjects faces = facenetRecognition(faceID, bgrImg, xyzCloud);
    pubFaces.publish(faces);
}
}

if (recFaceForever) {
    JustinaTools::PointCloud2Msg_ToCvMat(msg, bgrImg, xyzCloud);
    if(faceID.compare("") == 0){
        std::vector<faceobj> facesdetected = facerecognizer.facialRecognitionForever(bgrImg, xyzCloud, faceID);

        vision_msgs::VisionFaceObjects faces_detected;

        if(facesdetected.size() > 0) {
            //Sort vector
            std::sort (facesdetected.begin(), facesdetected.end(), faceobjSortFunction);

            for (int x = 0; x < facesdetected.size(); x++) {
                vision_msgs::VisionFaceObject face;
                geometry_msgs::Point p; 
                face.id = facesdetected[x].id;
                face.confidence = facesdetected[x].confidence;
                face.face_centroid.x = facesdetected[x].pos3D.x;
                face.face_centroid.y = facesdetected[x].pos3D.y;
                face.face_centroid.z = facesdetected[x].pos3D.z;
                p.x = facesdetected[x].boundingbox.x;
                p.y = facesdetected[x].boundingbox.y;
                face.bounding_box.push_back(p);
                p.x = facesdetected[x].boundingbox.x + facesdetected[x].boundingbox.width;
                p.y = facesdetected[x].boundingbox.y + facesdetected[x].boundingbox.height;
                face.bounding_box.push_back(p);
                face.smile = facesdetected[x].smile;
                face.gender = facesdetected[x].gender;

                faces_detected.recog_faces.push_back(face);
            }
        }
        pubFaces.publish(faces_detected);
    }
    else{
        vision_msgs::VisionFaceObjects faces = facenetRecognition(faceID, bgrImg, xyzCloud);
        pubFaces.publish(faces);
    }
}

//cv::imshow("FACE RECOGNIZER", bgrImg);
//cv::imshow("FACE RECOGNIZER POINT CLOUD", xyzCloud);

}

void callbackTrainFace(const std_msgs::String::ConstPtr& msg)
{
    trainID = msg->data;
    if(trainID != "") {
        numTrain = 1;
        trainNewFace = true;
        trainedcount = 0;
    }
}

void callbackRecFace(const std_msgs::Empty::ConstPtr& msg)
{
    faceID = "";
    recFace = true;
}

void callbackRecFaceByID(const std_msgs::String::ConstPtr& msg)
{
    faceID = msg->data;
    recFace = true;
}

void callbackStartRecog(const std_msgs::Empty::ConstPtr& msg)
{
    std::cout << "FaceRecognizer.->Starting face recognition..." << std::endl;
    trainNewFace = false;
    recFace = false;
    clearDB = false;
    clearDBByID = false;
    numTrain = 1;
    trainedcount = 0;
    trainID = "unknown";
    faceID = "";
    trainFailed = 0;
    recFaceForever = true;

    // Me suscribo al topico que publica los datos del kinect
    subPointCloud = node->subscribe("/hardware/point_cloud_man/rgbd_wrt_robot", 1, callbackPointCloud);


}

void callbackStartRecogOld(const std_msgs::Empty::ConstPtr& msg)
{
    std::cout << "FaceRecognizer.->Starting face recognition..." << std::endl;
    trainNewFace = false;
    recFace = false;
    clearDB = false;
    clearDBByID = false;
    numTrain = 1;
    trainedcount = 0;
    trainID = "unknown";
    faceID = "";
    trainFailed = 0;
    recFaceForever = false; // Por peticion

    // Me suscribo al topico que publica los datos del kinect
    subPointCloud = node->subscribe("/hardware/point_cloud_man/rgbd_wrt_robot", 1, callbackPointCloud);

}

void callbackStopRecog(const std_msgs::Empty::ConstPtr& msg)
{
    /// NOTHING
    std::cout << "FaceRecognizer.->Stopping face recognition..." << std::endl;
    recFaceForever = false;
    subPointCloud.shutdown();
    cv::destroyAllWindows();
}*/

void callbackTrainFaces(const vision_msgs::VisionFaceTrainObject& msg)
{
    trainID = msg.id;
    numTrain = msg.frames;
    if(trainID != "")
    {
        if (numTrain > 0)
        { 
            trainNewFace = true;
            trainedcount = 0;
        }
    }
}

void callbackStartFaceDetection(const std_msgs::Bool::ConstPtr& msg)
{
    enableFaceDetection = msg->data;
    enableFaceRecognition = false;
    enableFaceRecognition2D = false;
    enableFaceAgeGender = false;
    enableFaceAgeGender2D = false;
    faceID = "";
    if(enableFaceDetection)
        std::cout << "FaceRecognizer.->Starting face detection..." << std::endl;
    else
        std::cout << "FaceRecognizer.->Stoping face detection..." << std::endl;
}

void callbackStartFaceRecognition(const std_msgs::Bool::ConstPtr& msg)
{
    enableFaceRecognition = msg->data;
    enableFaceDetection = false;
    enableFaceRecognition2D = false;
    enableFaceAgeGender = false;
    enableFaceAgeGender2D = false;
    faceID = "";
    if(enableFaceRecognition)
        std::cout << "FaceRecognizer.->Starting face recognition..." << std::endl;
    else
        std::cout << "FaceRecognizer.->Stoping face recognition..." << std::endl;
}

void callbackStartFaceRecognition2D(const std_msgs::Bool::ConstPtr& msg)
{
    enableFaceRecognition2D = msg->data;
    enableFaceDetection = false;
    enableFaceRecognition = false;
    enableFaceAgeGender = false;
    enableFaceAgeGender2D = false;
    faceID = "";
    if(enableFaceRecognition2D)
        std::cout << "FaceRecognizer.->Starting face recognition..." << std::endl;
    else
        std::cout << "FaceRecognizer.->Stoping face recognition..." << std::endl;
}

void callbackStartFaceAgeGender(const std_msgs::Bool::ConstPtr& msg)
{
    enableFaceRecognition = false;
    enableFaceDetection = false;
    enableFaceRecognition2D = false;
    enableFaceAgeGender = msg->data;
    enableFaceAgeGender2D = false;
    faceID = "";
    if(enableFaceAgeGender)
        std::cout << "FaceRecognizer.->Starting face age gender classifier..." << std::endl;
    else
        std::cout << "FaceRecognizer.->Stoping face age gender classifier..." << std::endl;
}

void callbackStartFaceAgeGender2D(const std_msgs::Bool::ConstPtr& msg)
{
    enableFaceRecognition2D = false;
    enableFaceDetection = false;
    enableFaceRecognition = false;
    enableFaceAgeGender = false;
    enableFaceAgeGender2D = msg->data;
    faceID = "";
    if(enableFaceRecognition2D)
        std::cout << "FaceRecognizer.->Starting face recognition..." << std::endl;
    else
        std::cout << "FaceRecognizer.->Stoping face recognition..." << std::endl;
}

void callbackSetIDFaceRecognition(const std_msgs::String::ConstPtr& msg)
{
    std::cout << "FaceRecognizer.->Set the id for face recognition..." << std::endl;
    faceID = msg->data;
}

void callbackEnableAgeGenderRecognition(const std_msgs::Bool::ConstPtr& msg)
{
    enableFaceAgeGender = false;
    enableFaceAgeGender2D = msg->data;
    if(enableFaceAgeGender2D)
        std::cout << "FaceRecognizer.->Enable age and gender classifier." << std::endl;
    else
        std::cout << "FaceRecognizer.->Disable age and gender classifier." << std::endl;
}

bool callback_srvDetectPanoFaces(vision_msgs::GetFacesFromImage::Request &req, vision_msgs::GetFacesFromImage::Response &resp)
{
    std::cout << "FaceRecognizer.->Starting face detection..." << std::endl;
    sensor_msgs::Image panoramic = req.panoramic_image;
    Mat img = Mat(req.panoramic_image.height, req.panoramic_image.width, CV_8UC3);
    img.data = &req.panoramic_image.data[0];
    std::vector<faceobj> facesdetected = facerecognizer.facialRecognitionPano(img, "");
    vision_msgs::VisionFaceObjects faces_detected;
    if(facesdetected.size() > 0) {
        //Sort vector
        std::sort (facesdetected.begin(), facesdetected.end(), faceobjSortFunction);
        for (int x = 0; x < facesdetected.size(); x++) {
            vision_msgs::VisionFaceObject face;
            geometry_msgs::Point p; 
            face.id = facesdetected[x].id;
            face.confidence = facesdetected[x].confidence;
            face.face_centroid.x = 0;
            face.face_centroid.y = 0;
            face.face_centroid.z = 0;
            p.x = facesdetected[x].boundingbox.x;
            p.y = facesdetected[x].boundingbox.y;
            face.bounding_box.push_back(p);
            p.x = facesdetected[x].boundingbox.x + facesdetected[x].boundingbox.width;
            p.y = facesdetected[x].boundingbox.y + facesdetected[x].boundingbox.height;
            face.bounding_box.push_back(p);
            face.smile = facesdetected[x].smile;
            face.gender = facesdetected[x].gender;
            resp.faces.recog_faces.push_back(face);
        }
    }

    return true;
}

bool callback_srvDetectWaving(vision_msgs::FindWaving::Request &req, vision_msgs::FindWaving::Response &resp)
{
    std::cout << "FaceRecognizer.-> Starting wave detection..." << std::endl;

    std::vector<Rect> wavings = facerecognizer.wavingDetection();

    std::sort (wavings.begin(), wavings.end(), RectSortFunctionSize);

    for (int x = 0; x < wavings.size(); x++) {
        vision_msgs::VisionRect rect;
        rect.x = wavings[x].x;
        rect.y = wavings[x].y;
        rect.width = wavings[x].width;
        rect.height = wavings[x].height;

        resp.bounding_box.push_back(rect);
        resp.frame_width.data = facerecognizer.waveframe_width;
        resp.frame_height.data = facerecognizer.waveframe_height;
    }

    return true;
}

bool callback_srvDetectFaces(vision_msgs::FaceRecognition::Request &req, vision_msgs::FaceRecognition::Response &resp)
{
    std::cout << "FaceRecognizer.-> Starting face detection..." << std::endl;
    cv::Mat bgrImg;
    cv::Mat xyzCloud;
    if (!GetImagesFromJustina(bgrImg, xyzCloud))
        return false;

    resp.faces = faceDetection(bgrImg, xyzCloud);
    return true;
}


bool callback_srvFaceRecognition(vision_msgs::FaceRecognition::Request &req, vision_msgs::FaceRecognition::Response &resp)
{
    std::cout << "FaceRecognizer.-> Starting face recognition..." << std::endl;

    cv::Mat bgrImg;
    cv::Mat xyzCloud;
    if (!GetImagesFromJustina(bgrImg,xyzCloud))
        return false;

    vision_msgs::VisionFaceObjects faces = faceRecognition(req.id, bgrImg, xyzCloud);
    if(req.enable_age_gender)
        faceAgeGender2D(bgrImg, faces);
    resp.faces = faces;

    return true;
}

bool callback_srvFaceRecognition2D(vision_msgs::FaceRecognition::Request &req, vision_msgs::FaceRecognition::Response &resp)
{
    std::cout << "FaceRecognizer.-> Starting face recognition 2D..." << std::endl;

    cv::Mat bgrImg;
    if (!GetImagesFromJustina(bgrImg))
        return false;

    vision_msgs::VisionFaceObjects faces = faceRecognition2D(req.id, bgrImg);
    if(req.enable_age_gender)
        faceAgeGender2D(bgrImg, faces);
    resp.faces = faces;

    return true;
}

bool callback_srvFaceAgeGender(vision_msgs::FaceRecognition::Request &req, vision_msgs::FaceRecognition::Response &resp)
{
    std::cout << "FaceRecognizer.-> Starting face age gender recognition..." << std::endl;

    cv::Mat bgrImg;
    cv::Mat xyzCloud;
    if (!GetImagesFromJustina(bgrImg,xyzCloud))
        return false;

    vision_msgs::VisionFaceObjects faces; 
    faceAgeGender(bgrImg, xyzCloud, faces);
    resp.faces = faces;

    return true;
}

bool callback_srvFaceAgeGender2D(vision_msgs::FaceRecognition::Request &req, vision_msgs::FaceRecognition::Response &resp)
{
    std::cout << "FaceRecognizer.-> Starting face age gender recognition 2D..." << std::endl;

    cv::Mat bgrImg;
    if (!GetImagesFromJustina(bgrImg))
        return false;

    vision_msgs::VisionFaceObjects faces; 
    faceAgeGender2D(bgrImg, faces);
    resp.faces = faces;

    return true;
}

int main(int argc, char** argv)
{

    std::cout << "INITIALIZING FACE RECOGNIZER..." << std::endl;
    ros::init(argc, argv, "face_recognizer");
    ros::NodeHandle n;
    node = &n;

    /* 
       ros::Subscriber subStartRecog = n.subscribe("/vision/face_recognizer/start_recog", 1, callbackStartRecog);
       ros::Subscriber subStopRecog = n.subscribe("/vision/face_recognizer/stop_recog", 1, callbackStopRecog);
       ros::Subscriber subStartRecogOld = n.subscribe("/vision/face_recognizer/start_recog_old", 1, callbackStartRecogOld);
       */

    // Detect pano faces Service
    srvDetectPanoFaces = n.advertiseService("/vision/face_recognizer/detect_pano_faces", callback_srvDetectPanoFaces);
    // Waving service
    srvDetectWave = n.advertiseService("/vision/face_recognizer/detect_waving", callback_srvDetectWaving);
    // Face detection service
    srvDetectFaces = n.advertiseService("/vision/face_recognizer/detect_faces", callback_srvDetectFaces); 
    // Face recognition services
    srvFaceRecognition = n.advertiseService("/vision/facenet_recognizer/face_recognition", callback_srvFaceRecognition);
    srvFaceRecognition2D = n.advertiseService("/vision/facenet_recognizer/face_recognition_2D", callback_srvFaceRecognition2D);
    srvFaceAgeGender = n.advertiseService("/vision/facenet_recognizer/face_age_gender", callback_srvFaceAgeGender);
    srvFaceAgeGender2D = n.advertiseService("/vision/facenet_recognizer/face_age_gender_2D", callback_srvFaceAgeGender2D);

    ros::Subscriber subStartFaceDetection = n.subscribe("/vision/face_recognizer/start_detect", 1, callbackStartFaceDetection);
    ros::Subscriber subStartFaceRecognition = n.subscribe("/vision/face_recognizer/start_recog", 1, callbackStartFaceRecognition);
    ros::Subscriber subStartFaceRecognition2D = n.subscribe("/vision/face_recognizer/start_recog_2D", 1, callbackStartFaceRecognition2D);
    ros::Subscriber subStartFaceAgeGender = n.subscribe("/vision/face_recognizer/start_age_gender", 1, callbackStartFaceAgeGender);
    ros::Subscriber subStartFaceAgeGender2D = n.subscribe("/vision/face_recognizer/start_age_gender_2D", 1, callbackStartFaceAgeGender2D);
    ros::Subscriber subSetFaceRecognition = n.subscribe("/vision/face_recognizer/set_id_face_recognizer", 1, callbackSetIDFaceRecognition);
    ros::Subscriber subEnableFacesAgeGenderRecognition = n.subscribe("/vision/face_recognizer/enable_face_age_gender_recognizer", 1, callbackEnableAgeGenderRecognition);
    // Crear el topico donde se publican los resultados del reconocimiento
    pubFaces = n.advertise<vision_msgs::VisionFaceObjects>("/vision/face_recognizer/faces", 1);

    /* 
    // Suscripcion al topico de entrenamiento
    ros::Subscriber subTrainFace = n.subscribe("/vision/face_recognizer/run_face_trainer", 1, callbackTrainFace);

    // Suscripcion al topico de entrenamiento con ID y numero de frames a entrenar
    ros::Subscriber subTrainFaceNum = n.subscribe("/vision/face_recognizer/run_face_trainer_frames", 1, callbackTrainFaceNum);

    // Suscripcion al topico de reconocimiento (Todos los rostros)
    ros::Subscriber subRecFace = n.subscribe("/vision/face_recognizer/run_face_recognizer", 1, callbackRecFace);

    // Suscripcion al topico de reconocimiento (Por ID)
    ros::Subscriber subRecFaceByID = n.subscribe("/vision/face_recognizer/run_face_recognizer_id", 1, callbackRecFaceByID);

    // Suscripcion al topico para limpiar la base de datos de rostros conocidos (TODOS)
    ros::Subscriber subClearFacesDB = n.subscribe("/vision/face_recognizer/clearfacesdb", 1, callbackClearFacesDB);

    // Suscripcion al topico para limpiar la base de datos de rostros conocidos por ID
    ros::Subscriber subClearFacesDBByID = n.subscribe("/vision/face_recognizer/clearfacesdbbyid", 1, callbackClearFacesDBByID);


    // Crea un topico donde se publica el resultado del entrenamiento
    pubTrainer = n.advertise<std_msgs::Int32>("/vision/face_recognizer/trainer_result", 1);*/
    ros::Subscriber subTrainFaces = n.subscribe("/vision/face_recognizer/trainer_faces", 1, callbackTrainFaces);
    pubTrainer = n.advertise<std_msgs::Int32>("/vision/face_recognizer/trainer_result", 1);

    cltRgbdRobot = n.serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot");
    cltRgbWebCam = n.serviceClient<webcam_man::GetRgb>("/hardware/webcam_man/image_raw");
    cltFaceRecognition = n.serviceClient<vision_msgs::FaceRecognition>("/vision/face_recognizer/faces");
    cltFacesAgeGender = n.serviceClient<vision_msgs::FaceRecognition>("/vision/face_recognizer/faces_age_gender");
    cltFaceTrain = n.serviceClient<vision_msgs::FaceRecognition>("/vision/face_recognizer/train_face");
    cltFaceTrainFlush = n.serviceClient<std_srvs::Empty>("/vision/face_recognizer/train_flush");

    ros::Rate loop(30);

    std::cout << "FaceRecognizer.->Running..." << std::endl;

    while(ros::ok() && cv::waitKey(1) != 'q')
    {
        if(enableFaceDetection){
            cv::Mat bgrImg, xyzCloud;
            if (GetImagesFromJustina(bgrImg, xyzCloud)){
                vision_msgs::VisionFaceObjects faces_detected = faceDetection(bgrImg, xyzCloud);
                pubFaces.publish(faces_detected);
            }
        }
        if(enableFaceRecognition){
            cv::Mat bgrImg, xyzCloud;
            if (GetImagesFromJustina(bgrImg, xyzCloud))
            {
                vision_msgs::VisionFaceObjects faces_recog = faceRecognition(faceID, bgrImg, xyzCloud);
                if(enableFaceAgeGender2D && faces_recog.recog_faces.size() > 0)
                {
                    faceAgeGender2D(bgrImg, faces_recog);
                }
                pubFaces.publish(faces_recog);
            }
        }
        if(enableFaceRecognition2D){
            cv::Mat bgrImg;
            if (GetImagesFromJustina(bgrImg))
            {
                vision_msgs::VisionFaceObjects faces_recog = faceRecognition2D(faceID, bgrImg);
                if(enableFaceAgeGender2D && faces_recog.recog_faces.size() > 0)
                {
                    faceAgeGender2D(bgrImg, faces_recog);    
                }
                pubFaces.publish(faces_recog);
            }
        }
        if(!(enableFaceRecognition || enableFaceDetection || enableFaceRecognition2D) && (enableFaceAgeGender || enableFaceAgeGender2D))
        {
            vision_msgs::VisionFaceObjects faces;
            if(enableFaceAgeGender)
            {
                cv::Mat bgrImg, xyzCloud;
                if (GetImagesFromJustina(bgrImg, xyzCloud))
                {
                    faceAgeGender(bgrImg, xyzCloud, faces);
                    pubFaces.publish(faces);
                }
            }
            if(enableFaceAgeGender2D)
            {
                cv::Mat bgrImg;
                if (GetImagesFromJustina(bgrImg))
                {
                    faceAgeGender2D(bgrImg, faces);
                    pubFaces.publish(faces);
                }
            }
        }
        if(trainNewFace){
            for(int trainedcount = 0; trainedcount < numTrain; trainedcount++)
            {
                cv::Mat bgrImg, xyzCloud;
                if (GetImagesFromJustina(bgrImg))
                {
                    vision_msgs::FaceRecognition srv;
                    srv.request.id = trainID;
                    sensor_msgs::Image container;
                    cv_bridge::CvImage cvi_mat;
                    cvi_mat.encoding = sensor_msgs::image_encodings::BGR8;
                    cvi_mat.image = bgrImg;
                    cvi_mat.toImageMsg(container);
                    srv.request.imageBGR = container;	
                    if(!cltFaceTrain.call(srv))
                        std::cout << "FaceRecognizer.->Not service client face training is working" << std::endl;
                }
            }
            std_srvs::Empty srv;
            if(!cltFaceTrainFlush.call(srv))
                std::cout << "FaceRecognizer.->Not service client face training flush is working" << std::endl;
            trainID = "";
            trainedcount = 0;
            numTrain = 0;
            trainNewFace = false;
        }
        ros::spinOnce();
        loop.sleep();
    }

    cv::destroyAllWindows();
}
