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
#include "vision_msgs/VisionFaceObjects.h"
#include "vision_msgs/VisionFaceObject.h"
#include "vision_msgs/VisionFaceTrainObject.h"
#include "vision_msgs/GetFacesFromImage.h"
#include "vision_msgs/FindWaving.h"
#include "vision_msgs/VisionRect.h"
#include "vision_msgs/FaceRecognition.h"
#include "justina_tools/JustinaTools.h"
#include "geometry_msgs/Point.h"

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>


#include "facerecog/facerecog.h"
#include "facerecog/faceobj.h"


using namespace std;
using namespace cv;

ros::Subscriber subPointCloud;
ros::NodeHandle* node;
ros::ServiceClient cltRgbdRobot;


// Face recognizer
facerecog facerecognizer;
ros::Publisher pubFaces;
ros::Publisher pubTrainer;
bool trainNewFace = false;
bool recFace = false;
bool clearDB = false;
bool clearDBByID = false;
int numTrain = 1;
int trainedcount = 0;
string trainID = "unknown";
string faceID = "";
int trainFailed = 0;
int maxNumFailedTrain = 5;

bool recFaceForever = false;


// Services
ros::ServiceServer srvDetectFaces;
ros::ServiceServer srvDetectWave;
ros::ServiceServer srvFaceRecognition;




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


bool faceobjSortFunction (faceobj i,faceobj j) { 
	return (i.boundingbox.x < j.boundingbox.x); 
}

bool RectSortFunction (Rect i,Rect j) { 
	return (i.x < j.x); 
}

bool RectSortFunctionSize (Rect i,Rect j) { 
	return ((i.height * i.width) > (j.height * j.width)); 
}


void callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    cv::Mat bgrImg;
    cv::Mat xyzCloud;
    //JustinaTools::PointCloud2Msg_ToCvMat(msg, bgrImg, xyzCloud);
    
    
    /** TEST ONLY **/
    // Face recognition
    //int c = waitKey(1);

	//if (c == 'c') {
		//JustinaTools::PointCloud2Msg_ToCvMat(msg, bgrImg, xyzCloud);
		//facerecognizer.clearFaceDB();
	//}
	
	//if (c == 't') {
		//JustinaTools::PointCloud2Msg_ToCvMat(msg, bgrImg, xyzCloud);
		//facerecognizer.faceTrainer(bgrImg, xyzCloud, "TEST");
	//}
	
	//if (c == 'r') {
		//JustinaTools::PointCloud2Msg_ToCvMat(msg, bgrImg, xyzCloud);
		//std::vector<faceobj> facesdetected = facerecognizer.facialRecognition(bgrImg, xyzCloud);
		//for (int x = 0; x < facesdetected.size(); x++) {
			//cout << "Face detected - ID: " << facesdetected[x].id << " Gender: " << facesdetected[x].gender << " Confidence: " << facesdetected[x].confidence
			//<< " Smile: " << facesdetected[x].smile << " Pos: " << facesdetected[x].pos3D << endl;
		//}
	//}
	/** END TEST ONLY **/
	
	
	
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
		
	if (recFaceForever) {
		JustinaTools::PointCloud2Msg_ToCvMat(msg, bgrImg, xyzCloud);
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

void callbackTrainFaceNum(const vision_msgs::VisionFaceTrainObject& msg)
{
	trainID = msg.id;
	numTrain = msg.frames;
	
	if(trainID != "") {
		if (numTrain > 0) { 
			trainNewFace = true;
			trainedcount = 0;
		}
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

void callbackClearFacesDB(const std_msgs::Empty::ConstPtr& msg) 
{
	clearDB = true;
}


void callbackClearFacesDBByID(const std_msgs::String::ConstPtr& msg) 
{
	trainID = msg->data;
	if(trainID != "") {
		clearDBByID = true;
	}
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
}





// For testing only
void callbackTEST(sensor_msgs::Image panoramic_image)
{
	std::cout << "FaceRecognizer.->Starting face recognition..." << std::endl;
	    
    Mat img = Mat(panoramic_image.height, panoramic_image.width, CV_8UC3);

    img.data = &panoramic_image.data[0];
    
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
			
			//cv::rectangle(img, facesdetected[x].boundingbox, CV_RGB(255,0,0));
			
		}
		

	}
    
    //imshow("TEST", img);
    
}

bool callback_srvDetectFaces(vision_msgs::GetFacesFromImage::Request &req, vision_msgs::GetFacesFromImage::Response &resp)
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



bool callback_srvFaceRecognition(vision_msgs::FaceRecognition::Request &req, vision_msgs::FaceRecognition::Response &resp)
{
    std::cout << "FaceRecognizer.-> Starting face recognition..." << std::endl;
    
    cv::Mat bgrImg;
    cv::Mat xyzCloud;
    if (!GetImagesFromJustina(bgrImg,xyzCloud))
        return false;
    
    string fid = req.id;
    cout << "Searching for " << fid << endl;
    
    std::vector<faceobj> facesdetected = facerecognizer.facialRecognitionForever(bgrImg, xyzCloud, fid);
		
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
			
			resp.faces.recog_faces.push_back(face);
		}
	}
	
    return true;
}




int main(int argc, char** argv)
{
	
    std::cout << "INITIALIZING FACE RECOGNIZER..." << std::endl;
    ros::init(argc, argv, "face_recognizer");
    ros::NodeHandle n;
    node = &n;
    
    //TEST
    //ros::Subscriber subTESTPano = n.subscribe("/vision/pano_maker/panoramic_image", 1, callbackTEST);
    
    
    
    
    
    ros::Subscriber subStartRecog = n.subscribe("/vision/face_recognizer/start_recog", 1, callbackStartRecog);
    
    ros::Subscriber subStopRecog = n.subscribe("/vision/face_recognizer/stop_recog", 1, callbackStopRecog);
    // TEST
    ros::Subscriber subStartRecogOld = n.subscribe("/vision/face_recognizer/start_recog_old", 1, callbackStartRecogOld);
    
    
    // Service
    srvDetectFaces = n.advertiseService("/vision/face_recognizer/detect_faces", callback_srvDetectFaces);
    
    // Waving service
    srvDetectWave = n.advertiseService("/vision/face_recognizer/detect_waving", callback_srvDetectWaving);
    
    // face recognition service
    srvFaceRecognition = n.advertiseService("/vision/face_recognizer/face_recognition", callback_srvFaceRecognition);
    
    
    
    
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
    
    // Crear el topico donde se publican los resultados del reconocimiento
    pubFaces = n.advertise<vision_msgs::VisionFaceObjects>("/vision/face_recognizer/faces", 1);
    
    // Crea un topico donde se publica el resultado del entrenamiento
    pubTrainer = n.advertise<std_msgs::Int32>("/vision/face_recognizer/trainer_result", 1);
    
    
    cltRgbdRobot = n.serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot");
    
    ros::Rate loop(30);
    
    std::cout << "FaceRecognizer.->Running..." << std::endl;
    
    while(ros::ok() && cv::waitKey(1) != 'q')
    {
        ros::spinOnce();
        loop.sleep();
    }
    
    subPointCloud.shutdown();
    subTrainFace.shutdown();
    subRecFace.shutdown();
    cv::destroyAllWindows();
    
}


