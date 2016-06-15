#include <iostream>
#include <sstream>
#include <string>
#include <algorithm>    // std::sort
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "vision_msgs/VisionFaceObjects.h"
#include "vision_msgs/VisionFaceObject.h"
#include "vision_msgs/VisionFaceTrainObject.h"
#include "justina_tools/JustinaTools.h"
#include "geometry_msgs/Point.h"

#include "facerecog/facerecog.h"
#include "facerecog/faceobj.h"


using namespace std;
using namespace cv;

ros::Subscriber subPointCloud;
ros::NodeHandle* node;

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

bool faceobjSortFunction (faceobj i,faceobj j) { 
	return (i.boundingbox.x < j.boundingbox.x); 
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
		//vector<faceobj> facesdetected = facerecognizer.facialRecognition(bgrImg, xyzCloud);
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
		vector<faceobj> facesdetected = facerecognizer.facialRecognition(bgrImg, xyzCloud, faceID);
		
		if(facesdetected.size() > 0) {
			//Sort vector
			std::sort (facesdetected.begin(), facesdetected.end(), faceobjSortFunction);
		
			vision_msgs::VisionFaceObjects faces_detected;
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
			
			pubFaces.publish(faces_detected);
			
		}
		
	}
		
	if (recFaceForever) {
		JustinaTools::PointCloud2Msg_ToCvMat(msg, bgrImg, xyzCloud);
		vector<faceobj> facesdetected = facerecognizer.facialRecognitionForever(bgrImg, xyzCloud, faceID);
		
		if(facesdetected.size() > 0) {
			//Sort vector
			std::sort (facesdetected.begin(), facesdetected.end(), faceobjSortFunction);
		
			vision_msgs::VisionFaceObjects faces_detected;
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
			
			pubFaces.publish(faces_detected);
			
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
    // Me suscribo al topico que publica los datos del kinect
    subPointCloud = node->subscribe("/hardware/point_cloud_man/rgbd_wrt_robot", 1, callbackPointCloud);
    
    recFaceForever = true;
}

void callbackStopRecog(const std_msgs::Empty::ConstPtr& msg)
{
	/// NOTHING
    std::cout << "FaceRecognizer.->Stopping face recognition..." << std::endl;
    subPointCloud.shutdown();
    cv::destroyAllWindows();
}

int main(int argc, char** argv)
{
	
    std::cout << "INITIALIZING FACE RECOGNIZER..." << std::endl;
    ros::init(argc, argv, "face_recognizer");
    ros::NodeHandle n;
    node = &n;
    
    ros::Subscriber subStartRecog = n.subscribe("/vision/face_recognizer/start_recog", 1, callbackStartRecog);
    ros::Subscriber subStopRecog = n.subscribe("/vision/face_recognizer/stop_recog", 1, callbackStopRecog);
    
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


