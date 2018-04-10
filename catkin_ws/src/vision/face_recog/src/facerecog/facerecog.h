#pragma once

#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/face.hpp"
#include <iostream>
#include <sstream>
#include <math.h>
#include <string>
#include <cstdlib>
#include "boost/filesystem.hpp"
#include "faceobj.h"

#include "dlib/image_processing/frontal_face_detector.h"
#include "dlib/image_processing/render_face_detections.h"
#include "dlib/image_processing.h"
#include "dlib/opencv/cv_image.h"

using namespace std;
using namespace cv;
using namespace cv::face;
using namespace dlib;

class facerecog
{
private:
	
	bool debugmode;

	bool facerecognitionactive; // Main flag
	bool facedetectionactive; // Main flag
	bool use3D4recognition;

	bool usedlib;
	bool useprofilerecognition;

	string basePath;
	string configFileName;
	string resultsPath;

	string face_cascade_name;
	string profileface_cascade_name;

	string eyes_cascade_name1;
	string eyes_cascade_name2;

	string mouth_cascade_name;
	string nose_cascade_name;
	

	Size maxFaceSize;
	Size faceTrinedSize;

	int minNumFeatures; //Un ojo, nariz y boca; Dos ojos, boca; Dos ojos, nariz
	bool scaleScene;
	double maxErrorThreshold; // Maximo error permitido para reconocer 
	
	// Indica el valor maximo que puede tener cada vector de rostros entrenados
	int maxFacesVectorSize;

	// Indica el valor minimo de caras que se pueden tener como base de entrenamiento
	int minFacesVectorSize;

	string unknownName; // nombre que se le dara a la persona desconocida

	string trainingName;
	string trainingDataPath;
	string trainingData;
	string genderTrainingData;
	string agesTrainingData;
	string smileTrainingData;

	bool genderclassifier; // Gender classifier flag
	bool agesclassifier; // Gender classifier flag
	bool smileclassifier;

	CascadeClassifier face_cascade;
	CascadeClassifier profileface_cascade;
	CascadeClassifier eye_cascade1; //Left eye
	CascadeClassifier eye_cascade2; //right eye
	CascadeClassifier mouth_cascade;
	CascadeClassifier nose_cascade;
	

	/// Internal Variables
	
	// Variables necesarias para cargar la bd de rostros conocidos
	std::vector<string> trainingIDs;
	std::vector<int> trainingCounts;

	// Para almacenar la base de datos de caras
	// Cada vector almacena todas las tomas de cada rostro conocido
	// El numero de vectores corresponde con el numero de caras conocidas
	std::vector<std::vector<Mat> > facesDB;
	std::vector<std::vector<int> > labelsDB;

	/**** Face recognizer ****/
	Ptr<BasicFaceRecognizer> model;
	//Ptr<LBPHFaceRecognizer> modelLBPH;
	
	
	/**** Gender recognizer ****/
	Ptr<FisherFaceRecognizer> gendermodel; //Fisher

    /**** Ages recognizer ****/
    Ptr<FisherFaceRecognizer> agesmodel; //Fisher

	/**** Smile recognizer ****/
	Ptr<FisherFaceRecognizer> smilemodel; //Fisher

	

	//Funciones 
	void setDefaultValues();
	bool initClassifiers();
	std::vector<Rect> faceDetector(Mat sceneImage, bool findAllFaces = false);
	std::vector<Rect> eyesDetector(Mat faceImage);
	std::vector<Rect> mouthDetector(Mat faceImage);
	std::vector<Point> noseDetector(Mat faceImage);
	Mat reconstructFace(Mat preprocessedFace, Mat eigenvectors, Mat meanImage);
	Mat preprocessFace(Mat faceImg, std::vector<Rect> eyesVector, Size imgDesiredSize = Size(100, 120));
	Mat preprocess3DFace(Mat faceImg3D, Size imgDesiredSize = Size(100, 120));
	double getError(const Mat A, const Mat B);
	void tile(const std::vector<Mat> &src, Mat &dst, int grid_x, int grid_y);
	Mat rotate(Mat src, double angle);
	
	std::vector<Rect> profileFaceDetector(Mat sceneImage, bool findAllFaces);
	std::vector<Rect> faceDetectorV2(Mat sceneImage, bool findAllFaces);
	
	/*std::vector<Rect> NonMaximumSuppression(std::vector<Rect> boundingBoxes, double overlapThresh);
	bool sortByY2(Rect i, Rect j);
	bool removeFunction(Rect j, int cx1, int cy1, int cx2, int cy2, double overThres);*/
	
	template < typename T > std::string to_string( const T& n ) {
		std::ostringstream stm;
		stm << n;
		return stm.str();
	}
	
	
	

public:

	int waveframe_width;
	int waveframe_height;

	std::vector<faceobj> facialRecognition(Mat scene2D, Mat scene3D);
	std::vector<faceobj> facialRecognition(Mat scene2D);	
	std::vector<faceobj> facialRecognition(Mat scene2D, Mat scene3D, string faceID);
	std::vector<faceobj> facialRecognitionForever(Mat scene2D, Mat scene3D, string faceID);
	std::vector<faceobj> facialRecognitionPano(Mat scene2D, string faceID);
	bool faceTrainer(Mat scene2D, Mat scene3D, std::string id);
	bool saveConfigFile(string filename);
	bool loadConfigFile(string filename);
	bool loadTrainedData();
	bool clearFaceDB();
	bool clearFaceDB(string id);
	string expand_user(string path);
	
	std::vector<Rect> wavingDetection();
	
	
	facerecog();
	~facerecog();
};

