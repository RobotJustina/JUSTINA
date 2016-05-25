#pragma once

#include "opencv2/opencv.hpp"
#include <iostream>
#include <sstream>
#include <math.h>
#include "faceobj.h"


class facerecog
{
private:
	
	bool debugmode;

	bool facerecognitionactive; // Main flag
	bool facedetectionactive; // Main flag
	bool use3D4recognition;

	string basePath;
	string configFileName;

	string face_cascade_name;

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
	string smileTrainingData;

	bool genderclassifier; // Gender classifier flag
	bool smileclassifier;

	CascadeClassifier face_cascade;
	CascadeClassifier eye_cascade1; //Left eye
	CascadeClassifier eye_cascade2; //right eye
	CascadeClassifier mouth_cascade;
	CascadeClassifier nose_cascade;
	

	/// Internal Variables
	
	// Variables necesarias para cargar la bd de rostros conocidos
	vector<string> trainingIDs;
	vector<int> trainingCounts;

	// Para almacenar la base de datos de caras
	// Cada vector almacena todas las tomas de cada rostro conocido
	// El numero de vectores corresponde con el numero de caras conocidas
	vector<vector<Mat> > facesDB;
	vector<vector<int> > labelsDB;

	/**** Face recognizer ****/
	Ptr<FaceRecognizer> model;
	
	/**** Gender recognizer ****/
	Ptr<FaceRecognizer> gendermodel; //Fisher

	/**** Smile recognizer ****/
	Ptr<FaceRecognizer> smilemodel; //Fisher



	//Funciones 
	void setDefaultValues();
	bool initClassifiers();
	vector<Rect> faceDetector(Mat sceneImage, bool findAllFaces = false);
	vector<Rect> eyesDetector(Mat faceImage);
	vector<Rect> mouthDetector(Mat faceImage);
	vector<Point> noseDetector(Mat faceImage);
	Mat reconstructFace(Mat preprocessedFace, Mat eigenvectors, Mat meanImage);
	Mat preprocessFace(Mat faceImg, vector<Rect> eyesVector, Size imgDesiredSize = Size(100, 120));
	Mat preprocess3DFace(Mat faceImg3D, Size imgDesiredSize = Size(100, 120));
	double getError(const Mat A, const Mat B);
	void tile(const vector<Mat> &src, Mat &dst, int grid_x, int grid_y);
	Mat rotate(Mat src, double angle);
	
	template < typename T > std::string to_string( const T& n ) {
		std::ostringstream stm;
		stm << n;
		return stm.str();
	}
	
	
	

public:

	vector<faceobj> facialRecognition(Mat scene2D, Mat scene3D);
	bool faceTrainer(Mat scene2D, Mat scene3D, string id);
	bool saveConfigFile(string filename);
	bool loadConfigFile(string filename);
	bool loadTrainedData();
	bool clearFaceDB();
	bool clearFaceDB(string id);

	
	facerecog();
	~facerecog();
};

