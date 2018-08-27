#include "facerecog.h"

using namespace std;
using namespace cv;
using namespace dlib;

facerecog::facerecog()
{
	try {

		// Loading default values for all variables
		setDefaultValues();
		
		//loading config file
		if (!loadConfigFile(configFileName)) {

			// Creating file structure
			if( !boost::filesystem::exists(basePath)) //base path
				boost::filesystem::create_directory(basePath); 
				
			if( !boost::filesystem::exists(trainingDataPath)) // training data path
				boost::filesystem::create_directory(trainingDataPath); 
				
			if( !boost::filesystem::exists(resultsPath)) // results path
				boost::filesystem::create_directory(resultsPath); 

			saveConfigFile(configFileName);
			cout << "Default config file created: " << configFileName << endl;
		}
		else {
			// Creating file structure	
			if( !boost::filesystem::exists(trainingDataPath)) // training data path
				boost::filesystem::create_directory(trainingDataPath); 
				
			if( !boost::filesystem::exists(resultsPath)) // results path
				boost::filesystem::create_directory(resultsPath); 
				
			cout << "Config file loaded: " << configFileName << endl;
		}

		if (!initClassifiers()) {
			facedetectionactive = false;
			cout << "Initializing classifiers... Failed!" << endl;
		}
		else {
			facedetectionactive = true;
			cout << "Initializing classifiers... Success!" << endl;
		}

		if (facedetectionactive) {
			/**** Face recognizer ****/
            model = EigenFaceRecognizer::create(); //Eigen
			// model = createFisherFaceRecognizer(); //Fisher
			// modelLBPH = createLBPHFaceRecognizer(); //Local Binary Patterns Histograms

			/**** Gender recognizer ****/
            gendermodel = FisherFaceRecognizer::create(); //Fisher

            /**** Ages recognizer ****/
            agesmodel = FisherFaceRecognizer::create();  //Fisher
			
			/**** Smile recognizer ****/
            smilemodel = FisherFaceRecognizer::create(); //Fisher

			if (!loadTrainedData()) {
				facerecognitionactive = false;
				cout << "Loading training data... Failed!" << endl;
			}
			else {
				facerecognitionactive = true;
				cout << "Loading training data... Success!" << endl;
			}

		}
	}
	catch (...) {
		facerecognitionactive = false;
		facedetectionactive = false;
		cout << "Exception while creating face recognizer object." << endl;
	}
}


facerecog::~facerecog()
{
}

void facerecog::setDefaultValues() 
{
	debugmode = false;

	facerecognitionactive = false; // Main flag
	facedetectionactive = false; // Main flag
	use3D4recognition = false;

	usedlib = false;
	useprofilerecognition = false;

	basePath = expand_user("~/facerecog/");
	//basePath = "";
	
	
	//string basePathGender = "../JUSTINA/catkin_ws/src/vision/face_recog/facerecog/";
	string basePathGender = expand_user("~/JUSTINA/catkin_ws/src/vision/face_recog/facerecog/");
	configFileName = basePath + "facerecogconfig.xml";
	resultsPath = expand_user("~/faces/");

	//face_cascade_name = "/usr/share/opencv/lbpcascades/lbpcascade_profileface.xml";
	face_cascade_name = expand_user("~/JUSTINA/catkin_ws/src/vision/face_recog/facerecog_config/haarcascades/haarcascade_frontalface_alt.xml");
	
	//TESTING
	profileface_cascade_name = expand_user("~/JUSTINA/catkin_ws/src/vision/face_recog/facerecog_config/lbpcascade_profileface.xml");
	//profileface_cascade_name = expand_user("~/JUSTINA/catkin_ws/src/vision/face_recog/facerecog_config/haarcascades/haarcascade_profileface.xml");
	

	eyes_cascade_name1 = expand_user("~/JUSTINA/catkin_ws/src/vision/face_recog/facerecog_config/haarcascades/haarcascade_mcs_lefteye.xml");
	eyes_cascade_name2 = expand_user("~/JUSTINA/catkin_ws/src/vision/face_recog/facerecog_config/haarcascades/haarcascade_mcs_righteye.xml");

	mouth_cascade_name = expand_user("~/JUSTINA/catkin_ws/src/vision/face_recog/facerecog_config/haarcascades/haarcascade_mcs_mouth.xml");
	nose_cascade_name = expand_user("~/JUSTINA/catkin_ws/src/vision/face_recog/facerecog_config/haarcascades/haarcascade_mcs_nose.xml");

	maxFaceSize = Size(200, 200);
	faceTrinedSize = Size(100, 120);

	minNumFeatures = 3; //Un ojo, nariz y boca; Dos ojos, boca; Dos ojos, nariz
	scaleScene = false;
	maxErrorThreshold = 0.1; // Maximo error permitido para reconocer 
	
	// Indica el valor maximo que puede tener cada vector de rostros entrenados
	maxFacesVectorSize = 50;

	// Indica el valor minimo de caras que se pueden tener como base de entrenamiento
	minFacesVectorSize = 10;

	unknownName = "unknown"; // nombre que se le dara a la persona desconocida

	trainingName = basePath + "recfac.xml";
	trainingDataPath = basePath + "data/";
	//trainingDataPath = basePath;
	trainingData = basePath + "eigenfaces.xml";
	genderTrainingData = basePathGender + "efgender.xml";
	agesTrainingData = basePathGender + "efages.xml";
	smileTrainingData = basePathGender + "efsmile.xml";
	

	genderclassifier = false; // Gender classifier flag
	smileclassifier = false;
}

std::vector<faceobj> facerecog::facialRecognitionForever(Mat scene2D, Mat scene3D, string faceID)
{
	std::vector<faceobj> facesdetected;
	try {
		if (scaleScene){	
			resize(scene2D, scene2D, Size(scene2D.cols * 2, scene2D.rows * 2));
			resize(scene3D, scene3D, Size(scene3D.cols * 2, scene3D.rows * 2));
		}
		
		Mat sceneRGB = scene2D.clone();
		Mat sceneXYZ = scene3D.clone();
		Mat sceneRGBID1 = scene2D.clone(); //For id identification
		Mat sceneRGBID2 = scene2D.clone();
		
		facesdetected = facialRecognition(sceneRGB, sceneXYZ);
		std::vector<faceobj> facesdetected2save = facesdetected;
		double bestConfidence = 0.0;
		int bestConfidenceIdx = -1;
		
		
		
		for(int x = 0; x < (int)facesdetected.size(); x++) { //for each face detected
			if(faceID == facesdetected[x].id) { //If we found the face requested
				if(facesdetected[x].confidence >  bestConfidence) {
					bestConfidence = facesdetected[x].confidence;
					bestConfidenceIdx = x;			
				}
			} 
		}
		
		if(bestConfidence > 0.0) { // I found you =D

			// **** Prints info in image to show **** //
			cv::rectangle(scene2D, facesdetected[bestConfidenceIdx].boundingbox, CV_RGB(0, 255, 0), 4, 8, 0);
			//Name label
			putText(scene2D, faceID, Point(facesdetected[bestConfidenceIdx].boundingbox.x + 5, 
				facesdetected[bestConfidenceIdx].boundingbox.y + 15), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
			//Confidence
			string textConf = "CONF: " + to_string(facesdetected[bestConfidenceIdx].confidence);
			putText(scene2D, textConf,
				Point(facesdetected[bestConfidenceIdx].boundingbox.x + 5, facesdetected[bestConfidenceIdx].boundingbox.y + 30), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
			//Gender label
			string genderText = "GENDER: " + (facesdetected[bestConfidenceIdx].gender == faceobj::male ? String("MALE") : String("FEMALE"));
			putText(scene2D, genderText,
				Point(facesdetected[bestConfidenceIdx].boundingbox.x + 5, facesdetected[bestConfidenceIdx].boundingbox.y + 45), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
			//Mood label
			string smileText = (facesdetected[bestConfidenceIdx].smile ? String("HAPPY") : String("SAD"));
			putText(scene2D, smileText,
				Point(facesdetected[bestConfidenceIdx].boundingbox.x + 5, facesdetected[bestConfidenceIdx].boundingbox.y + 60), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
			//Ages label
			string agesText = (facesdetected[bestConfidenceIdx].ages == 0 ? String("Children") : (facesdetected[bestConfidenceIdx].ages == 1 ? String("Adult") : String("Elder")));
			putText(scene2D, agesText,
				Point(facesdetected[bestConfidenceIdx].boundingbox.x + 5, facesdetected[bestConfidenceIdx].boundingbox.y + 75), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
			
			
			// **** Prints info in image to save (The Face) **** //
			cv::rectangle(sceneRGBID1, facesdetected[bestConfidenceIdx].boundingbox, CV_RGB(0, 255, 0), 4, 8, 0);
			//Name label
			putText(sceneRGBID1, faceID, Point(facesdetected[bestConfidenceIdx].boundingbox.x + 5, 
				facesdetected[bestConfidenceIdx].boundingbox.y + 15), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
			//Gender label
			genderText = (facesdetected[bestConfidenceIdx].gender == faceobj::male ? String("Male") : String("Female"));
			putText(sceneRGBID1, genderText,
				Point(facesdetected[bestConfidenceIdx].boundingbox.x + 5, facesdetected[bestConfidenceIdx].boundingbox.y + 30), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
			
			// **** Prints info in image to save (All faces) **** //
			for(int x = 0; x < (int)facesdetected.size(); x++) { //for each face detected
				//Name label
				if(x == bestConfidenceIdx) {
					cv::rectangle(sceneRGBID2, facesdetected[x].boundingbox, CV_RGB(0, 255, 0), 4, 8, 0);
					putText(sceneRGBID2, faceID,
						Point(facesdetected[x].boundingbox.x + 5, 
						facesdetected[x].boundingbox.y + 15), 
						FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
				}
				else {
					cv::rectangle(sceneRGBID2, facesdetected[x].boundingbox, CV_RGB(255, 0, 0), 4, 8, 0);
					putText(sceneRGBID2, unknownName,
						Point(facesdetected[x].boundingbox.x + 5, 
						facesdetected[x].boundingbox.y + 15), 
						FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
				}
				//Gender label
				genderText = (facesdetected[x].gender == faceobj::male ? String("Male") : String("Female"));
				putText(sceneRGBID2, genderText,
					Point(facesdetected[x].boundingbox.x + 5, facesdetected[x].boundingbox.y + 30), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
			}
			
			
			// *** Prepare data to be returned **** //
			faceobj theFace = facesdetected[bestConfidenceIdx];
			facesdetected.clear();
			facesdetected.push_back(theFace);
			
			
			imwrite(resultsPath + faceID + "Scene.png", sceneRGBID1); //One face
			imwrite(resultsPath + "sceneComplete.png", sceneRGBID2); //All faces
			
			
		} else { // No coincidences
			
			// If they actually were looking for a face
			if(faceID != "") {
				for(int x = 0; x < (int)facesdetected.size(); x++) { //for each face detected
					//Name label
					cv::rectangle(sceneRGBID2, facesdetected[x].boundingbox, CV_RGB(255, 0, 0), 4, 8, 0);
					putText(sceneRGBID2, unknownName,
						Point(facesdetected[x].boundingbox.x + 5, 
						facesdetected[x].boundingbox.y + 15), 
						FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
					//Gender label
					string genderText = (facesdetected[x].gender == faceobj::male ? String("Male") : String("Female"));
					putText(sceneRGBID2, genderText,
						Point(facesdetected[x].boundingbox.x + 5, facesdetected[x].boundingbox.y + 30), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
                    //Ages label
                    string agesText = (facesdetected[x].ages == 0 ? String("Children") : (facesdetected[x].ages == 1 ? String("Adult") : String("Elder")));
                    putText(sceneRGBID2, agesText,
                        Point(facesdetected[x].boundingbox.x + 5, facesdetected[x].boundingbox.y + 75), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
				}
				facesdetected.clear();
				imwrite(resultsPath + "sceneComplete.png", sceneRGBID2); //All faces
			}
			else {
				// If they wanted all the faces
				
				
				for(int x = 0; x < (int)facesdetected.size(); x++) { //for each face detected
					////// IMAGE TO SAVE
					//Name label
					cv::rectangle(sceneRGBID2, facesdetected[x].boundingbox, CV_RGB(255, 0, 0), 4, 8, 0);
					putText(sceneRGBID2, facesdetected[x].id,
						Point(facesdetected[x].boundingbox.x + 5, 
						facesdetected[x].boundingbox.y + 15), 
						FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
					//Gender label
					string genderText = (facesdetected[x].gender == faceobj::male ? String("Male") : String("Female"));
					putText(sceneRGBID2, genderText,
						Point(facesdetected[x].boundingbox.x + 5, facesdetected[x].boundingbox.y + 30), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
						
						
					////// IMAGE TO SHOW
					cv::rectangle(scene2D, facesdetected[x].boundingbox, CV_RGB(255, 0, 0), 4, 8, 0);
					//Name label
					putText(scene2D, facesdetected[x].id, Point(facesdetected[x].boundingbox.x + 5, 
						facesdetected[x].boundingbox.y + 15), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
					//Confidence
					string textConf = "CONF: " + to_string(facesdetected[x].confidence);
					putText(scene2D, textConf,
						Point(facesdetected[x].boundingbox.x + 5, facesdetected[x].boundingbox.y + 30), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
					//Gender label
					genderText = "GENDER: " + (facesdetected[x].gender == faceobj::male ? String("MALE") : String("FEMALE"));
					putText(scene2D, genderText,
						Point(facesdetected[x].boundingbox.x + 5, facesdetected[x].boundingbox.y + 45), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
					//Mood label
					string smileText = (facesdetected[x].smile ? String("HAPPY") : String("SAD"));
					putText(scene2D, smileText,
						Point(facesdetected[x].boundingbox.x + 5, facesdetected[x].boundingbox.y + 60), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
                    //Ages label
                    string agesText = (facesdetected[x].ages == 0 ? String("Children") : (facesdetected[x].ages == 1 ? String("Adult") : String("Elder")));
                    putText(scene2D, agesText,
                        Point(facesdetected[x].boundingbox.x + 5, facesdetected[x].boundingbox.y + 75), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
					
				}
				
				imwrite(resultsPath + "sceneComplete.png", sceneRGBID2); //All faces
				
				
			}
		}
		
		imshow("Face Recog", scene2D);
		
		
	} catch(...) {
		cout << "Face recognizer exception." << endl;
	}
	return facesdetected;
}


// For panoramic image only. Point cloud not supported.
std::vector<faceobj> facerecog::facialRecognitionPano(Mat scene2D, string faceID)
{
	std::vector<faceobj> facesdetected;
	try {
		if (scaleScene){	
			resize(scene2D, scene2D, Size(scene2D.cols * 2, scene2D.rows * 2));
		}
		
		Mat sceneRGB = scene2D.clone();
		Mat sceneRGBID1 = scene2D.clone(); //For id identification
		Mat sceneRGBID2 = scene2D.clone();
		
		facesdetected = facialRecognition(sceneRGB);
		std::vector<faceobj> facesdetected2save = facesdetected;
		double bestConfidence = 0.0;
		int bestConfidenceIdx = -1;
		
		
		
		for(int x = 0; x < (int)facesdetected.size(); x++) { //for each face detected
			if(faceID == facesdetected[x].id) { //If we found the face requested
				if(facesdetected[x].confidence >  bestConfidence) {
					bestConfidence = facesdetected[x].confidence;
					bestConfidenceIdx = x;			
				}
			} 
		}
		
		if(bestConfidence > 0.0) { // I found you =D
			
			// **** Prints info in image to show **** //
			cv::rectangle(scene2D, facesdetected[bestConfidenceIdx].boundingbox, CV_RGB(0, 255, 0), 4, 8, 0);
			//Name label
			putText(scene2D, faceID, Point(facesdetected[bestConfidenceIdx].boundingbox.x + 5, 
				facesdetected[bestConfidenceIdx].boundingbox.y + 15), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
			//Confidence
			string textConf = "CONF: " + to_string(facesdetected[bestConfidenceIdx].confidence);
			putText(scene2D, textConf,
				Point(facesdetected[bestConfidenceIdx].boundingbox.x + 5, facesdetected[bestConfidenceIdx].boundingbox.y + 30), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
			//Gender label
			string genderText = "GENDER: " + (facesdetected[bestConfidenceIdx].gender == faceobj::male ? String("MALE") : String("FEMALE"));
			putText(scene2D, genderText,
				Point(facesdetected[bestConfidenceIdx].boundingbox.x + 5, facesdetected[bestConfidenceIdx].boundingbox.y + 45), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
			//Mood label
			string smileText = (facesdetected[bestConfidenceIdx].smile ? String("HAPPY") : String("SAD"));
			putText(scene2D, smileText,
				Point(facesdetected[bestConfidenceIdx].boundingbox.x + 5, facesdetected[bestConfidenceIdx].boundingbox.y + 60), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
			
			
			// **** Prints info in image to save (The Face) **** //
			cv::rectangle(sceneRGBID1, facesdetected[bestConfidenceIdx].boundingbox, CV_RGB(0, 255, 0), 4, 8, 0);
			//Name label
			putText(sceneRGBID1, faceID, Point(facesdetected[bestConfidenceIdx].boundingbox.x + 5, 
				facesdetected[bestConfidenceIdx].boundingbox.y + 15), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
			//Gender label
			genderText = (facesdetected[bestConfidenceIdx].gender == faceobj::male ? String("Male") : String("Female"));
			putText(sceneRGBID1, genderText,
				Point(facesdetected[bestConfidenceIdx].boundingbox.x + 5, facesdetected[bestConfidenceIdx].boundingbox.y + 30), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
			
			// **** Prints info in image to save (All faces) **** //
			for(int x = 0; x < (int)facesdetected.size(); x++) { //for each face detected
				//Name label
				if(x == bestConfidenceIdx) {
					cv::rectangle(sceneRGBID2, facesdetected[x].boundingbox, CV_RGB(0, 255, 0), 4, 8, 0);
					putText(sceneRGBID2, faceID,
						Point(facesdetected[x].boundingbox.x + 5, 
						facesdetected[x].boundingbox.y + 15), 
						FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
				}
				else {
					cv::rectangle(sceneRGBID2, facesdetected[x].boundingbox, CV_RGB(255, 0, 0), 4, 8, 0);
					putText(sceneRGBID2, unknownName,
						Point(facesdetected[x].boundingbox.x + 5, 
						facesdetected[x].boundingbox.y + 15), 
						FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
				}
				//Gender label
				genderText = (facesdetected[x].gender == faceobj::male ? String("Male") : String("Female"));
				putText(sceneRGBID2, genderText,
					Point(facesdetected[x].boundingbox.x + 5, facesdetected[x].boundingbox.y + 30), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
			}
			
			
			// *** Prepare data to be returned **** //
			faceobj theFace = facesdetected[bestConfidenceIdx];
			facesdetected.clear();
			facesdetected.push_back(theFace);
			
			
			imwrite(resultsPath + faceID + "Scene_pano.png", sceneRGBID1); //One face
			imwrite(resultsPath + "sceneComplete_pano.png", sceneRGBID2); //All faces
			
			
		} else { // No coincidences
			
			// If they actually were looking for a face
			if(faceID != "") {
				for(int x = 0; x < (int)facesdetected.size(); x++) { //for each face detected
					//Name label
					cv::rectangle(sceneRGBID2, facesdetected[x].boundingbox, CV_RGB(255, 0, 0), 4, 8, 0);
					putText(sceneRGBID2, unknownName,
						Point(facesdetected[x].boundingbox.x + 5, 
						facesdetected[x].boundingbox.y + 15), 
						FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
					//Gender label
					string genderText = (facesdetected[x].gender == faceobj::male ? String("Male") : String("Female"));
					putText(sceneRGBID2, genderText,
						Point(facesdetected[x].boundingbox.x + 5, facesdetected[x].boundingbox.y + 30), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
				}
				facesdetected.clear();
				imwrite(resultsPath + "sceneComplete_pano.png", sceneRGBID2); //All faces
			}
			else {
				// If they wanted all the faces
				
				
				for(int x = 0; x < (int)facesdetected.size(); x++) { //for each face detected
					////// IMAGE TO SAVE
					//Name label
					cv::rectangle(sceneRGBID2, facesdetected[x].boundingbox, CV_RGB(255, 0, 0), 4, 8, 0);
					putText(sceneRGBID2, facesdetected[x].id,
						Point(facesdetected[x].boundingbox.x + 5, 
						facesdetected[x].boundingbox.y + 15), 
						FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
					//Gender label
					string genderText = (facesdetected[x].gender == faceobj::male ? String("Male") : String("Female"));
					putText(sceneRGBID2, genderText,
						Point(facesdetected[x].boundingbox.x + 5, facesdetected[x].boundingbox.y + 30), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
						
						
					////// IMAGE TO SHOW
					cv::rectangle(scene2D, facesdetected[x].boundingbox, CV_RGB(255, 0, 0), 4, 8, 0);
					//Name label
					putText(scene2D, facesdetected[x].id, Point(facesdetected[x].boundingbox.x + 5, 
						facesdetected[x].boundingbox.y + 15), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
					//Confidence
					string textConf = "CONF: " + to_string(facesdetected[x].confidence);
					putText(scene2D, textConf,
						Point(facesdetected[x].boundingbox.x + 5, facesdetected[x].boundingbox.y + 30), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
					//Gender label
					genderText = "GENDER: " + (facesdetected[x].gender == faceobj::male ? String("MALE") : String("FEMALE"));
					putText(scene2D, genderText,
						Point(facesdetected[x].boundingbox.x + 5, facesdetected[x].boundingbox.y + 45), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
					//Mood label
					string smileText = (facesdetected[x].smile ? String("HAPPY") : String("SAD"));
					putText(scene2D, smileText,
						Point(facesdetected[x].boundingbox.x + 5, facesdetected[x].boundingbox.y + 60), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
					
				}
				
				imwrite(resultsPath + "sceneComplete_pano.png", sceneRGBID2); //All faces
				
				
			}
		}
		
		imshow("Face Recog", scene2D);
		
		
	} catch(...) {
		cout << "Face recognizer exception." << endl;
	}
	return facesdetected;
}





std::vector<faceobj> facerecog::facialRecognition(Mat scene2D, Mat scene3D, string faceID)
{
	std::vector<faceobj> facesdetected;
	try {
		if (scaleScene){	
			resize(scene2D, scene2D, Size(scene2D.cols * 2, scene2D.rows * 2));
			resize(scene3D, scene3D, Size(scene3D.cols * 2, scene3D.rows * 2));
		}
		
		Mat sceneRGB = scene2D.clone();
		Mat sceneXYZ = scene3D.clone();
		Mat sceneRGBID = scene2D.clone(); //For id identification
		
		facesdetected = facialRecognition(sceneRGB, sceneXYZ);
		double bestConfidence = 0.0;
		
		for(int x = 0; x < (int)facesdetected.size(); x++) { //for each face detected
			if(faceID == facesdetected[x].id) { //If we found the face requested
				if(facesdetected[x].confidence >  bestConfidence) {
					bestConfidence = facesdetected[x].confidence;
					Mat sceneRGBID2Save = sceneRGBID.clone();
					
					//Bounding box
					cv::rectangle(sceneRGBID2Save, facesdetected[x].boundingbox, CV_RGB(0, 255, 0), 4, 8, 0);
					//Name label
					putText(sceneRGBID2Save, faceID,
						Point(facesdetected[x].boundingbox.x + 5, facesdetected[x].boundingbox.y + 15), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
					//Gender label
					string genderText = (facesdetected[x].gender == faceobj::male ? String("MALE") : String("FEMALE"));
					putText(sceneRGBID2Save, genderText,
						Point(facesdetected[x].boundingbox.x + 5, facesdetected[x].boundingbox.y + 30), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
					//Mood label
					string smileText = (facesdetected[x].smile ? String("HAPPY") : String("SAD"));
					putText(sceneRGBID2Save, smileText,
						Point(facesdetected[x].boundingbox.x + 5, facesdetected[x].boundingbox.y + 45), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
								
					imwrite(resultsPath + faceID + "Scene.png", sceneRGBID2Save);
				}
			}
			cv::rectangle(scene2D, facesdetected[x].boundingbox, CV_RGB(255, 0, 0), 4, 8, 0);
			//Gender label
			string genderText = (facesdetected[x].gender == faceobj::male ? String("MALE") : String("FEMALE"));
			putText(scene2D, genderText,
				Point(facesdetected[x].boundingbox.x + 5, facesdetected[x].boundingbox.y + 15), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
			//Mood label
			string smileText = (facesdetected[x].smile ? String("HAPPY") : String("SAD"));
			putText(scene2D, smileText,
				Point(facesdetected[x].boundingbox.x + 5, facesdetected[x].boundingbox.y + 30), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
		}
		imwrite(resultsPath + "sceneComplete.png", scene2D);
		
	} catch(...) {
		cout << "Face recognizer exception." << endl;
	}
	return facesdetected;
}

std::vector<faceobj> facerecog::facialRecognition(Mat scene2D, Mat scene3D)
{
	std::vector<faceobj> facesdetected;
	if (facesDB.size() != 0) facerecognitionactive = true;
	else facerecognitionactive = false;

	if (facedetectionactive) {
		try{
			
			int count = 0;
			Mat frame_gray;

			cvtColor(scene2D, frame_gray, CV_BGR2GRAY);
			Mat grayTemp = frame_gray.clone();
			//equalizeHist(frame_gray, frame_gray); // Ecualiza la escena para 'mejorar' la deteccion de rostros

			std::vector<Rect> faces; //Vector donde se almacenaran los bounding box de cada rostro detectado

			faces = faceDetector(frame_gray, true);
			
			
			
			//Obtenemos la cara promedio
			Mat meanFace = model->getMean();

			//Obtenemos los eigenvectores
			/*Mat eigenvectors = model->get<Mat>("eigenvectors");
			Mat eigenvalues = model->get<Mat>("eigenvalues");*/
			Mat eigenvectors = model->getEigenVectors();
			Mat eigenvalues = model->getEigenValues();

			for (int i = 0; i < faces.size(); i++)
			{
				count = 0; //Inicializa el contador de elementos. Se necesitan al menos 3

				Mat faceImg = grayTemp(faces[i]).clone();
				Mat faceImgOriginal = faceImg.clone();
				Mat faceImgRGB = scene2D(faces[i]).clone();
				equalizeHist(faceImg, faceImg); //Ecualizo la cara original

				//Redimensiona la imagen del rostro a un tamaño mas conveniente
				resize(faceImg, faceImg, maxFaceSize);
				resize(faceImgOriginal, faceImgOriginal, maxFaceSize);
				resize(faceImgRGB, faceImgRGB, maxFaceSize);

				//Deteccion de ojos
				std::vector<Rect> eyesDetected = eyesDetector(faceImg);
				for (int e = 0; e < eyesDetected.size(); e++) {
					cv::rectangle(faceImgRGB, eyesDetected[e], CV_RGB(0, 0, 255), 1, 8, 0);
					count++;
				}

				// Deteccion de boca
				std::vector<Rect> mouthDetected = mouthDetector(faceImg);
				for (int m = 0; m < mouthDetected.size(); m++) {
					cv::rectangle(faceImgRGB, mouthDetected[m], CV_RGB(0, 0, 255), 1, 8, 0);
					count++;
				}

				// Deteccion de nariz
				std::vector<Point> noseDetected = noseDetector(faceImg);
				for (int n = 0; n < noseDetected.size(); n++) {
					circle(faceImgRGB, noseDetected[n], 5, CV_RGB(0, 0, 255), CV_FILLED, 8, 0);
					count++;
				}


				// Muestra un recuadro indicando la posicion del rostro detectado en el frame original
				if (count >= minNumFeatures) { //filtra por numero de caracteristicas detectadas

					//Encierra en un recuadro el rostro detectado
					if (debugmode)cv::rectangle(scene2D, faces[i], CV_RGB(0, 255, 0), 4, 8, 0);

					// Realiza un preprocesamiento del rostro detectado
					Mat preprocessedface;
					Mat preprocessedface3D;
					Rect roi3D;
					
					// Relacion 100*120. Cambiar si la relacion de la imagen a entrenar cambia
					roi3D.x = cvRound(faces[i].x + (faces[i].width * 0.1));
					roi3D.y = cvRound(faces[i].y + (faces[i].height * 0.02));
					roi3D.width = cvRound(faces[i].width * 0.8);
					roi3D.height = cvRound(faces[i].height * 0.96);

					Mat facexyz = scene3D(roi3D).clone();
					
					Vec3f face3Dcenter = facexyz.at<cv::Vec3f>(facexyz.rows * 0.5, facexyz.cols * 0.5);
					
					
					if(debugmode) imshow("2D", scene2D(roi3D).clone());
					if(debugmode) imshow("3D", facexyz);
					
					preprocessedface3D = preprocess3DFace(facexyz);
					preprocessedface = preprocessFace(faceImgOriginal, eyesDetected);
					
					if (debugmode) imshow("Proc", preprocessedface);
					if (debugmode) imshow("Proc3D", preprocessedface3D);

					double confidence = 0.0;
					String textName = unknownName;
					if (facerecognitionactive) {

						//Intenta identificar el rostro
						Mat reconstructedFace;
						if(use3D4recognition) {
							reconstructedFace = reconstructFace(preprocessedface3D, eigenvectors, meanFace);
						}
						else {
							reconstructedFace = reconstructFace(preprocessedface, eigenvectors, meanFace);
						}
						
						double imgError = 1.0;
						int clase = -1;
						for (int x = 0; x < facesDB.size(); x++) {
							for (int y = 0; y < facesDB[x].size(); y++) { //Comparamos con las imagenes de entrenamiento
								double error = getError(reconstructedFace, facesDB[x][y]);
								if (error < imgError) {
									imgError = error;
									clase = labelsDB[x][y]; // Para cada rostro, la clase es la misma :P 
								}

							}
						}

						confidence = 1.0 - imgError;

						textName = unknownName;
						String textConf = "Conf: " + to_string(confidence);

						if (imgError <= maxErrorThreshold) { // maxErrorThreshold es el error maximo aceptado para conciderar la prediccion correcta
							textName = trainingIDs[clase];
						}
						// Muestra el nombre y la prediccion en pantalla

						if (debugmode) {
							putText(scene2D, textName,
								Point(faces[i].x + 5, faces[i].y + 15), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
							putText(scene2D, textConf,
								Point(faces[i].x + 5, faces[i].y + 30), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
						}

					}
					
					
					// Clasificador de genero
					faceobj facedetectedobj;
					// Asegurarse de que la clase 0 es para hombres y la clase 1 para mujeres
					faceobj::Gender genderClass = faceobj::unknown;
					if (genderclassifier) {
						double genderConf = 0.0;
						int genderpredicted = 0;
						gendermodel->predict(preprocessedface, genderpredicted, genderConf);
						genderClass = genderpredicted <= 0 ? faceobj::male : faceobj::female;
					
						if (debugmode) {
							string genderText = "Gender: " + (genderClass == faceobj::male ? String("Male") : String("Female"));
							putText(scene2D, genderText,
								Point(faces[i].x + 5, faces[i].y + 45), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
						}
					}
					
					// Deteccion de sonrisa
					// Clasificador de sonrisas
					// Clase 0: Serio, Clase 1: Sonrisa
					bool smileDetected = false;
					if (smileclassifier) {
						double smileConf = 0.0;
						int smilepredicted = 0;
						Mat smileFace = preprocessedface(Rect(0, preprocessedface.rows * 0.6, preprocessedface.cols, preprocessedface.rows * 0.4));

						smilemodel->predict(smileFace, smilepredicted, smileConf);
						smileDetected = smilepredicted <= 0 ? false : true;

						if (debugmode) {
							string smileText = (smileDetected ? String("HAPPY :D") : String("SAD :("));
							putText(scene2D, smileText,
								Point(faces[i].x + 5, faces[i].y + 60), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
						}
					}

					int agesDetected;;
					if (agesclassifier) {
						double ageConf = 0.0;
						int agepredicted = 0;

						agesmodel->predict(preprocessedface, agepredicted, ageConf);
						agesDetected = agepredicted;
                        
						if (debugmode) {
                            string ageText;
                            if(agesDetected == 0)
                                ageText = String("Children");
                            if(agesDetected == 1)
                                ageText = String("Adult");
                            if(agesDetected == 2)
                                ageText = String("Elder");
							putText(scene2D, ageText,
								Point(faces[i].x + 5, faces[i].y + 75), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
						}
					}
					

					//Creates and saves face object
					facedetectedobj.faceRGB = faceImgRGB.clone();
					facedetectedobj.facePC = facexyz.clone();
					facedetectedobj.boundingbox = faces[i];
					facedetectedobj.confidence = confidence;
					facedetectedobj.gender = genderClass;
                    facedetectedobj.ages = agesDetected;
					facedetectedobj.id = textName;
					facedetectedobj.smile = smileDetected;
					facedetectedobj.pos3D = Point3f(face3Dcenter[0], face3Dcenter[1], face3Dcenter[2]);
					facesdetected.push_back(facedetectedobj);


				}
				if (debugmode) imshow("Rostro_" + to_string(i), faceImgRGB);
			}

			if (debugmode) imshow("Face detected", scene2D);
		}
		catch (...) {
			cout << "Face recognizer exception." << endl;
		}
	}
	else {
		cout << "Face recognizer status: Inactive." << endl;
	}

	return facesdetected;
}


// For rgb image only 
std::vector<faceobj> facerecog::facialRecognition(Mat scene2D)
{
	std::vector<faceobj> facesdetected;
	if (facesDB.size() != 0) facerecognitionactive = true;
	else facerecognitionactive = false;

	if (facedetectionactive) {

		try{
			
			int count = 0;
			Mat frame_gray;

			cvtColor(scene2D, frame_gray, CV_BGR2GRAY);
			Mat grayTemp = frame_gray.clone();
			equalizeHist(frame_gray, frame_gray); // Ecualiza la escena para 'mejorar' la deteccion de rostros

			std::vector<Rect> faces; //Vector donde se almacenaran los bounding box de cada rostro detectado

			faces = faceDetector(frame_gray, true);
			
			
			//Obtenemos la cara promedio
			Mat meanFace = model->getMean();

			//Obtenemos los eigenvectores
			/*Mat eigenvectors = model->get<Mat>("eigenvectors");
			Mat eigenvalues = model->get<Mat>("eigenvalues");*/
			Mat eigenvectors = model->getEigenVectors();
			Mat eigenvalues = model->getEigenValues();

			for (int i = 0; i < faces.size(); i++)
			{
				count = 0; //Inicializa el contador de elementos. Se necesitan al menos 3

				Mat faceImg = grayTemp(faces[i]).clone();
				Mat faceImgOriginal = faceImg.clone();
				Mat faceImgRGB = scene2D(faces[i]).clone();
				equalizeHist(faceImg, faceImg); //Ecualizo la cara original

				//Redimensiona la imagen del rostro a un tamaño mas conveniente
				resize(faceImg, faceImg, maxFaceSize);
				resize(faceImgOriginal, faceImgOriginal, maxFaceSize);
				resize(faceImgRGB, faceImgRGB, maxFaceSize);

				//Deteccion de ojos
				std::vector<Rect> eyesDetected = eyesDetector(faceImg);
				for (int e = 0; e < eyesDetected.size(); e++) {
					cv::rectangle(faceImgRGB, eyesDetected[e], CV_RGB(0, 0, 255), 1, 8, 0);
					count++;
				}

				// Deteccion de boca
				std::vector<Rect> mouthDetected = mouthDetector(faceImg);
				for (int m = 0; m < mouthDetected.size(); m++) {
					cv::rectangle(faceImgRGB, mouthDetected[m], CV_RGB(0, 0, 255), 1, 8, 0);
					count++;
				}

				// Deteccion de nariz
				std::vector<Point> noseDetected = noseDetector(faceImg);
				for (int n = 0; n < noseDetected.size(); n++) {
					circle(faceImgRGB, noseDetected[n], 5, CV_RGB(0, 0, 255), CV_FILLED, 8, 0);
					count++;
				}


				// Muestra un recuadro indicando la posicion del rostro detectado en el frame original
				if (count >= minNumFeatures) { //filtra por numero de caracteristicas detectadas

					//Encierra en un recuadro el rostro detectado
					if (debugmode)cv::rectangle(scene2D, faces[i], CV_RGB(0, 255, 0), 4, 8, 0);

					// Realiza un preprocesamiento del rostro detectado
					Mat preprocessedface;
					
					
					
					preprocessedface = preprocessFace(faceImgOriginal, eyesDetected);
					
					if (debugmode) imshow("Proc", preprocessedface);
					
					double confidence = 0.0;
					String textName = unknownName;
					if (facerecognitionactive) {

						//Intenta identificar el rostro
						Mat reconstructedFace;
						reconstructedFace = reconstructFace(preprocessedface, eigenvectors, meanFace);
						
						double imgError = 1.0;
						int clase = -1;
						for (int x = 0; x < facesDB.size(); x++) {
							for (int y = 0; y < facesDB[x].size(); y++) { //Comparamos con las imagenes de entrenamiento
								double error = getError(reconstructedFace, facesDB[x][y]);
								if (error < imgError) {
									imgError = error;
									clase = labelsDB[x][y]; // Para cada rostro, la clase es la misma :P 
								}

							}
						}

						confidence = 1.0 - imgError;

						textName = unknownName;
						String textConf = "Conf: " + to_string(confidence);

						if (imgError <= maxErrorThreshold) { // maxErrorThreshold es el error maximo aceptado para conciderar la prediccion correcta
							textName = trainingIDs[clase];
						}
						// Muestra el nombre y la prediccion en pantalla

						if (debugmode) {
							putText(scene2D, textName,
								Point(faces[i].x + 5, faces[i].y + 15), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
							putText(scene2D, textConf,
								Point(faces[i].x + 5, faces[i].y + 30), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
						}

					}
					
					
					// Clasificador de genero
					faceobj facedetectedobj;
					// Asegurarse de que la clase 0 es para hombres y la clase 1 para mujeres
					faceobj::Gender genderClass = faceobj::unknown;
					if (genderclassifier) {
						double genderConf = 0.0;
						int genderpredicted = 0;
						gendermodel->predict(preprocessedface, genderpredicted, genderConf);
						genderClass = genderpredicted <= 0 ? faceobj::male : faceobj::female;
					
						if (debugmode) {
							string genderText = "Gender: " + (genderClass == faceobj::male ? String("Male") : String("Female"));
							putText(scene2D, genderText,
								Point(faces[i].x + 5, faces[i].y + 45), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
						}
					}
					
					// Deteccion de sonrisa
					// Clasificador de sonrisas
					// Clase 0: Serio, Clase 1: Sonrisa
					bool smileDetected = false;
					if (smileclassifier) {
						double smileConf = 0.0;
						int smilepredicted = 0;
						Mat smileFace = preprocessedface(Rect(0, preprocessedface.rows * 0.6, preprocessedface.cols, preprocessedface.rows * 0.4));

						smilemodel->predict(smileFace, smilepredicted, smileConf);
						smileDetected = smilepredicted <= 0 ? false : true;

						if (debugmode) {
							string smileText = (smileDetected ? String("HAPPY :D") : String("SAD :("));
							putText(scene2D, smileText,
								Point(faces[i].x + 5, faces[i].y + 60), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
						}
					}
					
					int agesDetected;;
					if (agesclassifier) {
						double ageConf = 0.0;
						int agepredicted = 0;

						agesmodel->predict(preprocessedface, agepredicted, ageConf);
						agesDetected = agepredicted;

						if (debugmode) {
                            string ageText;
                            if(agesDetected == 0)
                                ageText = String("Children");
                            if(agesDetected == 1)
                                ageText = String("Adult");
                            if(agesDetected == 2)
                                ageText = String("Elder");
							putText(scene2D, ageText,
								Point(faces[i].x + 5, faces[i].y + 75), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
						}
					}

					//Creates and saves face object
					facedetectedobj.faceRGB = faceImgRGB.clone();
					facedetectedobj.boundingbox = faces[i];
					facedetectedobj.confidence = confidence;
					facedetectedobj.gender = genderClass;
					facedetectedobj.id = textName;
					facedetectedobj.smile = smileDetected;
					facedetectedobj.ages = agesDetected;
					facesdetected.push_back(facedetectedobj);


				}
				if (debugmode) imshow("Rostro_" + to_string(i), faceImgRGB);
			}

			if (debugmode) imshow("Face detected", scene2D);
		}
		catch (...) {
			cout << "Face recognizer exception." << endl;
		}
	}
	else {
		cout << "Face recognizer status: Inactive." << endl;
	}

	return facesdetected;
}



bool facerecog::faceTrainer(Mat scene2D, Mat scene3D, string id)
{
	bool result = false;
	try{

		//Face detection
		int count = 0;
		Mat frame_gray;

		if (scaleScene) {
			resize(scene2D, scene2D, Size(scene2D.cols * 2, scene2D.rows * 2));
			resize(scene3D, scene3D, Size(scene3D.cols * 2, scene3D.rows * 2));
		}
		
		cvtColor(scene2D, frame_gray, CV_BGR2GRAY);
		Mat grayTemp = frame_gray.clone();
		equalizeHist(frame_gray, frame_gray); // Ecualiza la escena para 'mejorar' la deteccion de rostros

		std::vector<Rect> faces; //Vector donde se almacenaran los bounding box de cada rostro detectado

		faces = faceDetector(frame_gray, false);

		for (int i = 0; i < faces.size(); i++)
		{
			count = 0; //Inicializa el contador de elementos. Se necesitan al menos 3

			Mat faceImg = grayTemp(faces[i]).clone();
			Mat faceImgOriginal = faceImg.clone();
			Mat faceImgRGB = scene2D(faces[i]).clone();
			equalizeHist(faceImg, faceImg); //Ecualizo la cara original

			//Redimensiona la imagen del rostro a un tamaño mas conveniente
			resize(faceImg, faceImg, maxFaceSize);
			resize(faceImgOriginal, faceImgOriginal, maxFaceSize);
			resize(faceImgRGB, faceImgRGB, maxFaceSize);

			//Deteccion de ojos
			std::vector<Rect> eyesDetected = eyesDetector(faceImg);
			for (int e = 0; e < eyesDetected.size(); e++) {
				cv::rectangle(faceImgRGB, eyesDetected[e], CV_RGB(0, 0, 255), 1, 8, 0);
				count++;
			}

			// Deteccion de boca
			std::vector<Rect> mouthDetected = mouthDetector(faceImg);
			for (int m = 0; m < mouthDetected.size(); m++) {
				cv::rectangle(faceImgRGB, mouthDetected[m], CV_RGB(0, 0, 255), 1, 8, 0);
				count++;
			}

			// Deteccion de nariz
			std::vector<Point> noseDetected = noseDetector(faceImg);
			for (int n = 0; n < noseDetected.size(); n++) {
				circle(faceImgRGB, noseDetected[n], 5, CV_RGB(0, 0, 255), CV_FILLED, 8, 0);
				count++;
			}


			// Muestra un recuadro indicando la posicion del rostro detectado en el frame original
			if (count >= minNumFeatures) { //filtra por numero de caracteristicas detectadas

				//Encierra en un recuadro el rostro detectado
				if (debugmode) cv::rectangle(scene2D, faces[i], CV_RGB(0, 255, 0), 4, 8, 0);

				// Realiza un preprocesamiento del rostro detectado
				Mat preprocessedface;
					
				Rect roi3D;
				// Relacion 100*120. Cambiar si se cambia el tamaño de la imagen a entrenar
				roi3D.x = cvRound(faces[i].x + (faces[i].width * 0.1));
				roi3D.y = cvRound(faces[i].y + (faces[i].height * 0.02));
				roi3D.width = cvRound(faces[i].width * 0.8);
				roi3D.height = cvRound(faces[i].height * 0.96);

				Mat facexyz = scene3D(roi3D).clone();
				
				if(use3D4recognition) {
					if(debugmode) imshow("2D", scene2D(roi3D).clone());
					if(debugmode) imshow("3D", facexyz);
					preprocessedface = preprocess3DFace(facexyz);
				}
				else {
					preprocessedface = preprocessFace(faceImgOriginal, eyesDetected);
				}
				
				
				if (debugmode) imshow("Proc", preprocessedface);




				// Entrenamos el rostro detectado ya preprocesado
				int classidx = trainingIDs.size();
				// search for id
				for (int x = 0; x < trainingIDs.size(); x++) {
					if (trainingIDs[x] == id) {
						classidx = x;
						break;
					}
				}

				if (classidx >= trainingIDs.size()) { //Nueva persona a entrenar
					//Agregamos la nueva clase
					trainingIDs.push_back(id);
					trainingCounts.push_back(1);

					std::vector<Mat> images;
					std::vector<int> labels;
					images.push_back(preprocessedface);
					labels.push_back(classidx);

					facesDB.push_back(images);
					labelsDB.push_back(labels);


				}
				else {
					if (facesDB[classidx].size() >= maxFacesVectorSize) { //Permitimos un numero maximo de imagenes
						facesDB[classidx].pop_back();
						labelsDB[classidx].pop_back();
					}
					facesDB[classidx].push_back(preprocessedface);
					labelsDB[classidx].push_back(classidx);

				}

				//Entrenamos el reconocedor
				std::vector<Mat> vectorImages2Train;
				std::vector<int> vectorLabels2Train;

				// Concatenamos los vectores para entrenamiento
				for (int x = 0; x < facesDB.size(); x++) {
					vectorImages2Train.insert(vectorImages2Train.end(), facesDB[x].begin(), facesDB[x].end());
					vectorLabels2Train.insert(vectorLabels2Train.end(), labelsDB[x].begin(), labelsDB[x].end());
				}

				model->train(vectorImages2Train, vectorLabels2Train);

				//Guardamos los archivos necesarios
				imwrite(trainingDataPath + id + to_string(facesDB[classidx].size() - 1) + ".jpg", facesDB[classidx][facesDB[classidx].size() - 1]);
				cout << "Image trained. Class: " << classidx << " Images trained: " << facesDB[classidx].size() << endl;

				model->save(trainingData);
				FileStorage trainingfile(trainingName, cv::FileStorage::WRITE);
				if (trainingfile.isOpened()) {
					trainingCounts.clear();
					for (int z = 0; z < facesDB.size(); z++) {
						trainingCounts.push_back(facesDB[z].size());
					}

					//Guardamos los nombres y los entrenamientos
					trainingfile << "trainingIDs" << trainingIDs;
					trainingfile << "trainingCounts" << trainingCounts;
					trainingfile.release();

				}
				
				result = true; // Entrenamiento exitoso
				if (debugmode) imshow("Image trained", scene2D);

			}

		}
		
	}
	catch (...) {
		cout << "Face trainer exception." << endl;
		result = false;
	}

	return result;
}

bool facerecog::saveConfigFile(string filename)
{
	bool result = false;
	FileStorage configFile(filename, cv::FileStorage::WRITE);
	if (configFile.isOpened()) {
		// Saves all face recognizer's parameters
		configFile << "face_cascade_name" << face_cascade_name; //Face cascade Name
		configFile << "profileface_cascade_name" << profileface_cascade_name; //Face cascade Name
		configFile << "eyes_cascade_name1" << eyes_cascade_name1; //Left eye cascade name
		configFile << "eyes_cascade_name2" << eyes_cascade_name2; //Right eye cascade name
		configFile << "mouth_cascade_name" << mouth_cascade_name; //Mouth cascade name
		configFile << "nose_cascade_name" << nose_cascade_name; //Nose cascade name
		configFile << "trainingName" << trainingName; //Training configuration file
		configFile << "trainingDataPath" << trainingDataPath; //Training data path where trained faces were saved
		configFile << "trainingData" << trainingData; //Faces trained
		configFile << "genderTrainingData" << genderTrainingData; //Gender training used for gender classification
		configFile << "agesTrainingData" << agesTrainingData; //Ages training used for ages classification
		configFile << "smileTrainingData" << smileTrainingData; //Smile training used for gender classification
		configFile << "maxErrorThreshold" << maxErrorThreshold; //recognizer max error
		configFile << "minNumFeatures" << minNumFeatures; //We have 4 features: left eye, right eye, nose and mouth
		configFile << "scaleScene" << scaleScene; //When true, scales scene by 2x to try enhance detection process
		configFile << "debugmode" << debugmode; 
		configFile << "minFacesVectorSize" << minFacesVectorSize; //Min faces for each person trained
		configFile << "maxFacesVectorSize" << maxFacesVectorSize; // Max faces for each person trained
		configFile << "use3D4recognition" << use3D4recognition;
		configFile << "resultsPath" << resultsPath;
		configFile << "usedlib" << usedlib;
		configFile << "useprofilerecognition" << useprofilerecognition;

		configFile.release();
		result = true;
	}

	return result;
}

bool facerecog::loadConfigFile(string filename)
{
	bool result = false;
	FileStorage configFile(filename, cv::FileStorage::READ);
	if (configFile.isOpened()){
		// Loads all face recognizer's parameters
		configFile["face_cascade_name"] >> face_cascade_name; //Face cascade Name
		configFile["profileface_cascade_name"] >> profileface_cascade_name; //Face cascade Name
		configFile["eyes_cascade_name1"] >> eyes_cascade_name1; //Left eye cascade name
		configFile["eyes_cascade_name2"] >> eyes_cascade_name2; //Right eye cascade name
		configFile["mouth_cascade_name"] >> mouth_cascade_name; //Mouth cascade name
		configFile["nose_cascade_name"] >> nose_cascade_name; //Nose cascade name
		configFile["trainingName"] >> trainingName; //Training configuration file
		configFile["trainingDataPath"] >> trainingDataPath; //Training data path where trained faces were saved
		configFile["trainingData"] >> trainingData; //Faces trained
		configFile["genderTrainingData"] >> genderTrainingData; //Gender training used for gender classification
		configFile["agesTrainingData"] >> agesTrainingData; //Gender training used for gender classification
		configFile["smileTrainingData"] >> smileTrainingData; //Gender training used for gender classification
		configFile["maxErrorThreshold"] >> maxErrorThreshold; //recognizer max error
		configFile["minNumFeatures"] >> minNumFeatures; //We have 4 features: left eye, right eye, nose and mouth
		configFile["scaleScene"] >> scaleScene; //When true, scales scene by 2x to try enhance detection process
		configFile["debugmode"] >> debugmode;
		configFile["minFacesVectorSize"] >> minFacesVectorSize; //Min faces for each person trained
		configFile["maxFacesVectorSize"] >> maxFacesVectorSize; // Max faces for each person trained
		configFile["use3D4recognition"] >> use3D4recognition;
		configFile["resultsPath"] >> resultsPath;
		configFile["usedlib"] >> usedlib;
		configFile["useprofilerecognition"] >> useprofilerecognition;
		
		configFile.release();
		result = true;
	}
	return result;
}

bool facerecog::loadTrainedData() {
	bool result = true;

	try {
		// Clear
		trainingIDs.clear();
		trainingCounts.clear();
		facesDB.clear();
		labelsDB.clear();
		std::vector<Mat> images;
		std::vector<int> labels;


		/* Leemos el archivo de entrenamiento */
		FileStorage file;
		file.open(trainingName, cv::FileStorage::READ);
		if (file.isOpened()){
			std::vector<cv::String> trainingIDsCV;
			file["trainingIDs"] >> trainingIDsCV;
			file["trainingCounts"] >> trainingCounts;
			file.release();
			for(int i = 0; i < trainingIDsCV.size(); i++)
				trainingIDs.push_back(trainingIDsCV[i].c_str());
		}

		if (trainingIDs.size() != 0) {
			// Cargamos los datos de entrenamiento. 
			for (int x = 0; x < trainingIDs.size(); x++) { // Para cada persona entrenada
				images.clear();
				labels.clear();
				for (int y = 0; y < trainingCounts[x]; y++) { // Para cada entrenamiento de cada persona
					string path = trainingDataPath + trainingIDs[x] + to_string(y) + ".jpg";
					images.push_back(imread(path, 0));
					labels.push_back(x);
				}
				facesDB.push_back(images);
				labelsDB.push_back(labels);
			}
		}
		else {
			cout << "No training data found!!. Face recognizer inactive." << endl;
			result = false;
		}

		//Entrenamos el reconocedor
		file.open(trainingData, cv::FileStorage::READ);
		if (file.isOpened()) { // Si ya se cuenta con un entrenamiento previo
			file.release();
			model->read(trainingData);
			cout << "Eigenfaces loaded." << endl;
		}
		else {
			std::vector<Mat> vectorImages2Train;
			std::vector<int> vectorLabels2Train;

			// Concatenamos los vectores para entrenamiento
			for (int x = 0; x < facesDB.size(); x++) {
				vectorImages2Train.insert(vectorImages2Train.end(), facesDB[x].begin(), facesDB[x].end());
				vectorLabels2Train.insert(vectorLabels2Train.end(), labelsDB[x].begin(), labelsDB[x].end());
			}

			if (vectorImages2Train.size() != 0) {
				//Entrenamos el modelo
				model->train(vectorImages2Train, vectorLabels2Train);
				cout << "Images trained." << endl;
			}
		}

		//Cargamos el clasificador de genero
		file.open(genderTrainingData, cv::FileStorage::READ);
		if (file.isOpened()) { // Si ya se cuenta con un entrenamiento previo
            FileNode node = file["opencv_fisherfaces"];
			gendermodel->read(node);
			genderclassifier = true;
			cout << "Gender classifier loaded." << endl;
			file.release();
		}
		
        //Cargamos el clasificador de edades
		file.open(agesTrainingData, cv::FileStorage::READ);
		if (file.isOpened()) { // Si ya se cuenta con un entrenamiento previo
            FileNode node = file["opencv_fisherfaces"];
			agesmodel->read(node);
			agesclassifier = true;
			cout << "Ages classifier loaded." << endl;
			file.release();
		}
		
		//Cargamos el clasificador de sonrisas
		file.open(smileTrainingData, cv::FileStorage::READ);
		if (file.isOpened()) { // Si ya se cuenta con un entrenamiento previo
			smilemodel->read(smileTrainingData);
			smileclassifier = true;
			cout << "Smile classifier loaded." << endl;
			file.release();
		}
		
	}
	catch (...) {
		cout << "Exception while loading trained data." << endl;
		result = false;
	}

	return result;
}

bool facerecog::clearFaceDB()
{
	try{
		trainingIDs.clear();
		trainingCounts.clear();
		facesDB.clear();
		labelsDB.clear();

		FileStorage trainingfile(trainingName, cv::FileStorage::WRITE);
		if (trainingfile.isOpened()) {
			//Guardamos los nombres y los entrenamientos
			trainingfile << "trainingIDs" << trainingIDs;
			trainingfile << "trainingCounts" << trainingCounts;
			trainingfile.release();
		}

		cout << "Data base cleared!!." << endl;
	}
	catch (...) {
		cout << "ClearFaceDB exception." << endl;
		return false;
	}
	return true;
}

bool facerecog::clearFaceDB(string id)
{
	bool result = false;
	try {
		
		// Buscamos el ID solicitado
		int classidx = trainingIDs.size();
		// search for id
		for (int x = 0; x < trainingIDs.size(); x++) {
			if (trainingIDs[x] == id) {
				classidx = x;
				break;
			}
		}

		if (classidx < (int)trainingIDs.size()) { //EL ID esta entrenado
			//Eliminamos el entrenamiento
			trainingIDs.erase(trainingIDs.begin() + classidx);
			trainingCounts.erase(trainingCounts.begin() + classidx);

			facesDB.erase(facesDB.begin() + classidx);
			labelsDB.erase(labelsDB.begin() + classidx);
			
			//ReEntrenamos el reconocedor
			std::vector<Mat> vectorImages2Train;
			std::vector<int> vectorLabels2Train;

			// Concatenamos los vectores para entrenamiento
			for (int x = 0; x < facesDB.size(); x++) {
				vectorImages2Train.insert(vectorImages2Train.end(), facesDB[x].begin(), facesDB[x].end());
				vectorLabels2Train.insert(vectorLabels2Train.end(), labelsDB[x].begin(), labelsDB[x].end());
			}

			
			if(vectorImages2Train.size() > 0) {
				model->train(vectorImages2Train, vectorLabels2Train);
				model->save(trainingData);
			}
			
			FileStorage trainingfile(trainingName, cv::FileStorage::WRITE);
			if (trainingfile.isOpened()) {
				//Guardamos los nombres y los entrenamientos
				trainingfile << "trainingIDs" << trainingIDs;
				trainingfile << "trainingCounts" << trainingCounts;
				trainingfile.release();

			}

		}
				
		result = true; // Eliminado exitoso
		
	}
	catch(...) {
		cout << "ClearFaceDBByID exception." << endl;
		result = false;
	}
	
	return result;
}

bool facerecog::initClassifiers()
{
	string errormessage = "Error loading the training file : ";

	try {
		if (!face_cascade.load(face_cascade_name))
		{
			cout << errormessage + face_cascade_name << endl;
			return false;
		};
		
		if (!profileface_cascade.load(profileface_cascade_name))
		{
			cout << errormessage + profileface_cascade_name << endl;
			return false;
		};

		if (!eye_cascade1.load(eyes_cascade_name1))
		{
			cout << errormessage + eyes_cascade_name1 << endl;
			return false;
		};

		if (!eye_cascade2.load(eyes_cascade_name2))
		{
			cout << errormessage + eyes_cascade_name2 << endl;
			return false;
		};

		if (!mouth_cascade.load(mouth_cascade_name))
		{
			cout << errormessage + mouth_cascade_name << endl;
			return false;
		};

		if (!nose_cascade.load(nose_cascade_name))
		{
			cout << errormessage + nose_cascade_name << endl;
			return false;
		};
		
	}
	catch (...) {
		cout << "Exception while initializing classifiers." << endl;
		return false;
	}

	return true;
}


std::vector<Rect> facerecog::profileFaceDetector(Mat sceneImage, bool findAllFaces)
{
	std::vector<Rect> faces; //Vector donde se almacenaran los bounding box de cada rostro detectado
	double scaleFactor = 1.1; //Indica el factor de escala a utilizar para las ventanas de busqueda
	int minNeighbors = 10; //Determina cuantas ventanas positivas (votos) minimo deben coincidir para considerar un rostro detectado
	cv::Size minFeatureSize(20, 34); // El tamaño minimo en pixeles de la venta de búsqueda. Determina el tamaño minimo de rostros a encontrar
	int flags = CASCADE_FIND_BIGGEST_OBJECT | CASCADE_DO_ROUGH_SEARCH; //0 para buscar todos los rostros, CASCADE_FIND_BIGGEST_OBJECT | CASCADE_DO_ROUGH_SEARCH para buscar solo el más grande

	if (findAllFaces) flags = 0;

	try {
		std::vector<Rect> facesProfileLeft;
		profileface_cascade.detectMultiScale(sceneImage, facesProfileLeft, scaleFactor, minNeighbors, flags, minFeatureSize);
		
		Mat dst;
		flip(sceneImage, dst, 1);
		std::vector<Rect> facesProfileRight;
		profileface_cascade.detectMultiScale(dst, facesProfileRight, scaleFactor, minNeighbors, flags, minFeatureSize);
		
		
		for (int x = 0; x < facesProfileRight.size(); x++) {
			facesProfileRight[x].x = sceneImage.cols - (facesProfileRight[x].x + facesProfileRight[x].width);
		}
		
		
		faces.insert( faces.end(), facesProfileLeft.begin(), facesProfileLeft.end() );
		
		faces.insert( faces.end(), facesProfileRight.begin(), facesProfileRight.end() );
		
		
	}
	catch (...) {
		cout << "Face detector exception. Can't detect faces." << endl;
	}

	return faces;
}




std::vector<Rect> facerecog::faceDetector(Mat sceneImage, bool findAllFaces)
{
	std::vector<Rect> faces; //Vector donde se almacenaran los bounding box de cada rostro detectado
	std::vector<Rect> finalfaces; //Vector donde se almacenaran los bounding box de cada rostro detectado
	double scaleFactor = 1.1; //Indica el factor de escala a utilizar para las ventanas de busqueda
	int minNeighbors = 5; //Determina cuantas ventanas positivas (votos) minimo deben coincidir para considerar un rostro detectado
	cv::Size minFeatureSize(25, 25); // El tamaño minimo en pixeles de la venta de búsqueda. Determina el tamaño minimo de rostros a encontrar
	//cv::Size minFeatureSize(20, 34); // El tamaño minimo en pixeles de la venta de búsqueda. Determina el tamaño minimo de rostros a encontrar
	int flags = CASCADE_FIND_BIGGEST_OBJECT | CASCADE_DO_ROUGH_SEARCH; //0 para buscar todos los rostros, CASCADE_FIND_BIGGEST_OBJECT | CASCADE_DO_ROUGH_SEARCH para buscar solo el más grande

	if (findAllFaces) flags = 0;

	try {
		
		if(usedlib) {
			faces = faceDetectorV2(sceneImage, true);
		} else {
			face_cascade.detectMultiScale(sceneImage, faces, scaleFactor, minNeighbors, flags, minFeatureSize);
		}
		
		if(useprofilerecognition) {
			// Profile Face Detector
			std::vector<Rect> pFaces = profileFaceDetector(sceneImage, true);
			faces.insert( faces.end(), pFaces.begin(), pFaces.end() );
			
			// Magic Code (drop overlaped bounding boxes)
			cv::Mat mask = cv::Mat::zeros(sceneImage.size(), CV_8UC1); 
			cv::Size scaleFactor(10,10); 
			for (int i = 0; i < faces.size(); i++)
			{
				cv::Rect box = faces.at(i) + scaleFactor;
				cv::rectangle(mask, box, cv::Scalar(255), CV_FILLED); 
			}
			std::vector<std::vector<cv::Point> > contours;
			cv::findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
			for (int j = 0; j < contours.size(); j++)
			{
				finalfaces.push_back(cv::boundingRect(contours.at(j)));
			}
		
		} else {
			finalfaces = faces;
		}
	}
	catch (...) {
		cout << "Face detector exception. Can't detect faces." << endl;
	}

	return finalfaces;
}



std::vector<Rect> facerecog::faceDetectorV2(Mat sceneImage, bool findAllFaces)
{
	std::vector<Rect> faces; //Vector donde se almacenaran los bounding box de cada rostro detectado
	
	//if (findAllFaces) flags = 0;

	try {
		
		frontal_face_detector detector = get_frontal_face_detector();
        // Gray
		array2d<unsigned char> dlibImage;
		
		// Convert from Mat to dlibImage
		assign_image(dlibImage, dlib::cv_image<unsigned char>(sceneImage));
		
		
		pyramid_up(dlibImage);
		
		std::vector<dlib::rectangle> dets = detector(dlibImage);
		
		if(debugmode) cout << "Number of faces detected: " << dets.size() << endl;
		
		// Print rects of detected faces
		for(int i = 0; i < dets.size(); i++)
		{
			// convert
			Rect r = Rect(cv::Point2i(dets[i].left() * 0.5, dets[i].top() * 0.5), cv::Point2i(dets[i].right() * 0.5 + 1, dets[i].bottom() * 0.5 + 1));;
			faces.push_back(r);
		}
		

		
	}
	catch (...) {
		cout << "Face detector exception. Can't detect faces." << endl;
	}

	return faces;
}






std::vector<Rect> facerecog::eyesDetector(Mat faceImage)
{
	std::vector<Rect> eyesVector;
	try {
		/* Eyes search area */
		double EYE_X = 0.10;
		double EYE_Y = 0.20;
		double EYE_W = 0.40;
		double EYE_H = 0.35;

		int leftX = cvRound(faceImage.cols * EYE_X);
		int topY = cvRound(faceImage.rows * EYE_Y);
		int widthX = cvRound(faceImage.cols * EYE_W);
		int heightY = cvRound(faceImage.rows * EYE_H);
		int rightX = cvRound(faceImage.cols * (1.0 - EYE_X - EYE_W));

		Rect leftEyeRect(leftX, topY, widthX, heightY);
		Rect rightEyeRect(rightX, topY, widthX, heightY);

		//Recortamos el area de los ojos
		Mat lefteye = faceImage(leftEyeRect);
		Mat righteye = faceImage(rightEyeRect);

		//Detectamos los ojos
		std::vector<Rect> leftEyeDetected;
		eye_cascade2.detectMultiScale(lefteye, leftEyeDetected, 1.1, 3, CASCADE_FIND_BIGGEST_OBJECT);

		std::vector<Rect> rightEyeDetected;
		eye_cascade1.detectMultiScale(righteye, rightEyeDetected, 1.1, 3, CASCADE_FIND_BIGGEST_OBJECT);

		if (leftEyeDetected.size() != 0)
			eyesVector.push_back(Rect(leftEyeDetected[0].x + leftX, leftEyeDetected[0].y + topY, leftEyeDetected[0].width, leftEyeDetected[0].height));
		if (rightEyeDetected.size() != 0)
			eyesVector.push_back(Rect(rightEyeDetected[0].x + rightX, rightEyeDetected[0].y + topY, rightEyeDetected[0].width, rightEyeDetected[0].height));
	}
	catch (...) {
		cout << "Eyes detector exception. Can't search eyes." << endl;
	}

	return eyesVector;
}

std::vector<Rect> facerecog::mouthDetector(Mat faceImage)
{
	std::vector<Rect> mouthVector;

	try{
		/* mouth search area */
		double MOUTH_X = 0.20;
		double MOUTH_Y = 0.65;
		double MOUTH_W = 0.60;
		double MOUTH_H = 0.35;

		int mouthX = cvRound(faceImage.cols * MOUTH_X);
		int mouthY = cvRound(faceImage.rows * MOUTH_Y);
		int mouthH = cvRound(faceImage.rows * MOUTH_H);
		int mouthW = cvRound(faceImage.cols * MOUTH_W);

		Rect mouthRect(mouthX, mouthY, mouthW, mouthH);

		// Recortamos el area de la boca
		Mat mouth = faceImage(mouthRect);

		//Detectamos la boca
		std::vector<Rect> mouthDetected;
		mouth_cascade.detectMultiScale(mouth, mouthDetected, 1.1, 3, CASCADE_FIND_BIGGEST_OBJECT);

		if (mouthDetected.size() != 0)
			mouthVector.push_back(Rect(mouthDetected[0].x + mouthX, mouthDetected[0].y + mouthY, mouthDetected[0].width, mouthDetected[0].height));
	}
	catch (...) {
		cout << "Mouth detector exception. Can't search mouth." << endl;
	}

	return mouthVector;
}

std::vector<Point> facerecog::noseDetector(Mat faceImage)
{
	std::vector<Point> noseVector;

	try{
		/* nose search area */
		double NOSE_X = 0.25;
		double NOSE_Y = 0.20;
		double NOSE_W = 0.50;
		double NOSE_H = 0.60;

		int noseX = cvRound(faceImage.cols * NOSE_X);
		int noseY = cvRound(faceImage.rows * NOSE_Y);
		int noseH = cvRound(faceImage.rows * NOSE_H);
		int noseW = cvRound(faceImage.cols * NOSE_W);

		Rect noseRect(noseX, noseY, noseW, noseH);

		// Deteccion de nariz
		Mat nose = faceImage(noseRect);
		std::vector<Rect> noseDetected;
		nose_cascade.detectMultiScale(nose, noseDetected, 1.1, 3, CASCADE_FIND_BIGGEST_OBJECT);

		if (noseDetected.size() != 0)
			noseVector.push_back(Point(noseDetected[0].x + (noseDetected[0].width / 2) + noseX, noseDetected[0].y + (noseDetected[0].height / 2) + noseY));
	}
	catch (...) {
		cout << "Nose detector exception. Can't search nose." << endl;
	}
	return noseVector;
}

Mat facerecog::reconstructFace(Mat preprocessedFace, Mat eigenvectors, Mat meanImage)
{
	Mat reconstructedFace;

	try {
		//Proyectamos al subespacio rostro. Esto nos devuelve los eigenvalores 
		Mat projection = LDA::subspaceProject(eigenvectors, meanImage, preprocessedFace.reshape(1, 1));

		//Reconstruimos una imagen a partir de los eigenvalores anteriores, multiplicandolos por los eignefaces del entrenamiento
		// y sumando la imagen promedio
		Mat reconstructionRow = LDA::subspaceReconstruct(eigenvectors, meanImage, projection);

		//Convertimos el vector resultante a un matriz de 100 * 120. Las imagenes se manejan como vectores columna
		Mat reconstructionMat = reconstructionRow.reshape(1, faceTrinedSize.height);
		// Convertimos de punto flotante a enteros de 8 bits
		reconstructedFace = Mat(reconstructionMat.size(), CV_8U);
		reconstructionMat.convertTo(reconstructedFace, CV_8U, 1, 0);
	}
	catch (...) {
		reconstructedFace = preprocessedFace.clone();
		cout << "Face reconstruction exception. Can't reconstruct the face." << endl;
	}

	return reconstructedFace;
}

/* Use only when process RGB images */
Mat facerecog::preprocessFace(Mat faceImg, std::vector<Rect> eyesVector, Size imgDesiredSize)
{
	Mat transfromedFace;
	resize(faceImg, transfromedFace, imgDesiredSize);

	if (eyesVector.size() >= 2) { //Se detecto ambos ojos
		//Primer elemento del vector es el ojo izquierdo

		//Centro del ojo
		Point leftEyeCenter(eyesVector[0].x + eyesVector[0].width / 2, eyesVector[0].y + eyesVector[0].height / 2);
		Point rightEyeCenter(eyesVector[1].x + eyesVector[1].width / 2, eyesVector[1].y + eyesVector[1].height / 2);

		/* Obtención de matriz de transformacion y aplicacion*/

		Point eyesCenter((leftEyeCenter.x + rightEyeCenter.x) * 0.5, (leftEyeCenter.y + rightEyeCenter.y) * 0.5);

		double dy = (rightEyeCenter.y - leftEyeCenter.y);
		double dx = (rightEyeCenter.x - leftEyeCenter.x);
		double len = sqrt(dx*dx + dy*dy);
		// Obtenemos el angulo entre los ojos
		double angle = atan2(dy, dx) * 180.0 / CV_PI;
		//Definimos la nueva posicion de los ojos
		double DESIRED_LEFT_EYE_X = 0.20;
		double DESIRED_RIGHT_EYE_X = 1.0 - DESIRED_LEFT_EYE_X;
		double DESIRED_LEFT_EYE_Y = 0.26;
		//Determinamos el factor de escala. Las imagenes son normalizadas a un tamaño de 100x120 pixeles
		int DESIRED_FACE_WIDTH = imgDesiredSize.width;
		int DESIRED_FACE_HEIGHT = imgDesiredSize.height;
		double desiredLen = (DESIRED_RIGHT_EYE_X - DESIRED_LEFT_EYE_X);
		double scale = desiredLen * DESIRED_FACE_WIDTH / len;
		//Obtenemos la matriz de transformacion
		Mat rot_mat = getRotationMatrix2D(eyesCenter, angle, scale);

		double ex = DESIRED_FACE_WIDTH * 0.5f - eyesCenter.x;
		double ey = DESIRED_FACE_HEIGHT * DESIRED_LEFT_EYE_Y - eyesCenter.y;
		rot_mat.at<double>(0, 2) += ex;
		rot_mat.at<double>(1, 2) += ey;
		//Aplicamos la transformación
		Mat faceImgWarped = Mat(DESIRED_FACE_HEIGHT, DESIRED_FACE_WIDTH, CV_8U, Scalar(128));
		warpAffine(faceImg, faceImgWarped, rot_mat, faceImgWarped.size());

		transfromedFace = faceImgWarped.clone();



		/** Ecualizamos **/
		int w = faceImgWarped.cols;
		int h = faceImgWarped.rows;
		Mat wholeFace;
		equalizeHist(faceImgWarped, wholeFace);
		int midX = w / 2;
		Mat leftSide = faceImgWarped(Rect(0, 0, midX, h)).clone();
		Mat rightSide = faceImgWarped(Rect(midX, 0, w - midX, h)).clone();
		equalizeHist(leftSide, leftSide);
		equalizeHist(rightSide, rightSide);


		for (int y = 0; y<h; y++) {
			for (int x = 0; x<w; x++) {
				int v;
				if (x < w / 4) {
					// Left 25%: just use the left face.
					v = leftSide.at<uchar>(y, x);
				}
				else if (x < w * 2 / 4) {
					// Mid-left 25%: blend the left face & whole face.
					int lv = leftSide.at<uchar>(y, x);
					int wv = wholeFace.at<uchar>(y, x);
					// Blend more of the whole face as it moves
					// further right along the face.
					float f = (x - w * 1 / 4) / (float)(w / 4);
					v = cvRound((1.0f - f) * lv + (f)* wv);
				}
				else if (x < w * 3 / 4) {
					// Mid-right 25%: blend right face & whole face.
					int rv = rightSide.at<uchar>(y, x - midX);
					int wv = wholeFace.at<uchar>(y, x);
					// Blend more of the right-side face as it moves
					// further right along the face.
					float f = (x - w * 2 / 4) / (float)(w / 4);
					v = cvRound((1.0f - f) * wv + (f)* rv);
				}
				else {
					// Right 25%: just use the right face.
					v = rightSide.at<uchar>(y, x - midX);
				}
				transfromedFace.at<uchar>(y, x) = v;
			}// end x loop
		}//end y loop



	}


	return transfromedFace;
}


// ONLY for 3D images
Mat facerecog::preprocess3DFace(Mat faceImg3D, Size imgDesiredSize)
{
	Mat finalImg;
	
	try {
		Mat rangeMat = Mat::zeros(faceImg3D.rows, faceImg3D.cols, CV_32FC1);
	
		double minDepth = 12.0; //Maxima profundidad en metros
		double maxDepth = 0.0; //Minima profundidad en metros
		int xmin = 0, ymin = 0;
		int xmax = 0, ymax = 0;
		
		//Obtenemos la distancia del sensor al rostro
		for(int y = 0; y < faceImg3D.rows; y++){
			for(int x = 0; x < faceImg3D.cols; x++){
				double depVal = 0.0;
				if(faceImg3D.data != NULL) {
					double xval = (double) faceImg3D.at<cv::Vec3f>(y,x)[0];
					double yval = (double) faceImg3D.at<cv::Vec3f>(y,x)[1];
					double zval = (double) faceImg3D.at<cv::Vec3f>(y,x)[2];
					
					if(isnan(xval)) xval = 0.0;
					if(isnan(yval)) yval = 0.0;
					if(isnan(zval)) zval = 0.0;
					
					depVal = sqrt((xval * xval) + (yval * yval) + (zval * zval));
					
					//depVal = (double) faceImg3D.at<cv::Vec3f>(y,x)[2];
					
				}
				rangeMat.at<float>(y,x) = depVal;

				//minDepth = (depVal < minDepth) && (depVal > 0) ? depVal : minDepth;
				
				if((depVal < minDepth) && (depVal > 0.0)) {
					minDepth = depVal;
					xmin = x;
					ymin = y;
				}
				
				//maxDepth = depVal > maxDepth ? depVal : maxDepth;
				if(depVal > maxDepth) {
					maxDepth = depVal;
					xmax = x;
					ymax = y;
				}
				
			}
		}

		
		//cout << "MinDepth: " << minDepth << " MaxDepth: " << maxDepth << endl;
		//cout << "xmin: " << xmin << " ymin: " << ymin << endl;
		//cout << "xmin,ymin: " << faceImg3D.at<cv::Vec3f>(ymin,xmin)[2] << endl;
		//cout << "xmax: " << xmax << " ymax: " << ymax << endl;
		//cout << "xmax,ymax: " << faceImg3D.at<cv::Vec3f>(ymax,xmax)[2] << endl;
		
		if(debugmode) imshow("Range Map 3D", rangeMat);
		
		//Convertimos a escala de grises
		Mat rangeImg(faceImg3D.rows, faceImg3D.cols, CV_8UC1, Scalar::all(255));
		double MAXFACEDEPTH = 0.5; //metros
		for(int a = 0; a < rangeMat.rows; a++) {
			for(int b = 0; b < rangeMat.cols; b++) {
				double depVal = rangeMat.at<float>(a,b) - minDepth;
				
				if(depVal >= 0.0) { //Solo datos validos
					if(depVal > MAXFACEDEPTH) { //Si tenemos puntos que no pertenecen al rostro
						rangeImg.at<uchar>(a,b) = 0;
					}
					else {
						int distCalc = 255 - (int)((255.0 * depVal) / MAXFACEDEPTH);
						//cout <<"Dist calc: " << distCalc << endl;
						rangeImg.at<uchar>(a,b) = distCalc;
					}

				} else rangeImg.at<uchar>(a,b) = 0;
			}
		}
		if(debugmode) imshow("Range", rangeImg);
		resize(rangeImg,finalImg, imgDesiredSize);
		if(debugmode) imshow("Range FINAL", finalImg);
		
	} catch(...) {
		cout << "Exception while preprocessing 3D face." << endl;
	}
	/// END TESTING
	
	return finalImg;
}



double facerecog::getError(const Mat A, const Mat B)
{
	double errornorm = 0.0;

	try {
		double errorL2 = norm(A, B, CV_L2);
		//Normaliza 
		errornorm = errorL2 / (double)(A.rows * A.cols);
	}
	catch (...) {
		errornorm = 1.0;
		cout << "Computing error exception." << endl;
	}

	return errornorm;
}

/* Debug only */
void facerecog::tile(const std::vector<Mat> &src, Mat &dst, int grid_x, int grid_y)
{
	try {
		// patch size
		int width = dst.cols / grid_x;
		int height = dst.rows / grid_y;
		// iterate through grid
		int k = 0;
		for (int i = 0; i < grid_y; i++) {
			for (int j = 0; j < grid_x; j++) {
				if (k >= src.size()) return;
				Mat s = src[k++];
				resize(s, s, Size(width, height));
				s.copyTo(dst(Rect(j*width, i*height, width, height)));
			}
		}
	}
	catch (...) {
		// NOTHING
	}
}

Mat facerecog::rotate(Mat src, double angle)
{
	Mat dst;
	try {
		Point2f center(src.cols / 2., src.rows / 2.);
		Mat rot = getRotationMatrix2D(center, angle, 1.0);
		warpAffine(src, dst, rot, src.size());
	}
	catch (...) {
		cout << "Rotate function exception. " << endl;
	}

	return dst;
}

string facerecog::expand_user(string path) {
  if (!path.empty() && path[0] == '~') {
    assert(path.size() == 1 || path[1] == '/');  // or other error handling
    char const* home = getenv("HOME");
    if (home || ((home = getenv("USERPROFILE")))) {
      path.replace(0, 1, home);
    }
    else {
      char const *hdrive = getenv("HOMEDRIVE"), *hpath = getenv("HOMEPATH");
      assert(hdrive);  // or other error handling
      assert(hpath);
      path.replace(0, 1, std::string(hdrive) + hpath);
    }
  }
  return path;
}


std::vector<Rect> facerecog::wavingDetection()
{
	std::vector<Rect> wavingDetected;
	
	bool debug = 0;
	int maxFrames = 30;
	int framecount = 0;
	int maxpercent = 80;
	int camid = 0;
	waveframe_width = 1920;
	waveframe_height = 1080;
	string filename = expand_user("~/JUSTINA/catkin_ws/src/vision/face_recog/facerecog_config/waveConfig.xml");;
	
	//Load config file
	try
	{
		FileStorage configFile(filename, cv::FileStorage::READ);
		if (configFile.isOpened()){
			configFile["debug"] >> debug; 
			configFile["maxFrames"] >> maxFrames; 
			configFile["maxpercent"] >> maxpercent; 
			configFile["camid"] >> camid; 
			configFile["frame_width"] >> waveframe_width; 
			configFile["frame_height"] >> waveframe_height;
			
			configFile.release();
			
		}
		
		
	} catch(...) 
	{
		cout << "Exception loading cofig file for waving. Default config loaded D:" << endl;
		debug = 0;
		maxFrames = 30;
		maxpercent = 80;
		camid = 0;
		waveframe_width = 1920;
		waveframe_height = 1080;
	}
	
	
	
	
	
	try
    {
        // Webcam
        VideoCapture cap;
        
		
		// open the default camera, use something different from 0 otherwise;
		if(!cap.open(camid))
		{
			cout << "Can't open the webcam." << endl;
			return wavingDetected;
		}

		//cap.set(CV_CAP_PROP_FRAME_WIDTH,1280);
		//cap.set(CV_CAP_PROP_FRAME_HEIGHT,720);
		
		// Set full HD resolution
		cap.set(CV_CAP_PROP_FRAME_WIDTH,waveframe_width);
		cap.set(CV_CAP_PROP_FRAME_HEIGHT,waveframe_height);
		
		
        frontal_face_detector detector = get_frontal_face_detector();
        
        // Backgound extractor
        std::vector<cv::Ptr<cv::BackgroundSubtractor> >  bg;
 
        std::vector<Rect> handArea;
        
        std::vector<int> wavecount;
        std::vector<Rect> faces;
        
        while (framecount <= maxFrames)
        {
			
			Mat frame;
			cap >> frame;
			
			// Real frame size
			waveframe_width = frame.cols;
			waveframe_height = frame.rows;
			
			if(framecount == 0) {
				
				
				array2d<rgb_pixel> dlibImage;
				
				assign_image(dlibImage, dlib::cv_image<bgr_pixel>(frame));
				
				
				pyramid_up(dlibImage);
				
				std::vector<dlib::rectangle> dets = detector(dlibImage);
				
				if(debug) cout << "Number of faces detected: " << dets.size() << endl;
				
				handArea.clear();
				wavecount.clear();
				bg.clear();
				faces.clear();
				wavingDetected.clear();
				
				for(int i = 0; i < dets.size(); i++)
				{
					// convert from dlibRect to OpenCV RECT
					Rect r = Rect(cv::Point2i(dets[i].left() * 0.5, dets[i].top() * 0.5), cv::Point2i(dets[i].right() * 0.5 + 1, dets[i].bottom() * 0.5 + 1));
					faces.push_back(r);
					
					//Creates two boundiing box of interest
					Rect roiL = Rect(r.x - (r.width * 2), r.y - (r.height * 2) - (r.height * 0.30), r.width * 2, r.height * 2);
					roiL.x = roiL.x < 0 ? 0 : roiL.x;
					roiL.x = roiL.x >= frame.cols ? frame.cols - 1 : roiL.x;
					roiL.y = roiL.y < 0 ? 0 : roiL.y;
					roiL.y = roiL.y >= frame.rows ? frame.rows - 1 : roiL.y;
					roiL.width = roiL.x + roiL.width >= frame.cols ? frame.cols - roiL.x - 1 : roiL.width;
					roiL.height = roiL.y + roiL.height >= frame.rows ? frame.rows - roiL.y - 1 : roiL.height;
					//roiL.height = roiL.y + roiL.height > r.y ? r.y - roiL.y : roiL.height;
					
					
					
					Rect roiR = Rect(r.x + r.width, r.y - (r.height * 2) - (r.height * 0.30), r.width * 2, r.height * 2);
					roiR.x = roiR.x < 0 ? 0 : roiR.x;
					roiR.x = roiR.x >= frame.cols ? frame.cols - 1 : roiR.x;
					roiR.y = roiR.y < 0 ? 0 : roiR.y;
					roiR.y = roiR.y >= frame.rows ? frame.rows - 1 : roiR.y;
					roiR.width = roiR.x + roiR.width >= frame.cols ? frame.cols - roiR.x - 1 : roiR.width;
					roiR.height = roiR.y + roiR.height >= frame.rows ? frame.rows - roiR.y - 1 : roiR.height;
					//roiR.height = roiR.y + roiR.height > r.y ? r.y - roiR.y : roiR.height;
					
					
					// Adds to te lists
					handArea.push_back(roiL);
					handArea.push_back(roiR);
					
					wavecount.push_back(0);
					wavecount.push_back(0);
					
					bg.push_back(cv::createBackgroundSubtractorMOG2(100, 100, false));
					bg.push_back(cv::createBackgroundSubtractorMOG2(100, 100, false));
					
					
				}
				
			} 
			else {

			
				for(int i = 0; i < handArea.size(); i++)
				{
					
					try{
						Mat frame_roi = frame(handArea[i]).clone();
						Mat fgimg, backgroundImage;
						
						bg[i]->apply(frame_roi, fgimg, 0.75);
						
						int dilation_size = 15;
						int erode_size = 3;
						Mat element1 = getStructuringElement( MORPH_RECT,
						   Size( 2*erode_size + 1, 2*erode_size+1 ),
						   Point( erode_size, erode_size ) );

						Mat element2 = getStructuringElement( MORPH_ELLIPSE,
						   Size( 2*dilation_size + 1, 2*dilation_size+1 ),
						   Point( dilation_size, dilation_size ) );


						cv::erode (fgimg, fgimg, element1);
						cv::dilate (fgimg, fgimg, element2);

						std::vector<std::vector<cv::Point> > contours;
        
						
						cv::findContours (fgimg, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
						
						if(debug) cv::imshow ("WaveWindow", fgimg);
						
						if(contours.size() > 0) wavecount[i] = wavecount[i] + 1;
						
						
					} catch(exception& e)
					{
						cout << "Wave detection exception!" << endl;
						cout << e.what() << endl;
					}					
					
				}
				
			}

			framecount++;
			
			if(debug) {
				for(int i = 0; i < handArea.size(); i++)
				{
					cv::rectangle(frame, handArea[i], CV_RGB(255,0,0), 2);
				}
				imshow("Video", frame);
			}
				
				
			waitKey(1);
			
		}
		
				
		for(int i = 0; i < wavecount.size(); i+=2)
		{
			double percent1 = wavecount[i] * 100 / maxFrames;
			double percent2 = wavecount[i+1] * 100 / maxFrames;
			
			if(percent1 > maxpercent || percent2 > maxpercent) 
			{
				cout << "WAVE DETECTED! :D  -   " <<  percent1 << " - "  << percent2 << endl;
				wavingDetected.push_back(faces[i * 0.5]);
			}
		}
        
    }
    catch (exception& e)
    {
        cout << "Wave detection exception!" << endl;
        cout << e.what() << endl;
    }
	
	if(debug) cv::destroyAllWindows();
	
	return wavingDetected;
}



