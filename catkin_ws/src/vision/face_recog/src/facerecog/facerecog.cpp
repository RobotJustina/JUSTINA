#include "facerecog.h"

using namespace std;
using namespace cv;

facerecog::facerecog()
{
	try {

		// Loading default values for all variables
		setDefaultValues();
		
		
		//loading config file
		if (!loadConfigFile(configFileName)) {
			saveConfigFile(configFileName);
			cout << "Default config file created." << endl;
		}
		else {
			cout << "Config file loaded." << endl;
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
			model = createEigenFaceRecognizer(); //Eigen
			// model = createFisherFaceRecognizer(); //Fisher
			// model = createLBPHFaceRecognizer(); //Local Binary Patterns Histograms

			/**** Gender recognizer ****/
			gendermodel = createFisherFaceRecognizer(); //Fisher
			
			/**** Smile recognizer ****/
			smilemodel = createFisherFaceRecognizer(); //Fisher

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

	//basePath = "/home/j0z3ph/facerecog/";
	basePath = "";
	string basePathGender = "../JUSTINA/catkin_ws/src/vision/face_recog/facerecog/";
	configFileName = basePath + "config.xml";


	//String face_cascade_name = "/usr/share/opencv/lbpcascades/lbpcascade_frontalface.xml";
	face_cascade_name = "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml";

	eyes_cascade_name1 = "/usr/share/opencv/haarcascades/haarcascade_mcs_lefteye.xml";
	eyes_cascade_name2 = "/usr/share/opencv/haarcascades/haarcascade_mcs_righteye.xml";

	mouth_cascade_name = "/usr/share/opencv/haarcascades/haarcascade_mcs_mouth.xml";
	nose_cascade_name = "/usr/share/opencv/haarcascades/haarcascade_mcs_nose.xml";

	maxFaceSize = Size(200, 200);
	faceTrinedSize = Size(100, 120);

	minNumFeatures = 3; //Un ojo, nariz y boca; Dos ojos, boca; Dos ojos, nariz
	scaleScene = false;
	maxErrorThreshold = 0.25; // Maximo error permitido para reconocer 
	
	// Indica el valor maximo que puede tener cada vector de rostros entrenados
	maxFacesVectorSize = 50;

	// Indica el valor minimo de caras que se pueden tener como base de entrenamiento
	minFacesVectorSize = 10;

	unknownName = "unknown"; // nombre que se le dara a la persona desconocida

	trainingName = basePath + "recfac.xml";
	//trainingDataPath = basePath + "data";
	trainingDataPath = basePath;
	trainingData = basePath + "eigenfaces.xml";
	genderTrainingData = basePathGender + "efgender.xml";
	smileTrainingData = basePathGender + "efsmile.xml";
	

	genderclassifier = false; // Gender classifier flag
	smileclassifier = false;
}

vector<faceobj> facerecog::facialRecognitionForever(Mat scene2D, Mat scene3D, string faceID)
{
	vector<faceobj> facesdetected;
	try {
		if (scaleScene){	
			resize(scene2D, scene2D, Size(scene2D.cols * 2, scene2D.rows * 2));
			resize(scene3D, scene3D, Size(scene3D.cols * 2, scene3D.rows * 2));
		}
		
		Mat sceneRGB = scene2D.clone();
		Mat sceneXYZ = scene3D.clone();
		Mat sceneRGBID = scene2D.clone(); //For id identification
		Mat sceneRGBID2Save = scene2D.clone();
		
		facesdetected = facialRecognition(sceneRGB, sceneXYZ);
		double bestConfidence = 0.0;
		int bestConfidenceIdx = -1;
		
		if(faceID != "") { //If we have a face id
			for(int x = 0; x < (int)facesdetected.size(); x++) { //for each face detected
				if(faceID == facesdetected[x].id) { //If we found the face requested
					if(facesdetected[x].confidence >  bestConfidence) {
						bestConfidence = facesdetected[x].confidence;
						bestConfidenceIdx = x;
						sceneRGBID2Save = sceneRGBID.clone();
						
						//Bounding box
						rectangle(sceneRGBID2Save, facesdetected[x].boundingbox, CV_RGB(0, 255, 0), 4, 8, 0);
						//Name label
						putText(sceneRGBID2Save, faceID,
							Point(facesdetected[x].boundingbox.x + 5, facesdetected[x].boundingbox.y + 15), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
						//Confidence
						string textConf = "CONF: " + to_string(bestConfidence);
						putText(sceneRGBID2Save, textConf,
							Point(facesdetected[x].boundingbox.x + 5, facesdetected[x].boundingbox.y + 30), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
						//Gender label
						string genderText = "GENDER: " + (facesdetected[x].gender == faceobj::male ? String("MALE") : String("FEMALE"));
						putText(sceneRGBID2Save, genderText,
							Point(facesdetected[x].boundingbox.x + 5, facesdetected[x].boundingbox.y + 45), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
						//Mood label
						string smileText = (facesdetected[x].smile ? String("HAPPY") : String("SAD"));
						putText(sceneRGBID2Save, smileText,
							Point(facesdetected[x].boundingbox.x + 5, facesdetected[x].boundingbox.y + 60), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
									
					}
				}
			}
			
			scene2D = sceneRGBID2Save.clone();
			if(bestConfidence > 0.0) {
				faceobj theFace = facesdetected[bestConfidenceIdx];
				facesdetected.clear();
				facesdetected.push_back(theFace);
			}
			
		} 
		else { //If we want to detect all faces
			for(int x = 0; x < (int)facesdetected.size(); x++) { //for each face detected
				rectangle(scene2D, facesdetected[x].boundingbox, CV_RGB(255, 0, 0), 4, 8, 0);
				//Name label
				putText(scene2D, facesdetected[x].id,
					Point(facesdetected[x].boundingbox.x + 5, facesdetected[x].boundingbox.y + 15), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
				//Confidence
				string textConf = "CONF: " + to_string(facesdetected[x].confidence);
				putText(scene2D, textConf,
					Point(facesdetected[x].boundingbox.x + 5, facesdetected[x].boundingbox.y + 30), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
				//Gender label
				string genderText = "GENDER: " + (facesdetected[x].gender == faceobj::male ? String("MALE") : String("FEMALE"));
				putText(scene2D, genderText,
					Point(facesdetected[x].boundingbox.x + 5, facesdetected[x].boundingbox.y + 45), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
				//Mood label
				string smileText = (facesdetected[x].smile ? String("HAPPY") : String("SAD"));
				putText(scene2D, smileText,
					Point(facesdetected[x].boundingbox.x + 5, facesdetected[x].boundingbox.y + 60), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
			}
			
		}
		
		
		imshow("Face Recog", scene2D);
		
		
	} catch(...) {
		cout << "Face recognizer exception." << endl;
	}
	return facesdetected;
}


vector<faceobj> facerecog::facialRecognition(Mat scene2D, Mat scene3D, string faceID)
{
	vector<faceobj> facesdetected;
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
					rectangle(sceneRGBID2Save, facesdetected[x].boundingbox, CV_RGB(0, 255, 0), 4, 8, 0);
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
								
					imwrite(faceID + "_scene.jpg", sceneRGBID2Save);
				}
			}
			rectangle(scene2D, facesdetected[x].boundingbox, CV_RGB(255, 0, 0), 4, 8, 0);
			//Gender label
			string genderText = (facesdetected[x].gender == faceobj::male ? String("MALE") : String("FEMALE"));
			putText(scene2D, genderText,
				Point(facesdetected[x].boundingbox.x + 5, facesdetected[x].boundingbox.y + 15), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
			//Mood label
			string smileText = (facesdetected[x].smile ? String("HAPPY") : String("SAD"));
			putText(scene2D, smileText,
				Point(facesdetected[x].boundingbox.x + 5, facesdetected[x].boundingbox.y + 30), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 2, 8, false);
		}
		imwrite("all_scene.jpg", scene2D);
		
	} catch(...) {
		cout << "Face recognizer exception." << endl;
	}
	return facesdetected;
}

vector<faceobj> facerecog::facialRecognition(Mat scene2D, Mat scene3D)
{
	vector<faceobj> facesdetected;
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
			Mat meanFace = model->get<Mat>("mean");

			//Obtenemos los eigenvectores
			Mat eigenvectors = model->get<Mat>("eigenvectors");
			Mat eigenvalues = model->get<Mat>("eigenvalues");

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
				vector<Rect> eyesDetected = eyesDetector(faceImg);
				for (int e = 0; e < eyesDetected.size(); e++) {
					rectangle(faceImgRGB, eyesDetected[e], CV_RGB(0, 0, 255), 1, 8, 0);
					count++;
				}

				// Deteccion de boca
				vector<Rect> mouthDetected = mouthDetector(faceImg);
				for (int m = 0; m < mouthDetected.size(); m++) {
					rectangle(faceImgRGB, mouthDetected[m], CV_RGB(0, 0, 255), 1, 8, 0);
					count++;
				}

				// Deteccion de nariz
				vector<Point> noseDetected = noseDetector(faceImg);
				for (int n = 0; n < noseDetected.size(); n++) {
					circle(faceImgRGB, noseDetected[n], 5, CV_RGB(0, 0, 255), CV_FILLED, 8, 0);
					count++;
				}


				// Muestra un recuadro indicando la posicion del rostro detectado en el frame original
				if (count >= minNumFeatures) { //filtra por numero de caracteristicas detectadas

					//Encierra en un recuadro el rostro detectado
					if (debugmode)rectangle(scene2D, faces[i], CV_RGB(0, 255, 0), 4, 8, 0);

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
					

					//Creates and saves face object
					facedetectedobj.faceRGB = faceImgRGB.clone();
					facedetectedobj.facePC = facexyz.clone();
					facedetectedobj.boundingbox = faces[i];
					facedetectedobj.confidence = confidence;
					facedetectedobj.gender = genderClass;
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
			vector<Rect> eyesDetected = eyesDetector(faceImg);
			for (int e = 0; e < eyesDetected.size(); e++) {
				rectangle(faceImgRGB, eyesDetected[e], CV_RGB(0, 0, 255), 1, 8, 0);
				count++;
			}

			// Deteccion de boca
			vector<Rect> mouthDetected = mouthDetector(faceImg);
			for (int m = 0; m < mouthDetected.size(); m++) {
				rectangle(faceImgRGB, mouthDetected[m], CV_RGB(0, 0, 255), 1, 8, 0);
				count++;
			}

			// Deteccion de nariz
			vector<Point> noseDetected = noseDetector(faceImg);
			for (int n = 0; n < noseDetected.size(); n++) {
				circle(faceImgRGB, noseDetected[n], 5, CV_RGB(0, 0, 255), CV_FILLED, 8, 0);
				count++;
			}


			// Muestra un recuadro indicando la posicion del rostro detectado en el frame original
			if (count >= minNumFeatures) { //filtra por numero de caracteristicas detectadas

				//Encierra en un recuadro el rostro detectado
				if (debugmode) rectangle(scene2D, faces[i], CV_RGB(0, 255, 0), 4, 8, 0);

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

					vector<Mat> images;
					vector<int> labels;
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
				vector<Mat> vectorImages2Train;
				vector<int> vectorLabels2Train;

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
		configFile << "eyes_cascade_name1" << eyes_cascade_name1; //Left eye cascade name
		configFile << "eyes_cascade_name2" << eyes_cascade_name2; //Right eye cascade name
		configFile << "mouth_cascade_name" << mouth_cascade_name; //Mouth cascade name
		configFile << "nose_cascade_name" << nose_cascade_name; //Nose cascade name
		configFile << "trainingName" << trainingName; //Training configuration file
		configFile << "trainingDataPath" << trainingDataPath; //Training data path where trained faces were saved
		configFile << "trainingData" << trainingData; //Faces trained
		configFile << "genderTrainingData" << genderTrainingData; //Gender training used for gender classification
		configFile << "smileTrainingData" << smileTrainingData; //Smile training used for gender classification
		configFile << "maxErrorThreshold" << maxErrorThreshold; //recognizer max error
		configFile << "minNumFeatures" << minNumFeatures; //We have 4 features: left eye, right eye, nose and mouth
		configFile << "scaleScene" << scaleScene; //When true, scales scene by 2x to try enhance detection process
		configFile << "debugmode" << debugmode; 
		configFile << "minFacesVectorSize" << minFacesVectorSize; //Min faces for each person trained
		configFile << "maxFacesVectorSize" << maxFacesVectorSize; // Max faces for each person trained
		configFile << "use3D4recognition" << use3D4recognition;
		

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
		configFile["eyes_cascade_name1"] >> eyes_cascade_name1; //Left eye cascade name
		configFile["eyes_cascade_name2"] >> eyes_cascade_name2; //Right eye cascade name
		configFile["mouth_cascade_name"] >> mouth_cascade_name; //Mouth cascade name
		configFile["nose_cascade_name"] >> nose_cascade_name; //Nose cascade name
		configFile["trainingName"] >> trainingName; //Training configuration file
		configFile["trainingDataPath"] >> trainingDataPath; //Training data path where trained faces were saved
		configFile["trainingData"] >> trainingData; //Faces trained
		configFile["genderTrainingData"] >> genderTrainingData; //Gender training used for gender classification
		configFile["smileTrainingData"] >> smileTrainingData; //Gender training used for gender classification
		configFile["maxErrorThreshold"] >> maxErrorThreshold; //recognizer max error
		configFile["minNumFeatures"] >> minNumFeatures; //We have 4 features: left eye, right eye, nose and mouth
		configFile["scaleScene"] >> scaleScene; //When true, scales scene by 2x to try enhance detection process
		configFile["debugmode"] >> debugmode;
		configFile["minFacesVectorSize"] >> minFacesVectorSize; //Min faces for each person trained
		configFile["maxFacesVectorSize"] >> maxFacesVectorSize; // Max faces for each person trained
		configFile["use3D4recognition"] >> use3D4recognition;
		
		
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
		vector<Mat> images;
		vector<int> labels;


		/* Leemos el archivo de entrenamiento */
		FileStorage file;
		file.open(trainingName, cv::FileStorage::READ);
		if (file.isOpened()){
			file["trainingIDs"] >> trainingIDs;
			file["trainingCounts"] >> trainingCounts;
			file.release();
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
			model->load(trainingData);
			cout << "Eigenfaces loaded." << endl;
		}
		else {
			vector<Mat> vectorImages2Train;
			vector<int> vectorLabels2Train;

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
			file.release();
			gendermodel->load(genderTrainingData);
			genderclassifier = true;
			cout << "Gender classifier loaded." << endl;
		}
		
		//Cargamos el clasificador de sonrisas
		file.open(smileTrainingData, cv::FileStorage::READ);
		if (file.isOpened()) { // Si ya se cuenta con un entrenamiento previo
			file.release();
			smilemodel->load(smileTrainingData);
			smileclassifier = true;
			cout << "Smile classifier loaded." << endl;
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
			vector<Mat> vectorImages2Train;
			vector<int> vectorLabels2Train;

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

vector<Rect> facerecog::faceDetector(Mat sceneImage, bool findAllFaces)
{
	std::vector<Rect> faces; //Vector donde se almacenaran los bounding box de cada rostro detectado
	double scaleFactor = 1.1; //Indica el factor de escala a utilizar para las ventanas de busqueda
	int minNeighbors = 5; //Determina cuantas ventanas positivas (votos) minimo deben coincidir para considerar un rostro detectado
	cv::Size minFeatureSize(25, 25); // El tamaño minimo en pixeles de la venta de búsqueda. Determina el tamaño minimo de rostros a encontrar
	int flags = CASCADE_FIND_BIGGEST_OBJECT | CASCADE_DO_ROUGH_SEARCH; //0 para buscar todos los rostros, CASCADE_FIND_BIGGEST_OBJECT | CASCADE_DO_ROUGH_SEARCH para buscar solo el más grande

	if (findAllFaces) flags = 0;

	try {
		face_cascade.detectMultiScale(sceneImage, faces, scaleFactor, minNeighbors, flags, minFeatureSize);
	}
	catch (...) {
		cout << "Face detector exception. Can't detect faces." << endl;
	}

	return faces;
}

vector<Rect> facerecog::eyesDetector(Mat faceImage)
{
	vector<Rect> eyesVector;
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

vector<Rect> facerecog::mouthDetector(Mat faceImage)
{
	vector<Rect> mouthVector;

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

vector<Point> facerecog::noseDetector(Mat faceImage)
{
	vector<Point> noseVector;

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
		Mat projection = subspaceProject(eigenvectors, meanImage, preprocessedFace.reshape(1, 1));

		//Reconstruimos una imagen a partir de los eigenvalores anteriores, multiplicandolos por los eignefaces del entrenamiento
		// y sumando la imagen promedio
		Mat reconstructionRow = subspaceReconstruct(eigenvectors, meanImage, projection);

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
Mat facerecog::preprocessFace(Mat faceImg, vector<Rect> eyesVector, Size imgDesiredSize)
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
		double MAXFACEDEPTH = 0.2; //metros
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
void facerecog::tile(const vector<Mat> &src, Mat &dst, int grid_x, int grid_y)
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


