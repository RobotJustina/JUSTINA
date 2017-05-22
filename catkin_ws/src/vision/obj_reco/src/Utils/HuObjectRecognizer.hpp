#pragma once

#include <iostream>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "DetectedObject.hpp"

class HuObjectRecognizer
{
public:
		bool debugMode; 
		double maxErrorForRecognition; 
		std::string trainingDir;
		std::string trainFile; 

		HuObjectRecognizer(); 
		HuObjectRecognizer(std::string trainingDir); 
		~HuObjectRecognizer(){}; 

		// para guardar archivos/imagenes de entrenamiento. 
		bool SaveFileTrainingObject(std::string objName, DetectedObject detObj, cv::Mat& outShadowImage ); 
		bool SaveImageTrainingObject(std::string objName, DetectedObject detObj, cv::Mat& outShadowImage ); 		
		
		// Para Cargar entrenmientos en el proagrama
		bool LoadTrainingFromFile();
		bool LoadTrainingFromImagesNames(std::vector< std::string > objNames ); 
		
		// Para reconocer
		std::string RecognizeObject( DetectedObject detObj ); 
		std::string RecognizeShape(DetectedObject detObj, double &outRecoError, std::vector< std::pair< double, std::string> >& shapeErrorList); 
		
		std::vector< std::string > trainNames;		
		std::vector< float > trainHeights; 
		std::vector< std::vector< cv::Point2f > > trainShadowsCont2D; 

		std::string RecognizeSize(DetectedObject detObj, double& outRecoError, std::vector< std::pair< double, std::string> >& sizeErrorList ); 
};