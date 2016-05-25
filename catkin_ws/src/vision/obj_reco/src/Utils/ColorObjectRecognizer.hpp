#pragma once

#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "DetectedObject.hpp"

class ColorObjectRecognizer
{
public:
		bool debugMode; 
		
		std::string trainingFile; 
		std::string trainingDir; 

		double minErrorForRecognition; 

		ColorObjectRecognizer(); 
		ColorObjectRecognizer(std::string trainingDir, std::string trainingFile, int binNo); 
		~ColorObjectRecognizer(){}; 

		// para guardar archivos/imagenes de entrenamiento
		bool SaveFileTraining( std::string objName, cv::Mat bgrImage, cv::Mat objectMask ); 
		bool SaveImageTraining( std::string objName, cv::Mat bgrImageToTrain ); 
		
		// Para Cargar entrenmientos en el proagrama
		bool LoadTrainingFromFile();
		bool LoadTrainingFromImagesNames( std::vector< std::string > objNames ); 
		
		// Para reconocer
		std::string RecognizeObject( cv::Mat bgrImageToRecognize ); 
		std::string RecognizeObject( cv::Mat bgrImage, cv::Mat objectMask, double& outRecoError, std::vector< std::pair< double, std::string> >& errorSort ); 

		std::vector< std::string > trainingNames;		
		std::vector< cv::Mat > trainingHistograms; 

		cv::Mat CalculateHistogram( cv::Mat bgrImage, cv::Mat mask ); 

private: 	
		
		int binNo; 
		std::string trainingFilePath; 


};