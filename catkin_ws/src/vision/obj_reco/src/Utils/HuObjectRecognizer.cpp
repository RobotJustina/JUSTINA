#include "HuObjectRecognizer.hpp"

HuObjectRecognizer::HuObjectRecognizer()
{
	this->trainingDir = "HuObjReco"; 
	this->trainFile = "HuObjReco.yml"; 
	this->maxErrorForRecognition = 0.1; 
}

HuObjectRecognizer::HuObjectRecognizer(std::string trainingDir)
{
	this->trainingDir = trainingDir; 
	this->trainFile = "HuObjReco.yml"; 
	this->maxErrorForRecognition = 0.1; 
}

bool HuObjectRecognizer::SaveFileTrainingObject(std::string detObjName, DetectedObject detObj, cv::Mat& outShadowImage)
{
	try
	{
		int idx = 0; 
		cv::FileStorage fs; 	

		std::vector< std::string > names; 
		std::vector< double > heights; 
		std::vector< std::vector < cv::Point2f > > contours; 

		if( fs.open(  this->trainFile, fs.READ ) )
		{
			cv::FileNode contoursNode = fs[ "Features3D" ]; 
			cv::FileNodeIterator it = contoursNode.begin(); 
			cv::FileNodeIterator it_end = contoursNode.end(); 

			for( ; it != it_end ; ++it, idx++ )
			{
				std::string oName = (*it)["name"]; 
				double oHeight = (double)(*it)["height"]; 
				
				std::vector < cv::Point2f > oCont; 
				(*it)["contour2d"] >> oCont; 

				names.push_back( oName ); 
				heights.push_back( oHeight ); 
				contours.push_back( oCont ); 
			}
			fs.release(); 
		}
			
		names.push_back(  detObjName ); 
		heights.push_back( detObj.height ); 
		contours.push_back( detObj.shadowContour2D ); 

		if( fs.open( this->trainFile, fs.WRITE) )
		{
			fs << "Features3D" << "["; 
			for( int i=0; i< names.size(); i++)
			{
				fs << "{:"; 
				fs << "name" << names[i]; 
				fs << "height" << heights[i]; 
				fs << "contour2d" << contours[i]; 
				fs << "}"; 
				
			}
			fs << "]"; 
			fs.release(); 
			std::cout << "HuObjectRecognizer : Saved Trained object:" + detObjName  << std::endl; 
		}
		else
		{
			std::cout << "HuObjectRecognizer : Cant Save Training File (objName=" + detObjName + ")" << std::endl; 
			return false; 
		}
		
		return true; 
	}
	catch(std::exception ex){
		std::cout << "HuObjectRecognizer : Cant Save Training File (objName=" + detObjName + ")" << " ex:" << ex.what() <<std::endl; 
		return false; 
	}
}

/* To train the objects, we use the shadowContour which is aa aporximated
Poligone of the proyection of the PointCloud of the Object. To save this 
as an image (8 bits, one channel) we change meters to milimeters and translate
the points of the poligone in order to every index is positive integer :)*/
bool HuObjectRecognizer::SaveImageTrainingObject(std::string objName, DetectedObject detObj, cv::Mat& outShadowImage)
{
	std::vector< cv::Point2i > shadowInt; 

	cv::Point2i min = cv::Point2i(  10000000.0 ,  10000000.0 ); 
	cv::Point2i max = cv::Point2i( -10000000.0 , -10000000.0 ); 

	for(int i=0; i< detObj.shadowContour2D.size(); i++)
	{
		cv::Point2f polyPtFloat = detObj.shadowContour2D[i]; 
		cv::Point2i polyPtInt = polyPtFloat*1000; 
		
		if( polyPtInt.x > max.x )
			max.x = polyPtInt.x; 
		if( polyPtInt.x < min.x )
			min.x = polyPtInt.x; 

		if( polyPtInt.y > max.y )
			max.y = polyPtInt.y; 
		if( polyPtInt.y < min.y )
			min.y = polyPtInt.y; 

		shadowInt.push_back( polyPtInt ); 
	}
	 
	int cols = max.x - min.x +7; 
	int rows = max.y - min.y +7; 
	
	cv::Mat shadowContImage = cv::Mat::zeros(rows, cols, CV_8UC1); 
	//for(int i=0; i < shadowInt.size()-1; i++)
	//{ 
	//	cv::Point2i p1 = cv::Point2i(3, 3) + shadowInt[i] - min; 
	//	cv::Point2i p2 = cv::Point2i(3, 3) + shadowInt[i+1] - min; 
	//	cv::line( shadowContImage, p1, p2, cv::Scalar(255, 255, 255) ); 
	//}

	//// Last Point to First Point. 
	//cv::Point2f p1 = cv::Point2i(3, 3) + shadowInt[ shadowInt.size()-1] - min; 
	//cv::Point2f p2 = cv::Point2i(3, 3) + shadowInt[0] - min; 
	//cv::line( shadowContImage, p1, p2, cv::Scalar(255, 255, 255), 2 ); 
	
	//// USING DRAW FUNCITON 
	std::vector< cv::Point2i > contourOffseted; 
	cv::Mat drawContImage = cv::Mat::zeros(rows, cols, CV_8UC1); 
	for(int i=0; i < shadowInt.size(); i++)
	{
		contourOffseted.push_back( shadowInt[i] - min + cv::Point2i(3, 3) );
	}
	std::vector< std::vector< cv::Point2i > > contours; 
	contours.push_back( contourOffseted ); 
	cv::drawContours( drawContImage, contours, 0, cv::Scalar(255,255,255), -1); 
	//cv::imshow("drawContImage",drawContImage ); 

	std::string imageFilePath = objName + ".shape" + ".jpg"; 
	cv::imwrite( imageFilePath , drawContImage); 

	if( debugMode )
		cv::imshow( "Shadow " + objName, drawContImage );

	return true; 
}

bool HuObjectRecognizer::LoadTrainingFromFile()
{
	try
	{
		int idx = 0; 
		cv::FileStorage fs; 	

		if( fs.open(  this->trainFile, fs.READ ) )
		{
			cv::FileNode contoursNode = fs[ "Features3D" ]; 
			cv::FileNodeIterator it = contoursNode.begin(); 
			cv::FileNodeIterator it_end = contoursNode.end(); 

			for( ; it != it_end ; ++it, idx++ )
			{
				std::string oName = (*it)["name"]; 
				double oHeight = (double)(*it)["height"]; 				
				std::vector < cv::Point2f > oCont; 
				(*it)["contour2d"] >> oCont; 

				this->trainNames.push_back( oName ); 
				this->trainHeights.push_back( oHeight ); 
				this->trainShadowsCont2D.push_back( oCont ); 

				std::cout << "HuObjectRecognizer : Loaded Trained object:" + oName  << std::endl;
			}
			fs.release(); 
		}
		
		
		
		return true; 
	}
	catch(std::exception ex){
		std::cout << "HuObjectRecognizer : Cant Load Training File (" + this->trainFile + ")" << " ex:" << ex.what() <<std::endl; 
		return false; 
	}
}

bool HuObjectRecognizer::LoadTrainingFromImagesNames(std::vector< std::string > objNames )
{
	for( int i=0; i<objNames.size(); i++ )
	{
		std::string imageFilePath = this->trainingDir + "//" + objNames[i] + ".jpg"; 
		cv::Mat contourIma = cv::imread( imageFilePath, CV_LOAD_IMAGE_GRAYSCALE ); 
		
		if( !contourIma.data )
		{
			std::cout << "HuObjectRecognizer : Cant train objName=" + objNames[i] << std::endl; 
			continue; 
		}

		std::vector< cv::Point2i > nonZeroIdx; 
		cv::findNonZero( contourIma, nonZeroIdx ); 
		
		cv::vector< cv::Point2i > cHull; 
		cv::convexHull( nonZeroIdx, cHull ); 

		std::vector< cv::Point2i > shadowContour2D; 
		cv::approxPolyDP( cHull , shadowContour2D, 0.00001, true); 		

		// converting to Meters
		std::vector< cv::Point2f > contoursFloat; 
		for( int j=0; j<shadowContour2D.size();  j++ )
		{ 
			contoursFloat.push_back( cv::Point2f( (float)shadowContour2D[j].x/1000.0f, (float)shadowContour2D[j].y/1000.0f ) ); 
		}

		this->trainNames.push_back( objNames[i] ); 
		this->trainShadowsCont2D.push_back( contoursFloat );
	}

	return true; 
}

std::string HuObjectRecognizer::RecognizeShape(DetectedObject detObj, double &outRecoError, std::vector< std::pair< double, std::string> >& shapeErrorList)
{
	std::string bestObject = ""; 

	double minError = 99999999.9; 
	for( int i=0; i<trainNames.size(); i++)
	{
		double shapeError = cv::matchShapes( detObj.shadowContour2D, this->trainShadowsCont2D[i],CV_CONTOURS_MATCH_I1, 0.0 ); 
		
		//double heightError = 10*std::abs( detObj.height - trainHeights[i] ); 
		//double totalError = shapeError + heightError;

		double totalError = shapeError; ;

		shapeErrorList.push_back( std::pair< double, std::string>( shapeError, this->trainNames[i] ) ); 
		//sizeErrorList.push_back( std::pair< double, std::string>( heightError, this->trainNames[i] ) ); 
		//totalErrorList.push_back( std::pair< double, std::string>( totalError, this->trainNames[i] ) ); 

		if( totalError<minError )
		{
			minError = totalError; 
			bestObject = this->trainNames[i]; 
		}
	}

	//std::sort( shapeErrorList.begin(), shapeErrorList.end() ); 
	//std::sort( sizeErrorList.begin(), sizeErrorList.end() ); 
	//std::sort( totalErrorList.begin(), totalErrorList.end() ); 

	outRecoError = minError; 
	return bestObject; 
}

std::string HuObjectRecognizer::RecognizeSize(DetectedObject detObj, double& outRecoError, std::vector< std::pair< double, std::string> >& sizeErrorList )
{
	std::string bestObject = ""; 

	double minError = 99999999.9; 
	for( int i=0; i<trainNames.size(); i++)
	{
		double heightError = std::abs( detObj.height - trainHeights[i] ); 
		//double totalError = shapeError + heightError;

		double totalError = heightError;

		//shapeErrorList.push_back( std::pair< double, std::string>( shapeError, this->trainNames[i] ) ); 
		sizeErrorList.push_back( std::pair< double, std::string>( heightError, this->trainNames[i] ) ); 
		//totalErrorList.push_back( std::pair< double, std::string>( totalError, this->trainNames[i] ) ); 

		if( totalError<minError )
		{
			minError = totalError; 
			bestObject = this->trainNames[i]; 
		}
	}

	//std::sort( shapeErrorList.begin(), shapeErrorList.end() ); 
	//std::sort( sizeErrorList.begin(), sizeErrorList.end() ); 
	//std::sort( totalErrorList.begin(), totalErrorList.end() ); 

	outRecoError = minError; 
	return bestObject; 
}