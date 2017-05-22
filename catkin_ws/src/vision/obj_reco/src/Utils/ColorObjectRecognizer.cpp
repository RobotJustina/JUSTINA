#include "ColorObjectRecognizer.hpp"

ColorObjectRecognizer::ColorObjectRecognizer()
{
	this->trainingDir = "ColorObjReco"; 
	this->trainingFile = "ColorObjReco.xml"; 
	this->trainingFilePath = trainingDir + "/" + trainingFile; 

	this->binNo = 16; 
	this->minErrorForRecognition = 0.98; // [1,0] 
}

ColorObjectRecognizer::ColorObjectRecognizer(std::string trainingDir, std::string trainingFile, int binNo)
{
	this->trainingDir = trainingDir; 
	this->trainingFile = trainingFile + ".xml"; 
	this->trainingFilePath = trainingDir + "/" + trainingFile; 

	this->binNo = binNo; 
	this->minErrorForRecognition = 0.98; // [1,0] great is better
}

bool ColorObjectRecognizer::SaveFileTraining( std::string objName, cv::Mat bgrImage, cv::Mat objectMask )
{
	cv::Mat histogram = CalculateHistogram( bgrImage, objectMask ); 

	std::vector< std::string > names; 
	std::vector< cv::Mat > histos;
	
	names.push_back( objName ); 
	histos.push_back( histogram ); 

	cv::FileStorage fs; 
	try
	{
		std::cout << "ColorObjectRecognizer : Trying to write " + this->trainingFilePath << std::endl;  
		if( fs.open(  this->trainingFilePath, cv::FileStorage::READ) )
		{
			cv::FileNode histNodes = fs[ "ColorHistograms" ]; 
			cv::FileNodeIterator it = histNodes.begin(); 
			cv::FileNodeIterator it_end = histNodes.end(); 

			int idx = 0;

			for( ; it != it_end ; ++it, idx++ )
			{
				std::string oName = (*it)["name"]; 
				
				cv::Mat oHist; 
				(*it)["histogram"] >> oHist; 

				names.push_back( oName ); 
				histos.push_back( oHist ); 
			}
			fs.release();
		}
		
		if( fs.open( this->trainingFilePath, cv::FileStorage::WRITE) )
		{
			fs << "ColorHistograms" << "["; 
			for( int i=0; i< names.size(); i++)
			{
				fs << "{:"; 
				fs << "name" << names[i]; 
				fs << "histogram" << histos[i]; 
				fs << "}"; 
			}
			fs << "]"; 
			fs.release(); 
			std::cout << "ColorObjectRecognizer : Saved Trained object:" + objName  << std::endl; 
			return true; 
		}
		else
		{
			std::cout << "ColorObjectRecognizer : Can't Save Training File (objName=" + objName + ")" << std::endl; 
			return false; 
		}
	}
	catch(std::exception ex){
		fs.release(); 
		std::cout << "ColorObjectRecognizer : Can't Save Training File (objName=" + objName + ")" << " ex:" << ex.what() <<std::endl; 
		return false; 
	}
}

bool ColorObjectRecognizer::LoadTrainingFromFile()
{
	cv::FileStorage fs;
	try
	{	
		std::cout << "ColorObjectRecognizer : Trying to load " + this->trainingFilePath << std::endl;  
		if( fs.open(  this->trainingFilePath, cv::FileStorage::READ) )
		{
			cv::FileNode histNodes = fs[ "ColorHistograms" ]; 
			cv::FileNodeIterator it = histNodes.begin(); 
			cv::FileNodeIterator it_end = histNodes.end(); 

			this->trainingHistograms.clear(); 
			this->trainingNames.clear(); 

			for( ; it != it_end ; ++it )
			{
				std::string oName = (*it)["name"]; 
				
				cv::Mat oHist; 
				(*it)["histogram"] >> oHist; 

				this->trainingNames.push_back( oName ); 
				this->trainingHistograms.push_back( oHist );
				
				std::cout << "ColorObjectRecognizer : Loaded Trained Object:" + oName  << std::endl;
			}
			fs.release();
		}
		else
		{
			std::cout << "ColorObjectRecognizer :  Cant Load Training File (" + this->trainingFilePath + ")"<<std::endl; 
			return false; 
		}
	}
	catch(std::exception ex){
		fs.release(); 
		std::cout << "ColorObjectRecognizer : Can't Load Training File (" <<  this->trainingFilePath << ")" << " ex:" << ex.what() <<std::endl; 
		return false; 
	}
}

//std::string ColorObjectRecognizer::RecognizeObject( cv::Mat bgrImage, cv::Mat objectMask, double& outRecoError, std::vector< std::pair< double, std::string> >& errorSort )
//{
//	cv::Mat objHisto = CalculateHistogram( bgrImage, objectMask ); 
//	
//	double bestError = 10000.0; 
//	std::string bestObject = ""; 
//
//	for( int i = 0 ; i<this->trainingHistograms.size() ; i++)
//	{
//		double correlationError = cv::compareHist( objHisto, this->trainingHistograms[i], CV_COMP_BHATTACHARYYA);
//		
//		errorSort.push_back( std::pair< double, std::string>( correlationError, this->trainingNames[i] ) ); 
//
//		if( correlationError < bestError && correlationError < this->minErrorForRecognition )
//		{
//			bestError = correlationError; 
//			bestObject = this->trainingNames[i]; 
//		}
//	}
//	
//	//std::sort( errorSort.begin(), errorSort.end() ); 
//
//	outRecoError = bestError; 
//	return bestObject; 
//}

std::string ColorObjectRecognizer::RecognizeObject( cv::Mat bgrImage, cv::Mat objectMask, double& outRecoError, std::vector< std::pair< double, std::string> >& errorSort )
{

	cv::Mat objHisto = CalculateHistogram( bgrImage, objectMask ); 
	
	double bestError = -10000.0; 
	std::string bestObject = ""; 

	for( int i = 0 ; i<this->trainingHistograms.size() ; i++)
	{
		//double correlationError = cv::compareHist( objHisto, this->trainingHistograms[i], CV_COMP_CORREL);
		double correlationError = cv::compareHist( objHisto, this->trainingHistograms[i], CV_COMP_INTERSECT);
		
		errorSort.push_back( std::pair< double, std::string>( correlationError, this->trainingNames[i] ) ); 

		if( correlationError > bestError )
		{
			bestError = correlationError; 
			bestObject = this->trainingNames[i]; 
		}
	}
	
	//std::sort( errorSort.begin(), errorSort.end(), [](	std::pair<double,std::string> a, std::pair<double,std::string> b){ return b.first < a.first; } ); 

	outRecoError = bestError; 
	return bestObject; 
}

cv::Mat ColorObjectRecognizer::CalculateHistogram( cv::Mat bgrImage, cv::Mat mask )
{
	cv::Mat hsvImage; 
	cv::cvtColor( bgrImage, hsvImage, CV_BGR2HSV_FULL ); 

	cv::Mat histogram;
	int chan[] = { 0 }; 
	int histSize[] = { binNo };
	float hueRange[] = { 0, 255 };
	const float* ranges[] = { hueRange };
	
	//cv::Mat bgrMasked; 
	//bgrImage.copyTo( bgrMasked, mask ); 
	//cv::imshow("bgrMasked", bgrMasked); 
	
	cv::Mat satValMask; 
	cv::inRange( hsvImage, cv::Scalar( 0, 50, 50), cv::Scalar(255, 255, 205), satValMask);
	cv::Mat maskAnd = mask & satValMask; 

	// Blacks
	cv::Mat satM; 
	cv::inRange( hsvImage, cv::Scalar( 0, 0, 0), cv::Scalar(255, 50, 50), satM);
	cv::Mat maskAnd_1 = mask & satM; 
	int blackPx = cv::countNonZero( maskAnd_1 ); 

	// Whites
	cv::Mat valM; 
	cv::inRange( hsvImage, cv::Scalar( 0, 0, 205), cv::Scalar(255, 50, 255), valM);
	cv::Mat maskAnd_2 = mask & valM; 
	int whitePx = cv::countNonZero( maskAnd_2 ); 

	/*cv::imshow( "mask", mask ); 
	cv::imshow( "satValMask", satValMask); 
	cv::imshow( "bitAnd", maskAnd);

	cv::imshow( "maskAnd_1", maskAnd_1);
	cv::imshow( "maskAnd_2", maskAnd_2);
	cv::waitKey(30); */

	calcHist(&hsvImage, 1, chan, maskAnd, histogram, 1, histSize, ranges, true, false);
	//std::cout << "histogram" << histogram << std::endl; 

	cv::Mat newHisto = cv::Mat( histogram.rows+2, histogram.cols, histogram.type() ); 
	for( int i=0; i< histogram.rows; i++)
		newHisto.at<float>(i,0) = histogram.at<float>( i, 0); 
	
	newHisto.at<float>( histogram.rows , 0 ) = blackPx; 
	newHisto.at<float>( histogram.rows+1 , 0 ) = whitePx; 
	/*std::cout << "newHisto" << newHisto <<std::endl; */

	cv::normalize(newHisto, newHisto, 1.0, 0.0, cv::NORM_L1); 
	/*std::cout << "newHistoNorm" << newHisto <<std::endl; */

	//cv::normalize(histogram, histogram,  0, 1, CV_MINMAX); 
	return newHisto; 
}