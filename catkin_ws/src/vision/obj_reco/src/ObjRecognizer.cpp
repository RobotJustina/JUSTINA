#include "ObjRecognizer.hpp"

ObjRecognizer::ObjRecognizer(int binNo)
{
	this->binNo = binNo; 
	this->TrainingDir = "TrainingDir";

	this->heightErrorThres = 0.01; 
	this->shapeErrorThres = 0.2;
	this->colorErrorThres = 0.6; 

	// Getting params from config file. 
	std::string configDir = ros::package::getPath("obj_reco") + "/ConfigDir";
	if( !boost::filesystem::exists(configDir ) )
		boost::filesystem::create_directory(configDir); 

	std::string configFile = configDir + "/ObjRecognizerConfig.xml"; 
	cv::FileStorage fs; 
	if( fs.open( configFile, fs.READ) ) 
	{
		this->heightErrorThres = (float)fs["heightErrorThres"]; 
		this->shapeErrorThres = (float)fs["shapeErrorThres"]; 
		this->colorErrorThres = (float)fs["colorErrorsVec"];  

		std::cout << "Readed configFile " << configFile << std::endl;  
		std::cout << "	- heightErrorThres: " << this->heightErrorThres << std::endl;
		std::cout << "	- shapeErrorThres: " << this->shapeErrorThres << std::endl;
		std::cout << "	- colorErrorsVec: " << this->colorErrorThres << std::endl;

		fs.release(); 
	}
	else
	{
		if(fs.open( configFile, fs.WRITE ) )
		{
			fs << "heightErrorThres" << this->heightErrorThres; 
			fs << "shapeErrorThres" << this->shapeErrorThres; 
			fs << "colorErrorsVec" << this->colorErrorThres; 

			fs.release(); 
		}
	}
}

ObjRecognizer::ObjRecognizer()
{
	this->binNo = 18; 
	this->TrainingDir = "TrainingDir";

	this->heightErrorThres = 0.01; 
	this->shapeErrorThres = 0.2;
	this->colorErrorThres = 0.6; 

	// Getting params from config filer 
	std::string configDir = ros::package::getPath("obj_reco") + "/ConfigDir";
	if( !boost::filesystem::exists(configDir ) )
		boost::filesystem::create_directory(configDir); 

	std::string configFile = configDir + "/ObjRecognizerConfig.xml"; 
	cv::FileStorage fs; 
	if( fs.open( configFile, fs.READ) ) 
	{
		this->heightErrorThres = (float)fs["heightErrorThres"]; 
		this->shapeErrorThres = (float)fs["shapeErrorThres"]; 
		this->colorErrorThres = (float)fs["colorErrorsVec"];  

		fs.release(); 
	}
	else
	{
		if(fs.open( configFile, fs.WRITE ) )
		{
			fs << "heightErrorThres" << this->heightErrorThres; 
			fs << "shapeErrorThres" << this->shapeErrorThres; 
			fs << "colorErrorsVec" << this->colorErrorThres; 

			std::cout << "Readed " << configFile << std::endl;  

			fs.release(); 
		}
	}

}

std::string ObjRecognizer::RecognizeObject(DetectedObject detObj, cv::Mat bgrImage)
{
	std::vector<double> heightErrorsVec; 
	std::vector<double> shapeErrorsVec; 
	std::vector<double> colorErrorsVec;
	
	// Getting ERRORS
	cv::Mat detObjHisto = CalculateHistogram( bgrImage, detObj.oriMask ); 
	for( int i=0; i< this->trainingNames.size(); i++)
	{
		// Getting Height Errors 
		float heightError = std::abs( detObj.height - this->trainingHeights[i] ); 
		heightErrorsVec.push_back( heightError ); 
		
		// Getting Shape Errors
		double shapeError = cv::matchShapes( detObj.shadowContour2D, this->trainingCont2D[i], CV_CONTOURS_MATCH_I1, 0.0); 
		shapeErrorsVec.push_back( shapeError ); 

		// Getting Color Errors
		double colorError = cv::compareHist( detObjHisto, this->trainingHistos[i], CV_COMP_INTERSECT);
		colorErrorsVec.push_back( colorError ); 
	}
	
    // recognizing Object 
	std::string recoName = "";
	double bestColorErrorSoFar = 0.0; 
	for( int i=0; i<this->trainingNames.size(); i++)
	{
		if( heightErrorsVec[i] < this->heightErrorThres && shapeErrorsVec[i] < this->shapeErrorThres && colorErrorsVec[i] > this->colorErrorThres  )
		{
			if( colorErrorsVec[i] > bestColorErrorSoFar )
			{
				bestColorErrorSoFar = colorErrorsVec[i]; 
				recoName = this->trainingNames[i]; 
			}
		}

	}

	return recoName;
}

bool ObjRecognizer::LoadTrainingDir()
{
    try
    {
        std::cout << "\nObjRecognizer.LoadTrainingDir ->Trying to load training dir: " << this->TrainingDir << std::endl;

        if( !boost::filesystem::exists(this->TrainingDir) )
        {
            std::cout << "\nObjRecognizer.LoadTrainingDir-> Training dir doesnt exist." << this->TrainingDir << std::endl;
            return false; 
        }        

        cv::FileStorage fs; 
        std::string nodeName = "obj"; 

        std::vector< std::string > trainingNames; 
        std::vector< int > trainingIds; 
        std::vector< float > trainingHeights; 
        std::vector< cv::Mat > trainingHistos; 
        std::vector< std::vector< cv::Point2f > > trainingCont2D; 

        boost::filesystem::path pathTrainDir( this->TrainingDir ); 
        boost::filesystem::directory_iterator endIt; 
        for( boost::filesystem::directory_iterator dirIt( pathTrainDir ) ; dirIt != endIt ; ++dirIt )
        {
            if( boost::filesystem::is_directory( dirIt->status() ) )
            {
                boost::filesystem::path p = dirIt->path(); 
                std::string trainingFilePath  = p.string() +"/" + p.filename().string() + ".xml"; 
                std::string objName = p.filename().string(); 

                // Loading for create new 
                int idCnt = 0;
                if( fs.open( trainingFilePath, fs.READ) ) 
                {				
                    cv::FileNode contoursNode = fs[ nodeName ]; 
                    cv::FileNodeIterator it = contoursNode.begin(); 
                    cv::FileNodeIterator it_end = contoursNode.end(); 

                    for( ; it != it_end ; ++it )
                    {
                        int oId = (int)(*it)["id"]; 

                        float oHeight = (float)(*it)["height"]; 

                        std::vector < cv::Point2f > oCont; 
                        (*it)["contour2d"] >> oCont; 

                        cv::Mat oHist; 
                        (*it)["histogram"] >> oHist; 

                        trainingNames.push_back( objName ); 
                        trainingIds.push_back( oId ); 
                        trainingHeights.push_back( oHeight  ); 
                        trainingHistos.push_back( oHist ); 
                        trainingCont2D.push_back( oCont ) ; 
                    }	
                    fs.release(); 
                }
            }

        }
        this->trainingNames   =   trainingNames   ;   
        this->trainingIds     =   trainingIds     ;
        this->trainingHeights =   trainingHeights ;
        this->trainingHistos  =   trainingHistos  ;
        this->trainingCont2D  =   trainingCont2D  ;

        for(int i=0; i< trainingNames.size(); i++)
        {
            std::cout << "ObjReco Trained: [" << i << "] "  << this->trainingNames[i] << " Hei:" <<  this->trainingHeights[i] << std::endl; 
        }
    }

    catch(std::exception& e)
    {
       std::cout << "Exception at LoadTrainingDir: " << e.what() << std::endl; 
        return false; 
    }
}

bool ObjRecognizer::TrainObject(DetectedObject detObj, cv::Mat bgrImage, std::string name)
{
	try
	{
        std::cout << "\nTrainingDir:" <<  this->TrainingDir << std::endl;   
		std::string trainingDirPath = this->TrainingDir;

        // Checking if directory of training exists.
		if( !boost::filesystem::exists(trainingDirPath) )
			boost::filesystem::create_directory(trainingDirPath); 

		std::string objDirPath = trainingDirPath + std::string("/") +  name;  
		// Checking if directory of object exist
		if( !boost::filesystem::exists(objDirPath) )
			boost::filesystem::create_directory(objDirPath); 

		std::string objFilePath = objDirPath + std::string("/") + name + std::string(".xml"); 

		std::vector< int > objIdVec; 
		std::vector< float > objHeightVec; 
		std::vector< std::vector< cv::Point2f > > objCont2DVec; 
		std::vector< cv::Mat > objHistoVec; 

		cv::FileStorage fs; 
		std::string nodeName = "obj"; 

		// Loading for create new 
		int idCnt = 0; 
		if( fs.open( objFilePath, fs.READ) ) 
		{
			cv::FileNode contoursNode = fs[ nodeName ]; 
			cv::FileNodeIterator it = contoursNode.begin(); 
			cv::FileNodeIterator it_end = contoursNode.end(); 

			for( ; it != it_end ; ++it )
			{
				int oId = (int)(*it)["id"]; 

				float oHeight = (float)(*it)["height"]; 

				std::vector < cv::Point2f > oCont; 
				(*it)["contour2d"] >> oCont; 

				cv::Mat oHist; 
				(*it)["histogram"] >> oHist; 

				objIdVec.push_back( oId ); 
				objHeightVec.push_back( oHeight ); 
				objCont2DVec.push_back( oCont ); 
				objHistoVec.push_back( oHist ); 
		
				if( idCnt <= oId )
					idCnt = oId; 
			}
			fs.release(); 
		}

		idCnt ++ ; 

		objIdVec.push_back( idCnt ); 
		objHeightVec.push_back( detObj.height ); 
		objCont2DVec.push_back( detObj.shadowContour2D ); 
		objHistoVec.push_back( this->CalculateHistogram( bgrImage, detObj.oriMask) ); 

		if( fs.open( objFilePath, fs.WRITE) )
		{
			fs << nodeName << "["; 
			for( int i=0; i< objIdVec.size(); i++)
			{
				fs << "{:"; 
				fs << "id" << objIdVec[i]; 
				fs << "height" << objHeightVec[i]; 
				fs << "contour2d" << objCont2DVec[i]; 
				fs << "histogram" << objHistoVec[i]; 
				fs << "}"; 	
			}
			fs << "]"; 
			fs.release(); 

			// Saving image to file; 
			cv::Mat masked; 
			bgrImage.copyTo( masked, detObj.oriMask );
			cv::Mat objIma = masked( detObj.boundBox );
			std::stringstream ss; 	
			ss << objDirPath << std::string("/") <<  name << "_" << idCnt <<".jpg"; 
			cv::imwrite(ss.str(), objIma); 

			std::cout << "Trained obj [" << objFilePath << "]" << std::endl; 
		}
		else
		{
			std::cout << "Cant write trining file: " << objFilePath << std::endl; 
			return false; 
		}
	}
	catch(std::exception& e) 
    {
		std::cout << "Exception at TrainObject: " << e.what() << std::endl; 
		return false; 
	}

	return true; 
}

cv::Mat ObjRecognizer::CalculateHistogram( cv::Mat bgrImage, cv::Mat mask )
{
	cv::Mat hsvImage; 
	cv::cvtColor( bgrImage, hsvImage, CV_BGR2HSV_FULL ); 

	cv::Mat histogram;
	int chan[] = { 0 }; 
	int histSize[] = { this->binNo };
	float hueRange[] = { 0, 255 };
	const float* ranges[] = { hueRange };
	
	//cv::Mat bgrMasked; 
	//bgrImage.copyTo( bgrMasked, mask ); 
	//cv::imshow("bgrMasked", bgrMasked); 
	
	//Hues
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
