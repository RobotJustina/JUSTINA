#include "ObjectRecognizer.hpp"

ObjectRecognizer::ObjectRecognizer(){
	
	this-> debugMode = false; 

	this-> detectorType = std::string( "SIFT" ); 
	this-> descriptorType = std::string( "SIFT" ); 

	//this-> trainingDir = "ObjRecoDir"; //Now this value is passed by param to the function TrainingFromDirectory
	this-> trainingFile = "ObjRecoFile" + std::string(".yml"); 
	
	// If we use Sift or Surf we must initialize this. 
	cv::initModule_nonfree(); 

	this-> featDetector = cv::FeatureDetector::create( this-> detectorType ); 
	this-> featDescriptor = cv::DescriptorExtractor::create( this->descriptorType ); 
	
	if( (this-> descriptorType == "SIFT") || (this-> descriptorType == "SURF") )
		this-> featMatcher = cv::DescriptorMatcher::create("FlannBased");
	else
		this-> featMatcher = cv::DescriptorMatcher::create("BruteForce-Hamming(2)");

	this-> maxBestMatches = 30;	
}

ObjectRecognizer::ObjectRecognizer(std::string detectorType, std::string descriptorType, std::string trainingFile)
{
	this-> detectorType = std::string( detectorType ); 
	this-> descriptorType = std::string( descriptorType ); 
	
	//this-> trainingDir = trainingDir; //Now this value is passed by param to the function TrainingFromDirectory
	this-> trainingFile = trainingFile + std::string(".yml"); 

	// If we use Sift or Surf we must initialize this. 
	cv::initModule_nonfree(); 

	this-> featDetector = cv::FeatureDetector::create( this-> detectorType ); 
	this-> featDescriptor = cv::DescriptorExtractor::create( this->descriptorType ); 
	
	if( (this-> descriptorType == "SIFT") || (this-> descriptorType == "SURF") )
		this-> featMatcher = cv::DescriptorMatcher::create("FlannBased");
	else
		this-> featMatcher = cv::DescriptorMatcher::create("BruteForce-Hamming(2)");

	this-> maxBestMatches = 30;
}

void ObjectRecognizer::TrainingFromDirectory(std::string trainingDir)
{	
	trainingDir = "ObjRecoDir"; 
	this-> detectorType = "SIFT"; 
	this-> descriptorType = "SIFT"; 

	std::cout << "ObjectRecognizer: Training from: dir[" << trainingDir << "]" << std::endl ; 
	std::cout << "	ObjectRecognizer: "	<< "feat[" << this->detectorType << "] , " 
										<< "desc[" << this->descriptorType << "]" << std::endl; 

	// Check if dir Exist. If not create it; 
//THIS BLOCK WAS USED IN THE WINDOWS IMPLEMENTATION BY MR JESUS
/*
	DWORD dirAttrib = ::GetFileAttributesA( this->trainingDir.c_str() );
	if(dirAttrib == INVALID_FILE_ATTRIBUTES){
		std::cout << "	ObjectRecognizer: Directory {"<< this->trainingDir.c_str() << "} doesnt exist. Creating it... " << std::endl ; 
		_mkdir( this->trainingDir.c_str() ); 
	}
*/
	// Check if dir Exist. If not create it; 
	if(!boost::filesystem::exists(trainingDir))
	{
		std::cout << "	ObjectRecognizer: Directory {"<< trainingDir << "} doesnt exist. Creating it... " << std::endl; 
		boost::filesystem::create_directory(trainingDir);
	}

	// Reading fileNames
	std::vector< std::string > imaName;// = UtilitiesVSN::GetFilesFromDir( this->trainingDir, ".jpg" ); 
	for( size_t i = 0 ; i < imaName.size() ; i++ ) 
	{
		std::string imageFilePath = trainingDir + "\\" +  imaName[i]; 
		std::string imaNameWithOutExt = imaName[i].substr(0, imaName[i].find_last_of(".") ); 

		cv::Mat ima = imread( imageFilePath, cv::IMREAD_GRAYSCALE ); 
		if( !ima.data ) 
		{
			std::cout << "	ObjectRecognizer: Cant read image " << imaName[i] << " for feat. extraction." << std::endl;; 
			continue; 
		}
		
		std::vector< cv::KeyPoint > keyPoints; 
		this-> featDetector->detect( ima, keyPoints );
		
		if( keyPoints.size() < 4 ) {
			std::cout << "	ObjectRecognizer: Cant describe image: {" << imaNameWithOutExt << "}. Not enough descriptors" << std::endl; 
			continue; 
		}
		
		cv::Mat descriptors;
		this->featDescriptor->compute(ima, keyPoints, descriptors); 

		this->trainingImages.push_back( ima ); 
		this->trainingNames.push_back( imaNameWithOutExt ); 		

		this->trainingFeatures.push_back( keyPoints );
		this->trainingDescriptors.push_back( descriptors );
			
		std::cout << "	ObjectRecognizer: Marker trained = " << imaNameWithOutExt << " ("<< descriptors.size() << ")"<< std::endl;
	}

	this->featMatcher->add( trainingDescriptors ); 
	this->featMatcher->train(); 
}

std::string ObjectRecognizer::RecognizeObject(cv::Mat& originalImage, cv::Mat& outObjectInScene)
{
	int64 totalTime = 0; 
	int64 homographyTime = 0; 
	int64 matchesTime = 0; 
	int64 work_begin = 0;
	int64 work_end = 0;

	std::string name = ""; 
	outObjectInScene = cv::Mat::zeros( originalImage.rows, originalImage.cols, CV_8UC3 ); 

	cv::Mat ima; 
	if( originalImage.channels() != 1) 
		cvtColor( originalImage, ima, CV_BGR2GRAY );
	else
		ima = originalImage; 

	// Obtaining Features
	std::vector< cv::KeyPoint > sceneFeatures; 
	this->featDetector->detect( ima, sceneFeatures );

	// If features are too few, cant calculate homography 
	if( sceneFeatures.size() < 4 )
		return ""; 

	// obtaining Descriptors
	cv::Mat sceneDescriptors;
	this->featDescriptor->compute(ima, sceneFeatures, sceneDescriptors); 		
	

	work_begin = cv::getTickCount(); 

	std::vector< std::vector< cv::DMatch > > matchesArr;		
	this->featMatcher->knnMatch( sceneDescriptors, matchesArr, 2); 
		
	// Cheking for good Matches
	std::vector< cv::DMatch > totalMatches;
	for( int m=0; m<matchesArr.size() ; m++){
		double ratio = matchesArr[m][0].distance / matchesArr[m][1].distance; 
		if( ratio < 0.6 )
			totalMatches.push_back( matchesArr[m][0] ); 
	}

		
	for(size_t i = 0; i < this->trainingDescriptors.size() ; i++ ){
		std::vector< cv::DMatch > matches;
		for( int m=0; m<totalMatches.size(); m++){
			if( totalMatches[m].imgIdx == i )
				matches.push_back( totalMatches[m] ); 
		}

		if( matches.size() < 4 )
			continue; 

		cv::Mat kp1; 
		drawKeypoints( trainingImages[i], trainingFeatures[i], kp1 ); 
		cv::imshow("kp1", kp1); 
		cv::Mat kp2; 
		drawKeypoints( ima, sceneFeatures, kp2 ); 
		cv::imshow("kp2", kp2); 
		cv::Mat drawMatchesMat;

 		drawMatches(ima, sceneFeatures, trainingImages[i], trainingFeatures[i], matches, drawMatchesMat, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
		cv::imshow("drawMatchesMat", drawMatchesMat); 
		cv::waitKey(10); 

		// sort matches by distance
 		int maxMatches = this->maxBestMatches; 
		std::sort(matches.begin(), matches.end());

		// Getting best MaxMatches or fewer if there are fewer 
		std::vector< cv::DMatch > bestMatches; 
		if( (int)matches.size() < maxMatches )
			bestMatches = std::vector< cv::DMatch >(matches); 
		else
			bestMatches = std::vector< cv::DMatch >(matches.begin(), matches.begin() + maxMatches) ;

	work_end = cv::getTickCount() - work_begin;
	matchesTime += work_end; 

	work_begin = cv::getTickCount(); 	

		// En esta implementacion, train son los de la escena o imagen original (dado que el arbol se generó o sea se "entreno" a partir de esta 
		// Query points son los de el entrenamiento , 
 		std::vector< cv::Point2f> scenePoints;
		std::vector< cv::Point2f> trainingPoints;
		for( size_t j=0; j<bestMatches.size(); j++)
		{ 
			scenePoints.push_back( sceneFeatures[ bestMatches[j].queryIdx ].pt );
			trainingPoints.push_back( trainingFeatures[i][ bestMatches[j].trainIdx ].pt );
		}

		cv::Mat H; 
		H = findHomography(trainingPoints, scenePoints, CV_RANSAC, 1.0);

		//-- Get the corners from the image_1 ( the object to be "detected" )
		std::vector< cv::Point2f > trainCorners(4);
		trainCorners[0] = cv::Point2f(0.0f , 0.0f); 
		trainCorners[1] = cv::Point2f( this->trainingImages[i].cols, 0.0f );
		trainCorners[2] = cv::Point2f( this->trainingImages[i].cols, this->trainingImages[i].rows ); 
		trainCorners[3] = cv::Point2f( 0.0f, this->trainingImages[i].rows );
		
		std::vector< cv::Point2f > trainCornersTrans(4);
		perspectiveTransform(trainCorners, trainCornersTrans, H);  

		bool isValidhomography = ValideteHomography( trainCorners, trainCornersTrans, H ); 

	work_end = cv::getTickCount() - work_begin;
	homographyTime += work_end; 


		cv::Mat trainInScene; 
		cv::warpPerspective( this->trainingImages[i], trainInScene, H, cv::Size(outObjectInScene.cols, outObjectInScene.rows )); 
		
		outObjectInScene = trainInScene;
		cv::imshow("outObjectInScene", outObjectInScene);
		if( isValidhomography ){
			name = trainingNames[i];
			std:: cout << "Found" << trainingNames[i] << std::endl ;
			return name; 
		}
	}
	
	totalTime = matchesTime + homographyTime; 
	
	std::cout << "matchesTime    = " << matchesTime/((double)cvGetTickFrequency() * 1000.)		<< "[ms]" << std::endl; 
	std::cout << "homographyTime = " << homographyTime/((double)cvGetTickFrequency() * 1000.)	<< "[ms]" << std::endl; 
	std::cout << "totalTime    = " << totalTime/((double)cvGetTickFrequency() * 1000.)		<< "[ms]" << std::endl; 

	return name; 
}

bool ObjectRecognizer::ValideteHomography( std::vector< cv::Point2f > trainCorners,  std::vector< cv::Point2f > trainCornersTrans, cv::Mat homography )
{ 
	// Validate Convex Contour 
	if( !cv::isContourConvex( trainCornersTrans ) )
		return false;

	// Validate clockwise arrangment
	double area;
	area = cv::contourArea( trainCornersTrans, true );
	if( area < 0 )
		return false;

	// Validete area 
	double areaOriginal; 
	areaOriginal = cv::contourArea( trainCorners ); 
	
	//if( area > 1.5*areaOriginal )
	//	return false; 

	//if( area < 0.5*areaOriginal )
	//	return false;


	// Validate getting line segments intersection: A-B must not intersct C-D && A-D must not interset B-C
	bool ab_cd_intersect = true; 
	bool ad_bc_intersect = true; 


	ab_cd_intersect = LineSegment::Intersection(trainCornersTrans[0].x, trainCornersTrans[0].y , 
												trainCornersTrans[1].x, trainCornersTrans[1].y , 
												trainCornersTrans[2].x, trainCornersTrans[2].y , 
												trainCornersTrans[3].x, trainCornersTrans[3].y ); 
										
	ad_bc_intersect = LineSegment::Intersection(trainCornersTrans[0].x, trainCornersTrans[0].y , 
												trainCornersTrans[3].x, trainCornersTrans[3].y , 
												trainCornersTrans[1].x, trainCornersTrans[1].y , 
												trainCornersTrans[2].x, trainCornersTrans[2].y ); 
	
	if( ab_cd_intersect || ad_bc_intersect)
		return false; 
	else 
		return true; 
}



//bool ObjectRecognizer::ValideteHomography( vector< cv::Point2f > trainCornersTrans, cv::Mat homography )
//{ 
//	// Validate Getting 
//	if( !cv::isContourConvex( trainCornersTrans ) )
//		return false;
//
//	// Validate getting line segments intersection: A-B must not intersct C-D && A-D must not interset B-C
//	bool ab_cd_intersect = true; 
//	bool ad_bc_intersect = true; 
//
//	double trainTransContour = cv::contourArea( trainCornersTrans, true); 
//
//	ab_cd_intersect = LineIntersection( trainCornersTrans[0].x, trainCornersTrans[0].y , 
//										trainCornersTrans[1].x, trainCornersTrans[1].y , 
//										trainCornersTrans[2].x, trainCornersTrans[2].y , 
//										trainCornersTrans[3].x, trainCornersTrans[3].y ); 
//										
//	ad_bc_intersect = LineIntersection( trainCornersTrans[0].x, trainCornersTrans[0].y , 
//										trainCornersTrans[3].x, trainCornersTrans[3].y , 
//										trainCornersTrans[1].x, trainCornersTrans[1].y , 
//										trainCornersTrans[2].x, trainCornersTrans[2].y ); 
//
//	//ab_cd_intersect = LineIntersection( trainCornersTrans[0], trainCornersTrans[1], trainCornersTrans[2], trainCornersTrans[3] );    
//	//ad_bc_intersect = LineIntersection( trainCornersTrans[0], trainCornersTrans[3], trainCornersTrans[1], trainCornersTrans[2] );    
//	
//	if( ab_cd_intersect || ad_bc_intersect)
//		return false; 
//	else 
//		return true; 
//}

//bool ObjectRecognizer::LineIntersection(float p0_x, float p0_y, float p1_x, float p1_y, float p2_x, float p2_y, float p3_x, float p3_y)
//{
//    double s1_x, s1_y, s2_x, s2_y;
//    s1_x = p1_x - p0_x;		s1_y = p1_y - p0_y;
//    s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;
//
//    double s, t;
//    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
//    t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);
//
//    if (s >= 0 && s <= 1 && t >= 0 && t <= 1)    
//        return true;
//	else
//		return false; // No collision
//}
