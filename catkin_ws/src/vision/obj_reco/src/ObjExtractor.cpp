#include "ObjExtractor.hpp"

bool ObjExtractor::DebugMode = false; 
bool ObjExtractor::UseBetterPlanes = false; 

int rhoRes = 0; 
int degRes = 0; 
int cntThr = 100; 
int minLen = 100; 
int maxGap = 50; 

cv::Scalar ObjExtractor::frontLeftTop = cv::Scalar(0.5, -0.25, 1.0);
cv::Scalar ObjExtractor::backRightTop = cv::Scalar(1.5,  0.25, 1.5); 

int ObjExtractor::H = 130, ObjExtractor::S = 127, ObjExtractor::V = 127;
int ObjExtractor:: Hth = 10, ObjExtractor::Sth = 80, ObjExtractor::Vth = 80;


std::vector<PlanarSegment>  ObjExtractor::GetHorizontalPlanes(cv::Mat pointCloud)
{
	// PARAMS: Valid Points 
	double floorDistRemoval = 0.15;
	// PARAMS: Normals Extraction
	int blurSize = 5; 
	double normalZThreshold = 0.8; 
	// PARAMS: Planes RANSAC 
	double maxDistToPlane = 0.02; 
	int maxIterations = 1000; 
	int minPointsForPlane = pointCloud.rows*pointCloud.cols*0.05;
	// PARAMS: Object Extracction
	double minObjDistToPlane = maxDistToPlane; 
	double maxObjDistToPlane = 0.25; 
	double minDistToContour = 0.02; 
	double maxDistBetweenObjs = 0.05; 

	// For removing floor and far far away points 
	cv::Mat validPointCloud;
	cv::inRange(pointCloud, cv::Scalar(-3.0, -3.0, floorDistRemoval), cv::Scalar(3.0, 3.0, 3.0), validPointCloud); 
	
	// Getting Normals 	
	cv::Mat pointCloudBlur;
	cv::blur(pointCloud, pointCloudBlur, cv::Size(blurSize, blurSize));
	cv::Mat normals = ObjExtractor::CalculateNormals( pointCloudBlur ); 

	// Getting Mask of Normals pointing horizonaliy
	cv::Mat  horizontalNormals;
	cv::inRange( normals, cv::Scalar(-1.0, -1.0, normalZThreshold), cv::Scalar(1.5,1.0, 2.0), horizontalNormals); 

	// Mask of horizontal normals and valid.  
	cv::Mat horizontalsValidPoints = horizontalNormals & validPointCloud; 

	//Getting horizontal planes.
	return	ObjExtractor::ExtractHorizontalPlanesRANSAC(pointCloud, maxDistToPlane, maxIterations, minPointsForPlane, horizontalsValidPoints);
}

cv::Vec4i ObjExtractor::GetLine(cv::Mat pointCloud)
{

	// PARAMS: Valid Points 
	double floorDistRemoval = 0.15;
	// PARAMS: Normals Extraction
	int blurSize = 5; 
	double normalZThreshold = 0.8; 
	// PARAMS: Planes RANSAC 
	double maxDistToPlane = 0.02; 
	int maxIterations = 1000; 
	int minPointsForPlane = pointCloud.rows*pointCloud.cols*0.05;
	// PARAMS: Object Extracction
	double minObjDistToPlane = maxDistToPlane; 
	double maxObjDistToPlane = 0.25; 
	double minDistToContour = 0.02; 
	double maxDistBetweenObjs = 0.05; 

	// For removing floor and far far away points 
	cv::Mat validPointCloud;
	cv::inRange(pointCloud, cv::Scalar(-1.0, -1.0, floorDistRemoval), cv::Scalar(1.5, 1.0, 2.0), validPointCloud); 
	
	// Getting Normals 	
	cv::Mat pointCloudBlur;
	cv::blur(pointCloud, pointCloudBlur, cv::Size(blurSize, blurSize));
	cv::Mat normals = ObjExtractor::CalculateNormals( pointCloudBlur ); 

	// Getting Mask of Normals pointing horizonaliy
	cv::Mat  horizontalNormals;
	cv::inRange( normals, cv::Scalar(-1.0, -1.0, normalZThreshold), cv::Scalar(1.0,1.0, 1.0), horizontalNormals); 

	// Mask of horizontal normals and valid.  
	cv::Mat horizontalsValidPoints = horizontalNormals & validPointCloud; 

	//Getting horizontal planes.
	std::vector<PlanarSegment> horizontalPlanes = ObjExtractor::ExtractHorizontalPlanesRANSAC(pointCloud, maxDistToPlane, maxIterations, minPointsForPlane, horizontalsValidPoints);

	if( DebugMode )
	{
		cv::namedWindow( "Trackbars"); 
		cv::createTrackbar( "rhoRes", "Trackbars", &rhoRes, 100 );
		cv::createTrackbar( "degRes", "Trackbars", &degRes, 100 );
		cv::createTrackbar( "cntThr", "Trackbars", &cntThr, 500 );
		cv::createTrackbar( "minLen", "Trackbars", &minLen, 500 );
		cv::createTrackbar( "maxGap", "Trackbars", &maxGap, 100 );
	}

	std::vector<cv::Vec4i> totalLines; 
	// check for every plane the lines; 
	for( int i=0; i<(int)horizontalPlanes.size(); i++)
	{
		cv::Mat planeIma = cv::Mat::zeros(pointCloud.rows, pointCloud.cols, CV_8UC1);  
		std::vector< cv::Point2i > indexes = horizontalPlanes[i].Get_Indexes(); 

		for( int j=0; j<(int)indexes.size(); j++)
			planeIma.at< uchar >( indexes[j] ) = 255;  

		int thresh = 100; 
		cv::Mat edgesIma; 
		cv::Canny( planeIma, edgesIma, thresh, thresh*2, 3 );

		if( DebugMode )
			cv::imshow( "edgesIma", edgesIma); 

		std::vector<cv::Vec4i> lines;
		cv::HoughLinesP( edgesIma, lines, 1+rhoRes, (degRes+1)*CV_PI/180, 1+cntThr, 1+minLen, 1+maxGap );

		totalLines.insert( totalLines.end(), lines.begin(), lines.end() );
	}

	cv::Vec4i bestLine; 
	float bestDist =100000.0; 
	for( int i=0; i<totalLines.size(); i++)
	{
		cv::Point3f iniLine = pointCloud.at<cv::Vec3f>( cv::Point(totalLines[i][0], totalLines[i][1]) ); 
		cv::Point3f endLine = pointCloud.at<cv::Vec3f>( cv::Point(totalLines[i][2], totalLines[i][3]) ); 

		if( iniLine == cv::Point3f(0,0,0) || endLine == cv::Point3f(0,0,0) )
			continue; 
		
		cv::Point3f midPoint = (iniLine + endLine)*(0.5); 
		
		// only proyection; 
		float distToRobot = midPoint.x*midPoint.x + midPoint.y*midPoint.y; 
		
		if( distToRobot < bestDist ) 
		{
			bestLine = totalLines[i];		
			bestDist = distToRobot; 
		}
	}

	if( DebugMode )	
	{
		cv::Mat linesIma = cv::Mat::zeros( pointCloud.size(), CV_8UC3 );
		std::cout << "lineCnt=" << totalLines.size() << std::endl; 
		for( int i = 0; i < totalLines.size(); i++ )
			cv::line( linesIma, cv::Point(totalLines[i][0], totalLines[i][1]), cv::Point(totalLines[i][2], totalLines[i][3]), cv::Scalar(0, 0, 255), 1, 8 );

		cv::line( linesIma, cv::Point(bestLine[0], bestLine[1]), cv::Point(bestLine[2], bestLine[3]), cv::Scalar(0, 255, 0), 3, 8 );
		cv::imshow( "linesIma", linesIma ); 		
	}

	return bestLine; 
}

std::vector<DetectedObject> ObjExtractor::GetObjectsInHorizontalPlanes(cv::Mat pointCloud)
{
	std::vector< DetectedObject > detectedObjectsList; 

	double ticks; 

	// PARAMS: Valid Points 
	double floorDistRemoval = 0.25;
	// PARAMS: Normals Extraction
	int blurSize = 5; 
	double normalZThreshold = 0.8; 
	// PARAMS: Planes RANSAC 
	double maxDistToPlane = 0.02; 
	int maxIterations = 1000; 
	int minPointsForPlane = pointCloud.rows*pointCloud.cols*0.025;
	// PARAMS: Object Extracction
	double minObjDistToPlane = maxDistToPlane; 
	double maxObjDistToPlane = 0.25; 
	double minDistToContour = 0.02; 
	double maxDistBetweenObjs = 0.05; 

	// For removing floor and far far away points 
	cv::Mat validPointCloud;
	cv::inRange(pointCloud, cv::Scalar(-1, -1.0, floorDistRemoval), cv::Scalar(1.5, 1.0, 2.0), validPointCloud); 
	
	// Getting Normals 	
	cv::Mat pointCloudBlur;
	cv::blur(pointCloud, pointCloudBlur, cv::Size(blurSize, blurSize));
	cv::Mat normals = ObjExtractor::CalculateNormals( pointCloudBlur ); 
	if(DebugMode)
		cv::imshow("normals", normals);

	// Getting Mask of Normals pointing horizonaliy
	cv::Mat  horizontalNormals;
	cv::inRange( normals, cv::Scalar(-1.0, -1.0, normalZThreshold), cv::Scalar(1.0,1.0, 1.0), horizontalNormals); 

	// Mask of horizontal normals and valid.  
	cv::Mat horizontalsValidPoints = horizontalNormals & validPointCloud; 


	//Getting horizontal planes.
	ticks = cv::getTickCount(); 
	std::vector<PlanarSegment> horizontalPlanes = ObjExtractor::ExtractHorizontalPlanesRANSAC(pointCloud, maxDistToPlane, maxIterations, minPointsForPlane, horizontalsValidPoints);
	if( DebugMode )
	{
		cv::Mat planesMat  = pointCloud.clone(); 
		for( int i=0; i<(int)horizontalPlanes.size(); i++)
		{
			std::vector< cv::Point2i > indexes = horizontalPlanes[i].Get_Indexes(); 
			cv::Vec3f color = ObjExtractor::RandomFloatColor();  
			for( int j=0; j<(int)indexes.size(); j++)
			{
				planesMat.at< cv::Vec3f >( indexes[j] ) = color; 
			}
		}
		cv::imshow("planesMat", planesMat); 
	}
	if(DebugMode)
		std::cout << "Getting horizontal planes t=" << ((double)cv::getTickCount() - ticks) / cv::getTickFrequency() << std::endl; 

	// Getting Mask of objects of every plane
	ticks = cv::getTickCount(); 
	std::vector< cv::Point3f > objectsPoints;
	std::vector< cv::Point2i > objectsIdx; 
	for( int i=0; i<(int)horizontalPlanes.size(); i++)
	{
		for(int row=0; row<pointCloud.rows; row++)
		{
			for(int col=0; col<pointCloud.cols; col++)
			{
				cv::Point3f xyzPoint = pointCloud.at< cv::Vec3f >(row, col ); 
				double dist = horizontalPlanes[i].Get_Plane().DistanceToPoint(xyzPoint, true);				

				if( minObjDistToPlane < dist && maxObjDistToPlane > dist )
				{
					double distToConvexHull = horizontalPlanes[i].IsInside( xyzPoint ); 
					if(distToConvexHull > minDistToContour)
					{
						objectsPoints.push_back( xyzPoint ); 
						objectsIdx.push_back( cv::Point2i(col, row) ); 
					}
				}
			}
		}
	}
	if(DebugMode)
		std::cout << "Getting Mask of objects of every plane t=" << ((double)cv::getTickCount() - ticks) / cv::getTickFrequency() << std::endl; 

	// Cluster objects by distance: 
	ticks = cv::getTickCount(); 
	std::vector< std::vector< int > > objIdxClusters  = ObjExtractor::SegmentByDistance( objectsPoints, maxDistBetweenObjs ); 
	
	cv::Mat objMat = cv::Mat::zeros(pointCloud.rows, pointCloud.cols, CV_8UC3); 
	for(int i=0; i<(int)objIdxClusters.size(); i++)
	{
		cv::Vec3b color( rand()%256, rand()%256, rand()%256);
		
		float maxZ = 0.0; 
		float minZ = 100000.0; 
		float objHeight	= 0.0; 
		cv::Point3f objCentroid( 0.0, 0.0, 0.0); 
	 	std::vector< cv::Point3f > objPoints3D; 
		std::vector< cv::Point2f > objPoints2D; 
		std::vector< cv::Point2i > objIndexes; 
		cv::Mat oriMask = cv::Mat::zeros(pointCloud.rows, pointCloud.cols, CV_8UC1);

		for( int j=0; j<(int)objIdxClusters[i].size(); j++)
		{
			cv::Point2i idx = objectsIdx[ objIdxClusters[i][j] ]; 
			cv::Point3f pnt = pointCloud.at< cv::Vec3f >( idx ); 
			// For Debug
			objMat.at<cv::Vec3b>( idx ) = color; 

			// Getting Object charactersics
			objCentroid += pnt; 
			
			if( pnt.z < minZ )
				minZ = pnt.z; 

			if( pnt.z > maxZ )
				maxZ = pnt.z; 
			
			objPoints3D.push_back( pnt );
			objPoints2D.push_back( cv::Point2f( pnt.x, pnt.y )) ; 
			objIndexes.push_back( idx ); 

			oriMask.at<uchar>( idx ) = 255; 
		}

		objHeight = std::abs(maxZ - minZ); 
		objCentroid *= (1.0f / (float)objIdxClusters[i].size()); 
		
		DetectedObject detObj = DetectedObject( objIndexes, objPoints3D, objPoints2D, objHeight, objCentroid, oriMask ); 
		if( detObj.shadowOriBoundBoxt2D.size.width <0.01 || detObj.shadowOriBoundBoxt2D.size.width > 0.25 )
			continue; 
		if( detObj.shadowOriBoundBoxt2D.size.height<0.01 || detObj.shadowOriBoundBoxt2D.size.height > 0.25 )
			continue; 
		if( detObj.height <0.01 || detObj.height > 0.25)
			continue;

		detectedObjectsList.push_back( detObj ); 
	}
	if( DebugMode )
		std::cout << "Cluster objects by distance: t=" << ((double)cv::getTickCount() - ticks) / cv::getTickFrequency() << std::endl; 

	if( DebugMode )
	{
		cv::imshow( "objMat", objMat );
	}

	return detectedObjectsList; 
}	

bool ObjExtractor::TrainGripper(cv::Mat imageBGR)
{
	cv::Rect2d roi;
	roi = cv::selectROI("Train Gripper",imageBGR,false ,false); 
	cv::Mat imageHSV;
	cv::cvtColor(imageBGR,imageHSV,CV_BGR2HSV);
	int numPoints = 0;
	int meanH =0;
	int max = -1 , min = 300;
	for (int i =roi.y ; i < roi.y+roi.height ; ++i)
	{
		for (int j =  roi.x ; j <roi.x+roi.width ; ++j)
		{
			int temp= (int)imageHSV.at<cv::Vec3b>(i,j)[0]; 
			meanH += temp;
			std::cout<<(int)temp<<std::endl;
			if (temp> max)
			{
				max= temp;
			}
			if (temp< min)
			{
				min= temp;
			}
			std::cout<<"min: "<<min<<" max: "<<max<<std::endl;
			++numPoints;
		}
	}
	if (numPoints<1)
	{
		cv::destroyWindow("Train Gripper");
		return false;
	}
	meanH/=numPoints;

	std::string configDir = ros::package::getPath("obj_reco") + "/ConfigDir";
	if( !boost::filesystem::exists(configDir ) )
		boost::filesystem::create_directory(configDir); 
	std::string configFile =configDir + "/ObjExtrGripperConfig.xml";
	cv::FileStorage fs;
	if(fs.open(configFile, fs.WRITE))
	{
		fs<< "H" << meanH;
		fs<< "S" << ObjExtractor::S;
		fs<< "V" << ObjExtractor::V;
		int th=(abs(meanH-max)>abs(meanH-min))?abs(meanH-max):abs(meanH-min);

		fs<< "Hth" << th;
		fs<< "Sth" << ObjExtractor::Sth;
		fs<< "Vth" << ObjExtractor::Vth;
		fs.release();

		ObjExtractor::H   = meanH; 
		ObjExtractor::Hth = th; 
		cv::destroyWindow("Train Gripper");
		return true;
	}
	cv::destroyWindow("Train Gripper");
	return false;

}

void ObjExtractor::LoadValueGripper()
{

	//ObjExtractor::H = 130, ObjExtractor::S = 127, ObjExtractor::V = 127;
	//ObjExtractor:: Hth = 10, ObjExtractor::Sth = 80, ObjExtrac tor::Vth = 80;

	std::string configDir = ros::package::getPath("obj_reco") + "/ConfigDir";
	if( !boost::filesystem::exists(configDir ) )
		boost::filesystem::create_directory(configDir); 
	std::string configFile =configDir + "/ObjExtrGripperConfig.xml";
	cv::FileStorage fs;

	if (fs.open(configFile, fs.READ ))
	{
		ObjExtractor::H   = (int)fs["H"]; 
		ObjExtractor::S   = (int)fs["S"]; 
		ObjExtractor::V   = (int)fs["V"]; 
		ObjExtractor::Hth = (int)fs["Hth"]; 
		ObjExtractor::Sth = (int)fs["Sth"]; 
		ObjExtractor::Vth = (int)fs["Vth"]; 
		
		fs.release();
	}else if(fs.open(configFile, fs.WRITE))
	{
		fs<< "H" << ObjExtractor::H;
		fs<< "S" << ObjExtractor::S;
		fs<< "V" << ObjExtractor::V;
		fs<< "Hth" << ObjExtractor::Hth;
		fs<< "Sth" << ObjExtractor::Sth;
		fs<< "Vth" << ObjExtractor::Vth;
		fs.release();
	}

}


cv::Vec3f ObjExtractor::GetGrippers(cv::Mat imageBGR, cv::Mat pointCloud)
{
	float minX = 0.10, maxX = 0.7;
	float minY = -0.7, maxY = 0.7;
	float minZ = 0.4, maxZ = 1.5;

	//int H = 130, S = 127, V = 127;
	//int Hth = 10, Sth = 80, Vth = 80, 
	int ItMor = 5;     

	cv::Mat imageHSV;
	cv::Mat maskHSV; 
	cv::Vec3f centroid (0.0, 0.0, 0.0); 

	//while(cv::waitKey(10)!='q'){
	cv::cvtColor(imageBGR,imageHSV,CV_BGR2HSV);
	cv::inRange(imageHSV,cv::Scalar(ObjExtractor::H-ObjExtractor::Hth, ObjExtractor::S-ObjExtractor::Sth,ObjExtractor::V-ObjExtractor::Vth),cv::Scalar(ObjExtractor::H+ObjExtractor::Hth,ObjExtractor::S+ObjExtractor::Sth,ObjExtractor::V+ObjExtractor::Vth),maskHSV);

	cv::Mat maskXYZ;
	cv::inRange(pointCloud,cv::Scalar(minX, minY,minZ),cv::Scalar(maxX,maxY,maxZ),maskXYZ);

	cv::Mat mask;
	maskXYZ.copyTo(mask,maskHSV);
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3,3));
	cv::morphologyEx(mask,mask,cv::MORPH_ERODE,kernel,cv::Point(-1,-1),2);
	cv::morphologyEx(mask,mask,cv::MORPH_CLOSE,kernel,cv::Point(-1,-1),ItMor);


	
	cv::Point imgCentroid(0,0);
	int numPoints = 0;
	for (int i = 0; i < mask.rows; ++i)
	{
		for (int j = 0; j < mask.cols; ++j)
		{
			if (mask.at<uchar>(i,j)>0)
			{
				centroid += pointCloud.at<cv::Vec3f>(i,j);
				imgCentroid += cv::Point(j,i);
				++numPoints;
			}
		}
	}
	if (numPoints == 0)
		return centroid;
	centroid /= numPoints;
	imgCentroid /= numPoints;
	
	cv::Mat maskedImage;
	imageBGR.copyTo(maskedImage,mask);
	cv::circle(maskedImage,imgCentroid,5,cv::Scalar(0,0,255),-1);
	//std::cout<<centroid<<std::endl;
	cv::imshow("Gripper",maskedImage);
	
	//}
	return centroid; 
}


// Return labels corresponding to wich cluster belong each point 
std::vector< std::vector< int > > ObjExtractor::SegmentByDistance(std::vector< cv::Point3f > xyzPointsList, double distThreshold)
{
	std::vector< int > labels(xyzPointsList.size(), 0); 
	std::vector< std::vector< int > > labelsVec; 

	if( xyzPointsList.size() == 0 )
		return labelsVec; 

	cv::Mat xyzPointMat = cv::Mat(xyzPointsList).reshape(1);

	cv::flann::KDTreeIndexParams idxParams(1); 
	cv::flann::Index kdTree(xyzPointMat, idxParams); 

	// KD uses distance without squareRoot 
	double distSquare = distThreshold*distThreshold; 

	int labelCnt = 0;
	for(int p=0; p<xyzPointMat.rows; p++)
	{
		if( labels[p] != 0 )
			continue; 

		std::vector< int > labelCluster; 

		labelCnt++;   
		labels[p] = labelCnt; 

		std::queue< cv::Mat > nnQueue; 	
		nnQueue.push( xyzPointMat.row(p) ); 

		labelCluster.push_back( p ); 

		while( nnQueue.size() > 0 )
		{
			cv::Mat currentPoint = nnQueue.front(); 
			nnQueue.pop(); 

			std::vector<float> dists(1000); 
			std::vector<int> idxs(1000); 
			kdTree.radiusSearch( currentPoint, idxs, dists, distSquare, 16 ); 

			for(int i=0; i<(int)idxs.size(); i++)
			{
				if( labels[ idxs[i] ] == 0)
				{
					labels[ idxs[i] ] = labelCnt; 
					nnQueue.push( xyzPointMat.row( idxs[i]) ); 
					labelCluster.push_back( idxs[i] );
				}
			}
		}
		labelsVec.push_back( labelCluster ); 
	}
	return labelsVec; 
}

std::vector<PlanarSegment> ObjExtractor::ExtractHorizontalPlanesRANSAC(cv::Mat pointCloud, double maxDistPointToPlane, int maxIterations, int minPointsForPlane, cv::Mat mask)
{
	std::vector< PlanarSegment > horizontalPlanesList; 

	// Getting mask indexe
	std::vector< cv::Point2i > indexes; 
	for( int i=0; i<mask.rows; i++)
	{
		for(int j=0; j< mask.cols; j++) 
		{
			if( mask.at<uchar>(i,j) != 0 )
				indexes.push_back( cv::Point(j,i) ); 
		}
	}

	int cntIteratiosn = 0; 
	while( cntIteratiosn++ < maxIterations && indexes.size() > minPointsForPlane)
	{
		cv::Point3f p1  = pointCloud.at< cv::Vec3f >( indexes[rand()%indexes.size()] ); 
		cv::Point3f p2  = pointCloud.at< cv::Vec3f >( indexes[rand()%indexes.size()] ); 
		cv::Point3f p3  = pointCloud.at< cv::Vec3f >( indexes[rand()%indexes.size()] ); 

		if( !Plane3D::AreValidPointsForPlane( p1, p2, p3))
			continue; 

		Plane3D candidatePlane( p1, p2, p3);

		// Heuristics ??
		if( std::abs(candidatePlane.GetNormal().z < 0.99) )
			continue; 

		// For all points, checking distance to candidate plane (getting inliers) 
		std::vector< cv::Point3f > inliers; 
		for( int i=0; i < (int)indexes.size(); i++)
		{
			cv::Point3f xyzPoint = pointCloud.at<cv::Vec3f>(indexes[i]);
			double distToPlane = candidatePlane.DistanceToPoint(xyzPoint);

			if( distToPlane < maxDistPointToPlane )
				inliers.push_back(xyzPoint); 
		}

		if( inliers.size() < minPointsForPlane )
			continue; 

		// Getting better plane using PCA
		cv::PCA pca( cv::Mat(inliers).reshape(1), cv::Mat(), CV_PCA_DATA_AS_ROW); 
		cv::Point3f pcaNormal( pca.eigenvectors.at<float>(2,0), pca.eigenvectors.at<float>(2,1), pca.eigenvectors.at<float>(2,2) ); 
		cv::Point3f pcaPoint(pca.mean.at<float>(0,0), pca.mean.at<float>(0,1), pca.mean.at<float>(0,2));

		Plane3D refinedPlane( pcaNormal, pcaPoint ); 

		// Obtaining Refined Plane
		std::vector< cv::Point3f > pointsPlane;
		std::vector< cv::Point2f > xyPointsPlane; 
		std::vector< cv::Point2i > indexesPlane;
		std::vector< cv::Point2i > indexesNew; 

		for( int i=0; i<(int)indexes.size() ; i++)
		{
			cv::Point3f xyzPoint = pointCloud.at<cv::Vec3f>(indexes[i]);
			double distToPlane = refinedPlane.DistanceToPoint(xyzPoint);

			if( distToPlane < maxDistPointToPlane )
			{
				pointsPlane.push_back( xyzPoint );
				xyPointsPlane.push_back( cv::Point2f(xyzPoint.x, xyzPoint.y) ); 
				indexesPlane.push_back( indexes[i] ); 
			}
			else
			{
				indexesNew.push_back( indexes[i] ); 
			}
		}
		indexes = indexesNew; 

		if( ObjExtractor::UseBetterPlanes )
		{
			std::cout << "Use better Planes" << std::endl; 
			// Segmenting plane
			std::vector< std::vector<int> > clusterPlane = SegmentByDistance( pointsPlane, 0.10 ); 
			int maxSize = 0; 
			int maxIndex = 0; 
			for(int i=0; i<clusterPlane.size(); i++)
			{
				if( clusterPlane[i].size() > maxSize) 
				{
					maxSize = clusterPlane[i].size(); 
					maxIndex = i; 
				}
			}

			// Return points if cluster are small 
			if( maxSize < minPointsForPlane )
			{
				indexes.insert( indexes.end(), indexesPlane.begin(), indexesPlane.end() ); 
				continue; 
			}

			// Remove only bigger cluster. 
			std::vector< cv::Point3f > finalPointsPlane;
			std::vector< cv::Point2i > finalIndexesPlane;
			std::vector< cv::Point2f > finalXYPointsPlane; 
			for( int i=0; i<clusterPlane.size(); i++)
			{
				for( int j=0; j<clusterPlane[i].size() ; j++)
				{
					if( i == maxIndex ) 
					{
						cv::Point3f ptPlane = pointsPlane[ clusterPlane[i][j] ];  

						finalPointsPlane.push_back(ptPlane); 
						finalIndexesPlane.push_back( indexesPlane[ clusterPlane[i][j] ] ); 
						finalXYPointsPlane.push_back(cv::Point2f(ptPlane.x, ptPlane.y)); 
					}
					else
					{
						indexes.push_back( indexesPlane[ clusterPlane[i][j] ] ) ; 
					}
				}
			}
			pointsPlane = finalPointsPlane ;
			indexesPlane = finalIndexesPlane;
			xyPointsPlane = finalXYPointsPlane; 
		}

		// GETTING REFINED PLANES 
		cv::PCA pca2( cv::Mat(pointsPlane).reshape(1), cv::Mat(), CV_PCA_DATA_AS_ROW); 
		cv::Point3f pcaNormal2( pca2.eigenvectors.at<float>(2,0), pca2.eigenvectors.at<float>(2,1), pca2.eigenvectors.at<float>(2,2) ); 
		cv::Point3f pcaPoint2( pca2.mean.at<float>(0,0), pca2.mean.at<float>(0,1), pca2.mean.at<float>(0,2));

		// Getting Convex Hull ( Convex hull is valid if remove Z, because is an horizontal plane ) 
		std::vector< cv::Point2f > convexHull2D; 
		cv::convexHull(xyPointsPlane , convexHull2D);	

		PlanarSegment ps = PlanarSegment( Plane3D(pcaNormal2, pcaPoint2), pointsPlane, indexesPlane, convexHull2D); 
		horizontalPlanesList.push_back( ps ); 
	}

	return horizontalPlanesList; 
}

cv::Mat ObjExtractor::CalculateNormals(cv::Mat pointCloud, cv::Mat mask)
{
	cv::Mat normals = cv::Mat::zeros(pointCloud.rows, pointCloud.cols, CV_32FC3); 

	if( !mask.data )
		mask = cv::Mat::ones(pointCloud.rows, pointCloud.cols, CV_8UC1); 

	cv::Point3f pointi; 
	cv::Point3f topLeft; 
	cv::Point3f topRight; 
	cv::Point3f downLeft; 
	cv::Point3f downRight; 

	cv::Point3f normal_1; 
	cv::Point3f normal_2; 
	cv::Point3f normal; 
			
	cv::Point3f viewPoint(0.0,0.0,0.0);

	for( int idxRow = 1 ; idxRow < pointCloud.rows - 1 ; idxRow++ )
	{
		for( int idxCol = 1 ; idxCol < pointCloud.cols - 1 ; idxCol++ )
		{			
			//if( mask.at<uchar>( idxRow,idxCol ) == 0.0 )
				//continue; 
			
			// Getting Vectors
			pointi = pointCloud.at<cv::Vec3f>(idxRow, idxCol);

			topLeft = pointCloud.at<cv::Vec3f>(idxRow-1, idxCol-1);
			topRight = pointCloud.at<cv::Vec3f>(idxRow-1, idxCol+1);
			downLeft = pointCloud.at<cv::Vec3f>(idxRow+1, idxCol-1);
			downRight = pointCloud.at<cv::Vec3f>(idxRow+1, idxCol+1);
			
			if( topLeft.x == 0.0 && topLeft.y == 0.0 && topLeft.z == 0.0 )
				continue; 
			if( topRight.x == 0.0 && topRight.y == 0.0 && topRight.z == 0.0 )
				continue; 
			if( downLeft.x == 0.0 && downLeft.y == 0.0 && downLeft.z == 0.0 )
				continue; 
			if( downRight.x == 0.0 && downRight.y == 0.0 && downRight.z == 0.0 )
				continue; 
			
			// Normals
			normal_1 = topRight - downLeft;
			normal_2 = topLeft - downRight; 

			// Normal by cross product (v x h)
			normal  = normal_2.cross(normal_1);

			// Make normal unitary and assignin to mat
			float norm = sqrt(normal.x*normal.x + normal.y*normal.y + normal.z*normal.z);
			if(norm != 0.0f)
			{
				// Proyecting all normals over XY plane
				//if(normal.dot(viewPoint-pointi)<0)
				//	normal *= -1;
				if( normal.z < 0 )
					normal *= -1; 

				normals.at<cv::Vec3f>(idxRow, idxCol) = ( 1.0f / norm ) * normal;
			}
		}
	}
	return normals;
}

cv::Vec3f ObjExtractor::RandomFloatColor()
{
	float x = (float)(rand()) / (float)(RAND_MAX);  
	float y = (float)(rand()) / (float)(RAND_MAX);  
	float z = (float)(rand()) / (float)(RAND_MAX);  

	return cv::Vec3f(x,y,z);  
}

std::vector<PlanarSegment> ObjExtractor::ExtractHorizontalPlanesRANSAC_2(cv::Mat pointCloud, double maxDistPointToPlane, int maxIterations, int minPointsForPlane, cv::Mat mask)
 {
 	std::cout << "NOT IMPLEMETNED YET !!!" << std::endl; 

	std::vector< PlanarSegment > horizontalPlanesList; 
	return horizontalPlanesList; 
	
	//// Getting mask indexe
	//std::vector< cv::Point2i > indexesVec; 
	//cv::Mat pointsRows(0, 3, CV_32FC1); 
	//for( int i=0; i<mask.rows; i++)
	//{
		//for(int j=0; j< mask.cols; j++) 
		//{
			//if( mask.at<uchar>(i,j) == 0 )
				//continue; 

			//cv::Point2i idx = cv::Point(j,i);
			//indexes.push_back( idx ); 
	
			//pointsRows.push_back( pointCloud.at<cv::Vec3f>( idx ); 
		//}
	//}

	////Creating KDTREE
	//cv::flann::KDTreeIndexParams idxParams; 
	//cv::flann::Index kdTree(, idxParams); 

	//int initRadius = 0.05; 

	////RANSAC (maybe ?)
	//int cntIteratiosn = 0; 
	//while( cntIteratiosn++ < maxIterations && remainingPointsIndexes.size() > minPointsForPlane)
	//{
		////Getting candidate plane using pca and NN
		
		//cv::Mat randPoint = pixelsTotalMat.row( remainingPointsIndexes[ rand() % remainingPointsIndexes.size() ] ); 
		
		//std::vector<float> tempDists(1000); 
		//std::vector<int> tempIdxs(1000); 
		//kdTree.radiusSearch( randPoint, tempIdxs, tempDists, initRadius*initRadius, 16 ); 

		//std::vector< cv::Point3f > tempNearestNeighs(1000); 
		//for( int i=0; i<tempIdxs.size(); i++)
			//tempNearestNeighs.push_back( pointsTotalVec[ tempIdxs[i]  ] );

		//cv::PCA pca( cv::Mat(tempNearestNeighs).reshape(1), cv::Mat(), CV_PCA_DATA_AS_ROW); 
		//cv::Point3f pcaNormal( pca.eigenvectors.at<float>(2,0), pca.eigenvectors.at<float>(2,1), pca.eigenvectors.at<float>(2,2) ); 
		//cv::Point3f pcaPoint(pca.mean.at<float>(0,0), pca.mean.at<float>(0,1), pca.mean.at<float>(0,2));
		
		//Plane3D candidatePlane( pcaNormal, pcaPoint ); 

		//// Heuristics
		//if( std::abs(candidatePlane.GetNormal().z < 0.99) )
			//continue; 
		//std::cout << "Candidate Plane ! i=" << cntIteratiosn << std:: endl; 

		//// Getting Neighbours of 
		
		
	//}
}

DetectedObject ObjExtractor::GetObjectInBox(cv::Mat& imaBGR, cv::Mat& imaXYZ)
{
    cv::Mat validMask; 
	cv::inRange(imaXYZ, ObjExtractor::frontLeftTop, ObjExtractor::backRightTop, validMask);  

    DetectedObject detObj =  DetectedObject(imaBGR, imaXYZ, validMask); 

    //cv::imshow("validMask", validMask);  
    return detObj; 
}

bool ObjExtractor::extractObjectsFromHorizontalPlanes(cv::Mat& imaXYZ, cv::Mat& objExtr){
    objExtr = cv::Mat::zeros(imaXYZ.size(), CV_8UC1);

	// PARAMS: Valid Points 
	double floorDistRemoval = 0.25;
	// PARAMS: Normals Extraction
	int blurSize = 5; 
	double normalZThreshold = 0.8; 
	// PARAMS: Planes RANSAC 
	double maxDistToPlane = 0.02; 
	int maxIterations = 1000; 
	int minPointsForPlane = imaXYZ.rows * imaXYZ.cols*0.025;
	// PARAMS: Object Extracction
	double minObjDistToPlane = maxDistToPlane; 
	double maxObjDistToPlane = 0.25; 
	double minDistToContour = 0.02; 
	double maxDistBetweenObjs = 0.05; 

	// For removing floor and far far away points 
	cv::Mat validPointCloud;
	cv::inRange(imaXYZ, cv::Scalar(-1, -1.0, floorDistRemoval), cv::Scalar(1.5, 1.0, 2.0), validPointCloud); 
	
	// Getting Normals 	
	cv::Mat pointCloudBlur;
	cv::blur(imaXYZ, pointCloudBlur, cv::Size(blurSize, blurSize));
	cv::Mat normals = ObjExtractor::CalculateNormals( pointCloudBlur ); 

	// Getting Mask of Normals pointing horizonaliy
	cv::Mat  horizontalNormals;
	cv::inRange( normals, cv::Scalar(-1.0, -1.0, normalZThreshold), cv::Scalar(1.0,1.0, 1.0), horizontalNormals); 

	// Mask of horizontal normals and valid.  
	cv::Mat horizontalsValidPoints = horizontalNormals & validPointCloud; 

	//Getting horizontal planes.
	std::vector<PlanarSegment> horizontalPlanes = ObjExtractor::ExtractHorizontalPlanesRANSAC(imaXYZ, maxDistToPlane, maxIterations, minPointsForPlane, horizontalsValidPoints);
    if(horizontalPlanes.size() == 0)
        return false;

	// Getting Mask of objects of every plane
	std::vector< cv::Point3f > objectsPoints;
	std::vector< cv::Point2i > objectsIdx;
	for( int i=0; i<(int)horizontalPlanes.size(); i++){
		for(int row=0; row < imaXYZ.rows; row++){
			for(int col=0; col < imaXYZ.cols; col++){
				cv::Point3f xyzPoint = imaXYZ.at< cv::Vec3f >(row, col ); 
				double dist = horizontalPlanes[i].Get_Plane().DistanceToPoint(xyzPoint, true);
				if( minObjDistToPlane < dist && maxObjDistToPlane > dist ){
					double distToConvexHull = horizontalPlanes[i].IsInside( xyzPoint ); 
					if(distToConvexHull > minDistToContour){
                        objExtr.at<uchar>(row, col) = 255;
					}
				}
			}
		}
	}

	/*for(int j=0; j<(int)horizontalPlanes.size(); j++){
		std::vector <cv::Point2i> indexVec;
		indexVec = horizontalPlanes[j].Get_Indexes();
		for(int k=0; k<indexVec.size(); k++){
			objExtr.at<uchar>(indexVec[k].x, indexVec[k].y)=255;
		}

	}*/

    return true;
}


bool ObjExtractor::extractObjectsIncludingPlanes(cv::Mat& imaXYZ, cv::Mat& objExtr){
    objExtr = cv::Mat::zeros(imaXYZ.size(), CV_8UC1);

	// PARAMS: Valid Points 
	double floorDistRemoval = 0.25;
	// PARAMS: Normals Extraction
	int blurSize = 5; 
	double normalZThreshold = 0.8; 
	// PARAMS: Planes RANSAC 
	double maxDistToPlane = 0.02; 
	int maxIterations = 1000; 
	int minPointsForPlane = imaXYZ.rows * imaXYZ.cols*0.025;
	// PARAMS: Object Extracction
	double minObjDistToPlane = maxDistToPlane; 
	double maxObjDistToPlane = 0.25; 
	double minDistToContour = 0.02; 
	double maxDistBetweenObjs = 0.05; 

	// For removing floor and far far away points 
	cv::Mat validPointCloud;
	cv::inRange(imaXYZ, cv::Scalar(-1, -1.0, floorDistRemoval), cv::Scalar(1.5, 1.0, 2.0), validPointCloud); 
	
	// Getting Normals 	
	cv::Mat pointCloudBlur;
	cv::blur(imaXYZ, pointCloudBlur, cv::Size(blurSize, blurSize));
	cv::Mat normals = ObjExtractor::CalculateNormals( pointCloudBlur ); 

	// Getting Mask of Normals pointing horizonaliy
	cv::Mat  horizontalNormals;
	cv::inRange( normals, cv::Scalar(-1.0, -1.0, normalZThreshold), cv::Scalar(1.0,1.0, 1.0), horizontalNormals); 

	// Mask of horizontal normals and valid.  
	cv::Mat horizontalsValidPoints = horizontalNormals & validPointCloud; 

	//Getting horizontal planes.
	std::vector<PlanarSegment> horizontalPlanes = ObjExtractor::ExtractHorizontalPlanesRANSAC(imaXYZ, maxDistToPlane, maxIterations, minPointsForPlane, horizontalsValidPoints);
    if(horizontalPlanes.size() == 0)
        return false;

	// Getting Mask of objects of every plane
	std::vector< cv::Point3f > objectsPoints;
	std::vector< cv::Point2i > objectsIdx;
	for( int i=0; i<(int)horizontalPlanes.size(); i++){
		std::vector <cv::Point2i> indexVec;
		indexVec = horizontalPlanes[i].Get_Indexes();
        float zmean = 0;
		for(int k=0; k<indexVec.size(); k++){
            zmean += imaXYZ.at<cv::Point3f>(indexVec[k].y, indexVec[k].x).z;
        }
        zmean /= indexVec.size();
		for(int row=0; row < imaXYZ.rows; row++){
			for(int col=0; col < imaXYZ.cols; col++){
				cv::Point3f xyzPoint = imaXYZ.at< cv::Vec3f >(row, col ); 
                double dist = horizontalPlanes[i].Get_Plane().DistanceToPoint(xyzPoint, true);
				if(xyzPoint.z >= zmean - 0.02){
				    if( maxObjDistToPlane > dist ){
                        double distToConvexHull = horizontalPlanes[i].IsInside( xyzPoint ); 
					    if(distToConvexHull > minDistToContour)
                            objExtr.at<uchar>(row, col) = 255;
                    }
                }
                
                
				// if( minObjDistToPlane < dist && maxObjDistToPlane > dist ){
				//if( maxObjDistToPlane > dist ){
					// double distToConvexHull = horizontalPlanes[i].IsInside( xyzPoint ); 
					// if(distToConvexHull > minDistToContour){
                        //objExtr.at<uchar>(row, col) = 255;
					// }
				// }
			}
		}
	}

	for(int j=0; j<(int)horizontalPlanes.size(); j++){
		std::vector <cv::Point2i> indexVec;
		indexVec = horizontalPlanes[j].Get_Indexes();
		for(int k=0; k<indexVec.size(); k++){
			objExtr.at<uchar>(indexVec[k].y, indexVec[k].x)=255;
		}

	}

    return true;
}
