#include "ObjectsExtractor.hpp"

std::vector<DetectedObject> ObjectsExtractor::ExtractObjectsHorizantalPlanes( cv::Mat rgbImage, cv::Mat pointsCloud, cv::Mat& outObjectsMask)
{
	//
	//************I M P O R T A N T **************
	//In the Mr Jesus' original implementation, transformation of the point cloud from kinect to robot frame was done HERE.
	//In this implementation, it is assumed that the point cloud is already with respect to the robots frame
	//Transformation is expected to be done in another class or another part of the code using tf from ros.
	//
	//There is an overload method for the stand_alone_trainer
	outObjectsMask = cv::Mat::zeros(pointsCloud.rows, pointsCloud.cols, CV_8UC1); 

	// Getting Normals
	bool normalsBlur = true; 
	int normalsBlurSize = 7; 
	float normalsZThreshold = 0.8f;
	
	// Plane segmentation RANSAC
	int minPointsForPlane = 40000; 
	double distThresholdForPlane =  0.02; 
	int maxIterations = 1000;

	// Object Segmentation 
	double objMaxDistThres = 0.25; 
	double objMinDistThres = distThresholdForPlane; 
	double minDistToContour = 0.02; 
	double minDistBetweenObj = 0.100; 
	

	// /// ALGORITHM 
	// if(false){
	// 	cv::imshow( "pointsCloud", pointsCloud);
	// }

	// // Converting Kinect Points into Robot Points for horizontal extracting
	// Transform::Kinect2Robot( pointsCloud, pan, tilt, headZ ); 
	// if(false) {
	// 	cv::imshow( "pointsCloudTransf", pointsCloud);
	// }
		
	// Removing floor and far away points
	cv::Mat validPoints, notValidPoints;
	cv::inRange( pointsCloud , cv::Scalar( -2, -2, 0.10) , cv::Scalar( 2, 2, 1.5 ), validPoints); 
	cv::bitwise_not( validPoints, notValidPoints ); 
	if(false){
		cv::imshow( "validPoints", validPoints ); 
		cv::imshow( "notValidPoints", notValidPoints ); 
	}
	pointsCloud.setTo( 0.0, notValidPoints); 


	// Calculating Normals
	cv::Mat normals;
	cv::Mat pointsCloudForNormals = normals; 
	if( normalsBlur ){
		cv::blur( pointsCloud, pointsCloudForNormals, cv::Size(normalsBlurSize,normalsBlurSize));
	}
	GetNormalsCross( pointsCloudForNormals , normals); 

	//if( this->Debug )
	//	cv::imshow( "normals", normals); 
	
	// Getting horizontal normals Mask only
	cv::Mat horNormalsMask; 
	cv::inRange( normals, cv::Scalar( -1, -1, normalsZThreshold ), cv::Scalar( 1, 1, 1 ), horNormalsMask); 				
	/*cv::erode( horNormalsMask, horNormalsMask, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3,3) ) ); */
	
	//if( this->Debug)
	//	cv::imshow( "horNormalsMask", horNormalsMask); 	

	// Getting Horizontals Planes Segments
	std::vector<PlanarHorizontalSegment> horizontalPlanes;

	cv::Mat horNorMat; 
	std::vector< cv::Point2i > horNormalsIndexes;
	if( cv::countNonZero(horNormalsMask) > 0 )
	{
		cv::findNonZero( horNormalsMask , horNormalsIndexes );
	}
	else{
		return std::vector<DetectedObject>(); 
	}
	
	cv::Mat planesMask; 	
	horizontalPlanes = GetPlanesRANSAC_2( pointsCloud, horNormalsIndexes, minPointsForPlane, distThresholdForPlane, maxIterations, planesMask); 
	if( this->Debug ){
		cv::Mat rgbPlanesMasked; 
		
		cv::Mat rgbPlanes; 
		rgbImage.copyTo( rgbPlanesMasked, planesMask); 
		//cv::imshow("rgbPlanesMasked", rgbPlanesMasked); 

		cv::Mat notPlanesMask; 
		cv::bitwise_not( planesMask, notPlanesMask ); 
		rgbImage.copyTo( rgbPlanes, notPlanesMask ); 
		cv::imshow("rgbPlanes", rgbPlanes); 
	}
	
	
	//horizontalPlanes = GetPlanesRANSAC( pointsCloud, horNormalsMask, minPointsForPlane, distThresholdForPlane, maxIterations); 
	// Getting Objects form horizontal Segments.
	
	//cv::Mat bgrPlanesMask;
	//rgbImage.copyTo(bgrPlanesMask, horizontalPlanes);
	//cv::imshow("bgrPlanesMask", bgrPlanesMask);


	cv::vector< DetectedObject > detObjects; 
	cv::vector< cv::Mat > objectsInPlanesMask; 
	for( size_t plane=0 ; plane<horizontalPlanes.size() ; plane++ ){
		
		cv::Mat objectsByPlaneMask = cv::Mat::zeros(pointsCloud.rows, pointsCloud.cols, CV_8UC1);
		
		for( int row=0 ; row<pointsCloud.rows ; row++ ){
			for( int col=0 ; col<pointsCloud.cols ; col++ ){
				cv::Point3f ptXYZ = pointsCloud.at<cv::Vec3f>(row,col);
				
				double distToPlane = horizontalPlanes[plane].Get_Plane().DistanceToPoint( ptXYZ, true );				
				if( distToPlane > objMinDistThres && distToPlane < objMaxDistThres ){
					double distToContour = horizontalPlanes[plane].IsInside(ptXYZ); 
					
					if( distToContour > minDistToContour){
						outObjectsMask.at<uchar>(row,col) = 255;
						objectsByPlaneMask.at<uchar>(row, col) = 255; 
					}
				}
			}
		}
		
		std::vector< cv::Mat > objMasks; 
		std::vector< std::vector< cv::Point2i > > objIndexes; 
		ConnectedComponents3D(pointsCloud, objectsByPlaneMask, minDistBetweenObj, cv::Mat(), objIndexes, objMasks); 

		for( int i=0; i<objIndexes.size(); i++){
			if( objIndexes[i].size() < 1000 )
				continue; 
	
			DetectedObject dObj(objIndexes[i], pointsCloud, objMasks[i], horizontalPlanes[plane]); 			
			if( dObj.shadowOriBoundBoxt2D.size.height< 0.01 || dObj.shadowOriBoundBoxt2D.size.width < 0.01 )
				continue; 
			/*if( dObj.shadowOriBoundBoxt2D.size.height> 0.20 || dObj.shadowOriBoundBoxt2D.size.width > 0.20 )
				continue; */

			detObjects.push_back( dObj ); 
		}
	}

	if( this->Debug )
	{
		cv::Mat objectsSegmentation = cv::Mat::zeros(pointsCloud.rows, pointsCloud.cols, CV_8UC3); 
		for( int i=0; i<detObjects.size(); i++)
		{
			cv::Scalar color = cv::Scalar( rand() % 256 , rand() % 256 , rand() % 256 );
			objectsSegmentation.setTo( color, detObjects[i].oriMask ); 
		}

		//cv::imshow( "objectsSegmentation", objectsSegmentation); 
	}

	//cv::imshow( "outObjectsMask", outObjectsMask); 
	cv::waitKey(10); 

	//std::vector< cv::Mat > objMasks = ConnectedComponents3D( pointsCloud, outObjectsMask, 0.01, cv::Mat());
	
	return detObjects; 
}

std::vector<DetectedObject> ObjectsExtractor::ExtractObjectsHorizantalPlanes( cv::Mat rgbImage, cv::Mat pointsCloud, cv::Mat& outObjectsMask, float headPan, float headTilt, float headZ)
{
	// Converting Kinect Points into Robot Points for horizontal extracting
	Transform::Kinect2Robot( pointsCloud, headPan, headTilt, headZ ); 
	
	return this->ExtractObjectsHorizantalPlanes(rgbImage, pointsCloud, outObjectsMask);
}

void ObjectsExtractor::GetNormalsCross(cv::Mat& xyzPoints, cv::Mat& normals){
	
	normals = cv::Mat::zeros(xyzPoints.rows, xyzPoints.cols, CV_32FC3); 
	
	cv::Point3f topLeft; 
	cv::Point3f topRight; 
	cv::Point3f downLeft; 
	cv::Point3f downRight; 

	cv::Point3f normal_1; 
	cv::Point3f normal_2; 
	cv::Point3f normal; 
			
	cv::Point3f center;
	
	float norm = 0.0;

	for( int idxRow = 1 ; idxRow < xyzPoints.rows - 1 ; idxRow++ )
	{
		for( int idxCol = 1 ; idxCol < xyzPoints.cols - 1 ; idxCol++ )
		{			
			// Getting Vectors
			topLeft = xyzPoints.at<cv::Vec3f>(idxRow-1, idxCol-1);
			topRight = xyzPoints.at<cv::Vec3f>(idxRow-1, idxCol+1);
			downLeft = xyzPoints.at<cv::Vec3f>(idxRow+1, idxCol-1);
			downRight = xyzPoints.at<cv::Vec3f>(idxRow+1, idxCol+1);
			
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

			// Proyecting all normals over XY plane
			if(  normal.z < 0 )
				normal = -normal;

			// Make normal unitary
			norm = sqrt(normal.x*normal.x + normal.y*normal.y + normal.z*normal.z);
						
			if(norm == 0.0f)
				continue; 

			// Asignign Value To normal
			normals.at<cv::Vec3f>(idxRow, idxCol) = ( 1.0f / norm ) * normal;
		}
	}
}

std::vector<PlanarHorizontalSegment> ObjectsExtractor::GetPlanesRANSAC_2(cv::Mat pointCloud, std::vector<cv::Point2i> indexes, int minPointsForPlane, double distThresholdForPlane, int maxIterations, cv::Mat& out_planesMask){
	
	float distToPlane; 
	cv::Point3f xyzPoint; 
	double dist1;
	double dist2;
	double dist3;

	std::vector<PlanarHorizontalSegment> horizontalPlanes; 
	std::vector< std::vector< cv::Point2i > > indexesPlanes; 

	out_planesMask = cv::Mat::zeros( pointCloud.rows, pointCloud.cols, CV_8UC1); 

	int iterationsCnt = 0; 
	while( iterationsCnt++ < maxIterations && indexes.size() > minPointsForPlane ){
		
		// Getting Random points 
		cv::Point3f p1 = pointCloud.at<cv::Vec3f>( indexes[rand() % indexes.size() ] );  
		cv::Point3f p2 = pointCloud.at<cv::Vec3f>( indexes[rand() % indexes.size() ] ); 
		cv::Point3f p3 = pointCloud.at<cv::Vec3f>( indexes[rand() % indexes.size() ] ); 

		// Checking that are valid points for plane, ej. not paralls
		if( !Plane3D::AreValidPointsForPlane( p1, p2, p3 ) )
			continue;
		
		// Checking not so close points
		dist1 = cv::norm(p1 - p2);
		dist2 = cv::norm(p1 - p3); 
		dist3 = cv::norm(p2 - p3); 
		if(dist1 < 0.2 || dist2 < 0.2 || dist3 < 0.2 )
			continue; 

		// Calculating candidate plane
		Plane3D candidatePlane( p1, p2, p3 ); 

		if( std::abs( candidatePlane.GetNormal().z ) < 0.99 ){			
			continue; 
		}

		// Checking distance to candidate plane of points
		std::vector< cv::Point3f > inliers; 
		inliers.reserve( indexes.size() ); 

		for(size_t i=0; i<indexes.size(); i++ ){
			
			xyzPoint = pointCloud.at<cv::Vec3f>(indexes[i]); 
			distToPlane = candidatePlane.DistanceToPoint( xyzPoint );
			
			if( distToPlane < distThresholdForPlane )
				inliers.push_back( xyzPoint ); 
		}

		// If there are few inliers discard
		if( inliers.size() < minPointsForPlane )
			continue; 

		// Getting a better plane using PCA analisys
		cv::PCA pca( cv::Mat(inliers).reshape(1), cv::Mat(), CV_PCA_DATA_AS_ROW);

		// Creating new plane with a normal(eigenvector with lowest eigenvalue) and a point (mean)
		cv::Point3f pcaNormal(pca.eigenvectors.at<float>(2,0), pca.eigenvectors.at<float>(2,1), pca.eigenvectors.at<float>(2,2)); 
		cv::Point3f pcaPoint(pca.mean.at<float>(0,0), pca.mean.at<float>(0,1), pca.mean.at<float>(0,2)); 		
		
		Plane3D refinedPlane(pcaNormal, pcaPoint); 

		// Checking for new inliers
		
		// this is for not removing elemnts from list due speed. memory vs speed
		std::vector< cv::Point2i > newIndexes; 
		newIndexes.reserve( indexes.size() );
		// this is for not removing elemnts from list due speed. memory vs speed
		std::vector< cv::Point3f > newInliers; 
		newInliers.reserve( indexes.size() ); 
		// this is for not removing elemnts from list due speed. memory vs speed
		std::vector< cv::Point2f > pointsXY_forConvexHull; 
		pointsXY_forConvexHull.reserve( indexes.size() ); 
		// this is for not removing elemnts from list due speed. memory vs speed
		std::vector< cv::Point2i > indexPlane; 
		indexPlane.reserve( indexes.size() ); 

		for(size_t i=0; i<indexes.size(); i++ ){
			
			xyzPoint = pointCloud.at<cv::Vec3f>(indexes[i]); 
			distToPlane = refinedPlane.DistanceToPoint( xyzPoint );

			if( distToPlane < distThresholdForPlane ){
				newInliers.push_back( xyzPoint ); 
				pointsXY_forConvexHull.push_back( cv::Point2f( xyzPoint.x , xyzPoint.y ) );
				indexPlane.push_back( indexes[i] );

				out_planesMask.at<uchar>( indexes[i] ) = 255; 
			}
			else
				newIndexes.push_back(indexes[i]);
		}

		indexes = newIndexes; 
		indexesPlanes.push_back( indexPlane ); 

		// Getting Convex Hull ( Convex hull is valid if remove Z, because is an horizontal plane ) 
		std::vector< cv::Point2f > convexHull2D; 
		cv::convexHull( pointsXY_forConvexHull , convexHull2D); 		
		// Creating Horizontal Planar Segment
		PlanarHorizontalSegment ps( newInliers, refinedPlane, pca, convexHull2D ); 
		// Adding to vector to return
		horizontalPlanes.push_back( ps );
	}

	if( this->Debug ){
		cv::Mat planesColor = cv::Mat::zeros( pointCloud.rows , pointCloud.cols , CV_8UC3 ); 
		for( int i=0 ; i<indexesPlanes.size() ; i++ ){
			cv::Scalar color = cv::Scalar( rand() % 256 , rand() % 256 , rand() % 256 ); 
			for( int j=0 ; j<indexesPlanes[i].size() ; j++){
				planesColor.at<cv::Vec3b>( indexesPlanes[i][j] )[0] = color[0]; 
				planesColor.at<cv::Vec3b>( indexesPlanes[i][j] )[1] = color[1]; 
				planesColor.at<cv::Vec3b>( indexesPlanes[i][j] )[2] = color[2]; 
			}
		}
		//cv::imshow("planesColor", planesColor); 
	}

	return horizontalPlanes; 
}

std::vector<PlanarHorizontalSegment> ObjectsExtractor::GetPlanesRANSAC(cv::Mat pointCloud, cv::Mat mask , int minPointsForPlane, double distThresholdForPlane, int maxIterations){
	
	std::vector< cv::Point2i > indexes; 
	if( mask.data && mask.isContinuous() ){
		cv::findNonZero( mask, indexes ); 
	}

	float distToPlane; 
	cv::Point3f xyzPoint; 
	double dist1;
	double dist2;
	double dist3;

	double distThreshBetweenPoints = 0.1; 

	std::vector<PlanarHorizontalSegment> detectedPlanes; 
	std::vector< std::vector< cv::Point2i > > indexesPlanes; 
	
	cv::Mat labels = cv::Mat::zeros( pointCloud.rows, pointCloud.cols, CV_32FC1 ); 
	float actLabel = 0;

	int iterationsCnt = 0; 
	while( iterationsCnt++ < maxIterations && indexes.size() > minPointsForPlane ){
		
		/////////////////////////////
		// Getting Candidate Plane //
		/////////////////////////////

		// Getting Random points 
		cv::Point3f p1 = pointCloud.at<cv::Vec3f>( indexes[rand() % indexes.size() ] );  
		cv::Point3f p2 = pointCloud.at<cv::Vec3f>( indexes[rand() % indexes.size() ] ); 
		cv::Point3f p3 = pointCloud.at<cv::Vec3f>( indexes[rand() % indexes.size() ] ); 

		// Checking that are valid points for plane, ej. not paralls
		if( !Plane3D::AreValidPointsForPlane( p1, p2, p3 ) )
			continue;
		
		// Checking not so close points
		dist1 = cv::norm(p1 - p2);
		dist2 = cv::norm(p1 - p3); 
		dist3 = cv::norm(p2 - p3); 
		if(dist1 < 0.2 || dist2 < 0.2 || dist3 < 0.2 )
			continue; 

		// Calculating candidate plane
		Plane3D candidatePlane( p1, p2, p3 ); 

		if( std::abs( candidatePlane.GetNormal().z ) < 0.99 ){			
			continue; 
		}
		
		// Checking distance to candidate plane of points
		std::vector< cv::Point3f > inliers; 
		inliers.reserve( indexes.size() ); 

		for(size_t i=0; i<indexes.size(); i++ ){
			xyzPoint = pointCloud.at<cv::Vec3f>(indexes[i]); 
			distToPlane = candidatePlane.DistanceToPoint( xyzPoint );
			if( distToPlane < distThresholdForPlane )
				inliers.push_back( xyzPoint ); 
		}

		// If there are few inliers discard
		if( inliers.size() < minPointsForPlane )
			continue; 

		///////////////////////////
		// Getting Refined Plane //
		///////////////////////////

		// Getting a better plane using PCA analisys
		cv::PCA pca( cv::Mat(inliers).reshape(1), cv::Mat(), CV_PCA_DATA_AS_ROW);
		cv::Point3f pcaNormal(pca.eigenvectors.at<float>(2,0), pca.eigenvectors.at<float>(2,1), pca.eigenvectors.at<float>(2,2)); 
		cv::Point3f pcaPoint(pca.mean.at<float>(0,0), pca.mean.at<float>(0,1), pca.mean.at<float>(0,2)); 		
		
		// Creating new plane with a normal(eigenvector with lowest eigenvalue) and a point (mean)
		Plane3D refinedPlane(pcaNormal, pcaPoint); 

		////////////////////////////// 
		// Getting Planner Segments //
		//////////////////////////////
		
		// this is for not removing elemnts from list due speed. memory vs speed
		std::vector< cv::Point2i > newIndexes; 
		newIndexes.reserve( indexes.size() );
				
		
		for( size_t i=0 ; i<indexes.size() ; i++ ){
			
			if( labels.at<float>( indexes[i] ) != 0 ){
				continue; 
			}

			xyzPoint = pointCloud.at<cv::Vec3f>( indexes[i] ); 
			if( refinedPlane.DistanceToPoint(xyzPoint) > distThresholdForPlane ){
				newIndexes.push_back( indexes[i] );
				continue; 
			}
		
			std::vector< cv::Point2i> planeIndexes; 
			
			actLabel += 1.0f; 
			labels.at<float>( indexes[i] ) = actLabel; 

			std::vector< cv::Point2i > neighboursQueue; 			
			neighboursQueue.push_back( indexes[i] ); 
			
			while( neighboursQueue.size() > 0 ){
				
				cv::Point2i actualPixel = neighboursQueue.back(); 
				neighboursQueue.pop_back(); 
				planeIndexes.push_back(actualPixel); 
				
				cv::Point3f actualPoint = pointCloud.at<cv::Vec3f>( actualPixel );

				// Neighbours
				cv::Point2i up    = actualPixel + cv::Point2i( 1 , 0); 
				cv::Point2i down  = actualPixel + cv::Point2i(-1 , 0); 
				cv::Point2i left  = actualPixel + cv::Point2i( 0 ,-1); 
				cv::Point2i right = actualPixel + cv::Point2i( 0 ,+1); 
				
				cv::Point2i upLeft    = actualPixel + cv::Point2i( +1,-1); 
				cv::Point2i upRight   = actualPixel + cv::Point2i( +1,+1); 
				cv::Point2i downLeft  = actualPixel + cv::Point2i( -1,-1); 
				cv::Point2i downRight = actualPixel + cv::Point2i( -1,+1); 

				cv::Point2i neighPixel; 
				cv::Point3f neighPoint; 

				bool isInMask; 
				bool isNotLabeled; 
				bool isNearPlane; 
				bool isNearPoint; 

				// Checking for Up
				neighPixel = up;
				neighPoint = pointCloud.at<cv::Vec3f>( neighPixel );
				isInMask = mask.at<uchar>(neighPixel) == 255 ;  // has normal up
				isNotLabeled = labels.at<float>(neighPixel) == 0.0 ;
				isNearPlane = refinedPlane.DistanceToPoint( neighPoint ) < distThresholdForPlane; 
				isNearPoint = cv::norm( neighPoint - actualPoint ) < distThreshBetweenPoints; 
				if( isInMask && isNotLabeled && isNearPlane && isNearPoint ){
					labels.at<float>( neighPixel ) = actLabel; 	
					neighboursQueue.push_back( neighPixel ); 
				}


				// Checking for down
				neighPixel = down; 
				neighPoint = pointCloud.at<cv::Vec3f>( neighPixel );
				isInMask = mask.at<uchar>(neighPixel) == 255 ;  // has normal up
				isNotLabeled = labels.at<float>(neighPixel) == 0.0 ;
				isNearPlane = refinedPlane.DistanceToPoint( neighPoint ) < distThresholdForPlane; 
				isNearPoint = cv::norm( neighPoint - actualPoint ) < distThreshBetweenPoints; 
				if( isInMask && isNotLabeled && isNearPlane && isNearPoint ){
					labels.at<float>( neighPixel ) = actLabel; 	
					neighboursQueue.push_back( neighPixel ); 
				}
				
				// Checking for Left
				neighPixel = left; 
				neighPoint = pointCloud.at<cv::Vec3f>( neighPixel );
				isInMask = mask.at<uchar>(neighPixel) == 255 ;  // has normal up
				isNotLabeled = labels.at<float>(neighPixel) == 0.0 ;
				isNearPlane = refinedPlane.DistanceToPoint( neighPoint ) < distThresholdForPlane; 
				isNearPoint = cv::norm( neighPoint - actualPoint ) < distThreshBetweenPoints; 
				if( isInMask && isNotLabeled && isNearPlane && isNearPoint ){
					labels.at<float>( neighPixel ) = actLabel; 	
					neighboursQueue.push_back( neighPixel ); 
				}

				// Checking for Right
				neighPixel = right; 
				neighPoint = pointCloud.at<cv::Vec3f>( neighPixel );
				isInMask = mask.at<uchar>(neighPixel) == 255 ;  // has normal up
				isNotLabeled = labels.at<float>(neighPixel) == 0.0 ;
				isNearPlane = refinedPlane.DistanceToPoint( neighPoint ) < distThresholdForPlane; 
				isNearPoint = cv::norm( neighPoint - actualPoint ) < distThreshBetweenPoints; 
				if( isInMask && isNotLabeled && isNearPlane && isNearPoint ){
					labels.at<float>( neighPixel ) = actLabel; 	
					neighboursQueue.push_back( neighPixel ); 
				}

				// Checking for upLeft
				neighPixel = upLeft; 
				neighPoint = pointCloud.at<cv::Vec3f>( neighPixel );
				isInMask = mask.at<uchar>(neighPixel) == 255 ;  // has normal up
				isNotLabeled = labels.at<float>(neighPixel) == 0.0 ;
				isNearPlane = refinedPlane.DistanceToPoint( neighPoint ) < distThresholdForPlane; 
				isNearPoint = cv::norm( neighPoint - actualPoint ) < distThreshBetweenPoints; 
				if( isInMask && isNotLabeled && isNearPlane && isNearPoint ){
					labels.at<float>( neighPixel ) = actLabel; 	
					neighboursQueue.push_back( neighPixel ); 
				}

				// Checking for upRight
				neighPixel = upRight; 
				neighPoint = pointCloud.at<cv::Vec3f>( neighPixel );
				isInMask = mask.at<uchar>(neighPixel) == 255 ;  // has normal up
				isNotLabeled = labels.at<float>(neighPixel) == 0.0 ;
				isNearPlane = refinedPlane.DistanceToPoint( neighPoint ) < distThresholdForPlane; 
				isNearPoint = cv::norm( neighPoint - actualPoint ) < distThreshBetweenPoints; 
				if( isInMask && isNotLabeled && isNearPlane && isNearPoint ){
					labels.at<float>( neighPixel ) = actLabel; 	
					neighboursQueue.push_back( neighPixel ); 
				}

				// Checking for downLeft
				neighPixel = downLeft; 
				neighPoint = pointCloud.at<cv::Vec3f>( neighPixel );
				isInMask = mask.at<uchar>(neighPixel) == 255 ;  // has normal up
				isNotLabeled = labels.at<float>(neighPixel) == 0.0 ;
				isNearPlane = refinedPlane.DistanceToPoint( neighPoint ) < distThresholdForPlane; 
				isNearPoint = cv::norm( neighPoint - actualPoint ) < distThreshBetweenPoints; 
				if( isInMask && isNotLabeled && isNearPlane && isNearPoint ){
					labels.at<float>( neighPixel ) = actLabel; 	
					neighboursQueue.push_back( neighPixel ); 
				}

				// Checking for downRight
				neighPixel = downRight; 
				neighPoint = pointCloud.at<cv::Vec3f>( neighPixel );
				isInMask = mask.at<uchar>(neighPixel) == 255 ;  // has normal up
				isNotLabeled = labels.at<float>(neighPixel) == 0.0 ;
				isNearPlane = refinedPlane.DistanceToPoint( neighPoint ) < distThresholdForPlane; 
				isNearPoint = cv::norm( neighPoint - actualPoint ) < distThreshBetweenPoints; 
				if( isInMask && isNotLabeled && isNearPlane && isNearPoint ){
					labels.at<float>( neighPixel ) = actLabel; 	
					neighboursQueue.push_back( neighPixel ); 
				}

			}
			
			indexesPlanes.push_back( planeIndexes ); 
		}
		indexes = newIndexes ; 
	}

	if( this->Debug ){
		cv::Mat planesColor = cv::Mat::zeros( pointCloud.rows , pointCloud.cols , CV_8UC3 ); 
		for( int i=0 ; i<indexesPlanes.size() ; i++ ){
			cv::Scalar color = cv::Scalar( rand() % 256 , rand() % 256 , rand() % 256 ); 
			for( int j=0 ; j<indexesPlanes[i].size() ; j++){
				planesColor.at<cv::Vec3b>( indexesPlanes[i][j] )[0] = color[0]; 
				planesColor.at<cv::Vec3b>( indexesPlanes[i][j] )[1] = color[1]; 
				planesColor.at<cv::Vec3b>( indexesPlanes[i][j] )[2] = color[2]; 
			}
		}
		cv::imshow("planesColor", planesColor); 
	}

	return detectedPlanes; 
}

void ObjectsExtractor::ConnectedComponents3D(cv::Mat pointCloud, cv::Mat mask, double minDist, cv::Mat labelsMask, std::vector<std::vector<cv::Point2i> >& objIndexes, std::vector<cv::Mat>& objectsMask){
	
	labelsMask = cv::Mat::zeros(pointCloud.rows, pointCloud.cols, CV_32FC1); 
	
	float currentLabel = 0.0f; 
	for( int i=1 ; i < pointCloud.rows-1 ; i++){
		for( int j=1 ; j < pointCloud.cols-1 ; j++){

			if( mask.at<uchar>( i,j ) == 0 ) 
				continue; 

			if( labelsMask.at<float>( i,j ) != 0.0f )
				continue; 

			std::queue< cv::Point2i > neighboursQueue; 
			std::vector< cv::Point2i > indexes; 
			cv::Mat objMask = cv::Mat::zeros(pointCloud.rows, pointCloud.cols, CV_8UC1); 

			currentLabel += 1.0; 
			labelsMask.at<float>( i,j ) = currentLabel; 
			neighboursQueue.push( cv::Point2i(j,i) ); 
			indexes.push_back( cv::Point2i(j,i) ); 

			objMask.at<uchar>(i,j) = 255; 

			while( neighboursQueue.size() > 0 ){
				
				cv::Point2i currentPixel = neighboursQueue.front(); 
				neighboursQueue.pop(); 

				cv::Point3f currentPoint = pointCloud.at< cv::Vec3f >( currentPixel ); 			

				// Neighbours			
				cv::Point2i up    = currentPixel + cv::Point2i( 1 , 0); 
				cv::Point2i down  = currentPixel + cv::Point2i(-1 , 0); 
				cv::Point2i left  = currentPixel + cv::Point2i( 0 ,-1); 
				cv::Point2i right = currentPixel + cv::Point2i( 0 ,+1); 
				
				// Checking for Neighbours
				cv::Point2i neighPixel; 
				cv::Point3f neighPoint; 

				bool isInMask = false; 
				bool isNotLabeled = false; 
				bool isNearPoint = false;
				
				// Checking for up
				neighPixel = up;
				if( neighPixel.x>=0 && neighPixel.x<pointCloud.cols && neighPixel.y>=0 && neighPixel.y<pointCloud.rows ) {
					neighPoint = pointCloud.at<cv::Vec3f>( neighPixel );
					isInMask = mask.at<uchar>(neighPixel) != 0 ;  // has normal up
					isNotLabeled = labelsMask.at<float>(neighPixel) == 0.0 ;
					isNearPoint = cv::norm( neighPoint - currentPoint ) < minDist; 
					if( isInMask && isNearPoint )
					{
						if( isNotLabeled ) 
						{
							labelsMask.at<float>( neighPixel ) = currentLabel;
							neighboursQueue.push( neighPixel );
							indexes.push_back( neighPixel ); 
							objMask.at<uchar>(neighPixel) = 255; 
						}
						else
						{
							float neighLabel = labelsMask.at<float>( neighPixel ); 
							if( neighLabel < currentLabel )
							{
								labelsMask.at<float>( neighPixel ) = neighLabel; 	
								neighboursQueue.push( neighPixel );
								indexes.push_back( neighPixel ); 
								objMask.at<uchar>(neighPixel) = 255; 
							}
						}
					}
				}

				// Checking for down
				neighPixel = down;
				if( neighPixel.x>=0 && neighPixel.x<pointCloud.cols && neighPixel.y>=0 && neighPixel.y<pointCloud.rows ) {
					neighPoint = pointCloud.at<cv::Vec3f>( neighPixel );
					isInMask = mask.at<uchar>(neighPixel) != 0 ;  // has normal up
					isNotLabeled = labelsMask.at<float>(neighPixel) == 0.0 ;
					isNearPoint = cv::norm( neighPoint - currentPoint ) < minDist; 
					if( isInMask && isNearPoint )
					{
						if( isNotLabeled ) 
						{
							labelsMask.at<float>( neighPixel ) = currentLabel;
							neighboursQueue.push( neighPixel );
							indexes.push_back( neighPixel ); 
							objMask.at<uchar>(neighPixel) = 255; 
						}
						else
						{
							float neighLabel = labelsMask.at<float>( neighPixel ); 
							if( neighLabel < currentLabel )
							{
								labelsMask.at<float>( neighPixel ) = neighLabel; 	
								neighboursQueue.push( neighPixel );
								indexes.push_back( neighPixel ); 
								objMask.at<uchar>(neighPixel) = 255; 
							}
						}
					}
				}

				// Checking for left
				neighPixel = left;
				if( neighPixel.x>=0 && neighPixel.x<pointCloud.cols && neighPixel.y>=0 && neighPixel.y<pointCloud.rows ) {
					neighPoint = pointCloud.at<cv::Vec3f>( neighPixel );
					isInMask = mask.at<uchar>(neighPixel) != 0 ;  // has normal up
					isNotLabeled = labelsMask.at<float>(neighPixel) == 0.0 ;
					isNearPoint = cv::norm( neighPoint - currentPoint ) < minDist; 
					if( isInMask && isNearPoint )
					{
						if( isNotLabeled ) 
						{
							labelsMask.at<float>( neighPixel ) = currentLabel;
							neighboursQueue.push( neighPixel );
							indexes.push_back( neighPixel ); 
							objMask.at<uchar>(neighPixel) = 255; 
						}
						else
						{
							float neighLabel = labelsMask.at<float>( neighPixel ); 
							if( neighLabel < currentLabel )
							{
								labelsMask.at<float>( neighPixel ) = neighLabel; 	
								neighboursQueue.push( neighPixel );
								indexes.push_back( neighPixel ); 
								objMask.at<uchar>(neighPixel) = 255; 
							}
						}
					}
				}

				// Checking for right
				neighPixel = right;
				if( neighPixel.x>=0 && neighPixel.x<pointCloud.cols && neighPixel.y>=0 && neighPixel.y<pointCloud.rows ) {
					neighPoint = pointCloud.at<cv::Vec3f>( neighPixel );
					isInMask = mask.at<uchar>(neighPixel) != 0 ;  // has normal up
					isNotLabeled = labelsMask.at<float>(neighPixel) == 0.0 ;
					isNearPoint = cv::norm( neighPoint - currentPoint ) < minDist; 
					if( isInMask && isNearPoint )
					{
						if( isNotLabeled ) 
						{
							labelsMask.at<float>( neighPixel ) = currentLabel;
							neighboursQueue.push( neighPixel );
							indexes.push_back( neighPixel ); 
							objMask.at<uchar>(neighPixel) = 255; 
						}
						else
						{
							float neighLabel = labelsMask.at<float>( neighPixel ); 
							if( neighLabel < currentLabel )
							{
								labelsMask.at<float>( neighPixel ) = neighLabel; 	
								neighboursQueue.push( neighPixel );
								indexes.push_back( neighPixel ); 
								objMask.at<uchar>(neighPixel) = 255; 
							}
						}
					}
				}

			}

			objectsMask.push_back( objMask );
			objIndexes.push_back( indexes ); 
		}
	}

	if( false  ){

		cv::Mat colorLabel = cv::Mat::zeros(pointCloud.rows, pointCloud.cols, CV_8UC3); 
		
		std::vector< cv::Vec3b > colors; 
		for( int i=0; i <= currentLabel; i++ ){
			colors.push_back( cv::Vec3b( rand() % 256 , rand() % 256 , rand() % 256 ) ); 
		}
		
		for( int i=0 ; i<pointCloud.rows ; i++){
			for( int j=0 ; j<pointCloud.cols ; j++){
				if( labelsMask.at<float>(i,j) != 0.0 )
					colorLabel.at<cv::Vec3b>(i,j) = colors[ labelsMask.at<float>(i,j) ] ; 		
			}
		}
		cv::imshow( "colorLabel", colorLabel); 
		cv::waitKey(20); 
	}
}

//void ObjectsExtractor::ConnectedComponents3D(cv::Mat pointCloud, cv::Mat mask, double minDist, cv::Mat labelsMask, std::vector<std::vector<cv::Point2i>>& objIndexes, std::vector<cv::Mat>& objectsMask){
//	
//	labelsMask = cv::Mat::zeros(pointCloud.rows, pointCloud.cols, CV_32FC1); 
//	
//	float currentLabel = 0.0f; 
//	for( int i=1 ; i < pointCloud.rows-1 ; i++){
//		for( int j=1 ; j < pointCloud.cols-1 ; j++){
//
//			if( mask.at<uchar>( i,j ) == 0 ) 
//				continue; 
//
//			if( labelsMask.at<float>( i,j ) != 0.0f )
//				continue; 
//
//			std::queue< cv::Point2i > neighboursQueue; 
//			std::vector< cv::Point2i > indexes; 
//			cv::Mat objMask = cv::Mat::zeros(pointCloud.rows, pointCloud.cols, CV_8UC1); 
//
//			currentLabel += 1.0; 
//			labelsMask.at<float>( i,j ) = currentLabel; 
//			neighboursQueue.push( cv::Point2i(j,i) ); 
//			indexes.push_back( cv::Point2i(j,i) ); 
//
//			objMask.at<uchar>(i,j) = 255; 
//
//			while( neighboursQueue.size() > 0 ){
//				
//				cv::Point2i currentPixel = neighboursQueue.front(); 
//				neighboursQueue.pop(); 
//
//				cv::Point3f currentPoint = pointCloud.at< cv::Vec3f >( currentPixel ); 			
//
//				// Neighbours			
//				cv::Point2i up    = currentPixel + cv::Point2i( 1 , 0); 
//				cv::Point2i down  = currentPixel + cv::Point2i(-1 , 0); 
//				cv::Point2i left  = currentPixel + cv::Point2i( 0 ,-1); 
//				cv::Point2i right = currentPixel + cv::Point2i( 0 ,+1); 
//				
//				// Checking for Neighbours
//				cv::Point2i neighPixel; 
//				cv::Point3f neighPoint; 
//
//				bool isInMask = false; 
//				bool isNotLabeled = false; 
//				bool isNearPoint = false;
//				
//				// Checking for up
//				neighPixel = up;
//				if( neighPixel.x>=0 && neighPixel.x<pointCloud.cols && neighPixel.y>=0 && neighPixel.y<pointCloud.rows ) {
//					neighPoint = pointCloud.at<cv::Vec3f>( neighPixel );
//					isInMask = mask.at<uchar>(neighPixel) != 0 ;  // has normal up
//					isNotLabeled = labelsMask.at<float>(neighPixel) == 0.0 ;
//					isNearPoint = cv::norm( neighPoint - currentPoint ) < minDist; 
//					if( isInMask && isNotLabeled && isNearPoint ){
//						labelsMask.at<float>( neighPixel ) = currentLabel; 	
//						neighboursQueue.push( neighPixel );
//						indexes.push_back( neighPixel ); 
//						objMask.at<uchar>(neighPixel) = 255; 
//					}
//				}
//
//				// Checking for down
//				neighPixel = down;
//				if( neighPixel.x>=0 && neighPixel.x<pointCloud.cols && neighPixel.y>=0 && neighPixel.y<pointCloud.rows ) {
//					neighPoint = pointCloud.at<cv::Vec3f>( neighPixel );
//					isInMask = mask.at<uchar>(neighPixel) != 0 ;  // has normal up
//					isNotLabeled = labelsMask.at<float>(neighPixel) == 0.0 ;
//					isNearPoint = cv::norm( neighPoint - currentPoint ) < minDist; 
//					if( isInMask && isNotLabeled && isNearPoint ){
//						labelsMask.at<float>( neighPixel ) = currentLabel; 	
//						neighboursQueue.push( neighPixel );
//						indexes.push_back( neighPixel ); 
//						objMask.at<uchar>(neighPixel) = 255; 
//					}
//				}
//
//				// Checking for left
//				neighPixel = left;
//				if( neighPixel.x>=0 && neighPixel.x<pointCloud.cols && neighPixel.y>=0 && neighPixel.y<pointCloud.rows ) {
//					neighPoint = pointCloud.at<cv::Vec3f>( neighPixel );
//					isInMask = mask.at<uchar>(neighPixel) != 0 ;  // has normal up
//					isNotLabeled = labelsMask.at<float>(neighPixel) == 0.0 ;
//					isNearPoint = cv::norm( neighPoint - currentPoint ) < minDist; 
//					if( isInMask && isNotLabeled && isNearPoint ){
//						labelsMask.at<float>( neighPixel ) = currentLabel; 	
//						neighboursQueue.push( neighPixel );
//						indexes.push_back( neighPixel ); 
//						objMask.at<uchar>(neighPixel) = 255; 
//					}
//				}
//
//				// Checking for right
//				neighPixel = right;
//				if( neighPixel.x>=0 && neighPixel.x<pointCloud.cols && neighPixel.y>=0 && neighPixel.y<pointCloud.rows ) {
//					neighPoint = pointCloud.at<cv::Vec3f>( neighPixel );
//					isInMask = mask.at<uchar>(neighPixel) != 0 ;  // has normal up
//					isNotLabeled = labelsMask.at<float>(neighPixel) == 0.0 ;
//					isNearPoint = cv::norm( neighPoint - currentPoint ) < minDist; 
//					if( isInMask && isNotLabeled && isNearPoint ){
//						labelsMask.at<float>( neighPixel ) = currentLabel; 	
//						neighboursQueue.push( neighPixel );
//						indexes.push_back( neighPixel ); 
//						objMask.at<uchar>(neighPixel) = 255; 
//					}
//				}
//
//			}
//
//			objectsMask.push_back( objMask );
//			objIndexes.push_back( indexes ); 
//		}
//	}
//
//	if( false  ){
//
//		cv::Mat colorLabel = cv::Mat::zeros(pointCloud.rows, pointCloud.cols, CV_8UC3); 
//		
//		std::vector< cv::Vec3b > colors; 
//		for( int i=0; i <= currentLabel; i++ ){
//			colors.push_back( cv::Vec3b( rand() % 256 , rand() % 256 , rand() % 256 ) ); 
//		}
//		
//		for( int i=0 ; i<pointCloud.rows ; i++){
//			for( int j=0 ; j<pointCloud.cols ; j++){
//				if( labelsMask.at<float>(i,j) != 0.0 )
//					colorLabel.at<cv::Vec3b>(i,j) = colors[ labelsMask.at<float>(i,j) ] ; 		
//			}
//		}
//		cv::imshow( "colorLabel", colorLabel); 
//		cv::waitKey(20); 
//	}
//}