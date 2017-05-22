#include "PersonExtractor.hpp"

PersonExtractor::PersonExtractor()
{	
	this->trainingDir = "PersonExtractor//"; 
	this->faceTrainingFile = "haarcascade_frontalface_alt_tree.xml"; 
	this->smileTrainingFile = "haarcascade_smile.xml"; 

	if( !this->faceClassifier.load( this->trainingDir + this->faceTrainingFile ) )
	{
		std::cout << "PersonExtractor: Cant load FACES Training File :(" << std::endl; 
	}
	this->debugMode = true; 
	
	this->minFaceSize = cv::Size(30, 30); 
	this->maxFaceSize = cv::Size(); 
	
	this->maxDistanceBackgroud = 3.0; 

	this->bodyFaceHeightRatio = 4.5; 
	this->bodyFaceWidthRatio = 3.0; 

	this->maxDistanceBodyPixel = 0.05; 
}

std::vector< cv::Point3f > PersonExtractor::DetectFaces( cv::Mat bgrImage, cv::Mat pointCloud, std::vector< cv::Rect > outFaceBoxes )
{
	std::vector< cv::Point3f > facesCentroids;

	// Getting distance Mask for first segmentation (far away points) 
	cv::Mat distanceMask = cv::Mat::zeros( pointCloud.rows, pointCloud.cols, CV_8UC1); 
	for(int i=0; i<pointCloud.rows; i++)
	{
		for(int j=0; j<pointCloud.cols; j++)
		{
			double dist = cv::norm( pointCloud.at<cv::Vec3f>(i,j) , cv::NORM_L2); 
			if( dist > 0.0 && dist < this->maxDistanceBackgroud )
				distanceMask.at<uchar>( i,j ) = 255; 
		}
	}

	// Preprocessing Image
	cv::Mat grayImage; 
	cv::cvtColor( bgrImage, grayImage, CV_BGR2GRAY ); 
	grayImage &= distanceMask; 
	cv::equalizeHist( grayImage, grayImage ); 

	// Getting Faces
	std::vector< cv::Rect > faceBoxes; 
	this->faceClassifier.detectMultiScale( grayImage, faceBoxes, 1.05, 6, 0, minFaceSize, maxFaceSize ); 
	
	// Getting centeroid of faces
	for(int i=0; i<faceBoxes.size(); i++)
	{
		cv::Point2i boxCenter = ( faceBoxes[i].tl() + faceBoxes[i].br() )*0.5;
		cv::Point3f faceCentroid = pointCloud.at<cv::Vec3f>( boxCenter ); 
		facesCentroids.push_back( faceCentroid ); 
	}

	outFaceBoxes = faceBoxes; 
	return facesCentroids; 
}

std::vector< cv::Mat > PersonExtractor::SegmentBody( cv::Mat bgrImage, cv::Mat pointCloud, std::vector< cv::Rect > &outBodyRects )
{
	std::vector< cv::Mat > bodyMasks; 
	
	cv::Mat bgrImageCopy; 
	if( this->debugMode )
		bgrImage.copyTo( bgrImageCopy ); 

	// Getting distance Mask for first segmentation (far away points) 
	cv::Mat distanceMask = cv::Mat::zeros( pointCloud.rows, pointCloud.cols, CV_8UC1); 
	for(int i=0; i<pointCloud.rows; i++)
	{
		for(int j=0; j<pointCloud.cols; j++)
		{
			double dist = cv::norm( pointCloud.at<cv::Vec3f>(i,j) , cv::NORM_L2); 
			if( dist > 0.0 && dist < this->maxDistanceBackgroud )
				distanceMask.at<uchar>( i,j ) = 255; 
		}
	}

	// Preprocessing Image
	cv::Mat grayImage; 
	cv::cvtColor( bgrImage, grayImage, CV_BGR2GRAY ); 
	grayImage &= distanceMask; 
	cv::equalizeHist( grayImage, grayImage ); 

	// Getting Faces
	std::vector< cv::Rect > faceBoxes; 
	this->faceClassifier.detectMultiScale( grayImage, faceBoxes, 1.05, 6, 0, minFaceSize, maxFaceSize ); 
	
	// Getting Bodies for every face:
	for(int i=0; i<faceBoxes.size(); i++)
	{
		int x, y, w, h;

		w = faceBoxes[i].width; 
		h = faceBoxes[i].height; 
		x = faceBoxes[i].x;
		y = faceBoxes[i].y * 1 + h; 
		cv::Rect chestBox = cv::Rect( x, y, w, h ); 
		cv::Point2i centerChestPixel = (chestBox.tl() + chestBox.br() ) * 0.5; 

		w = faceBoxes[i].width * this->bodyFaceWidthRatio; 
		h = faceBoxes[i].height * this->bodyFaceHeightRatio; 
		x = faceBoxes[i].x + faceBoxes[i].width/2 - w/2; 
		y = faceBoxes[i].y + faceBoxes[i].height;
		cv::Rect bodyBox = cv::Rect( x, y, w, h ); 
		cv::Mat bodyMask = cv::Mat::zeros( pointCloud.rows, pointCloud.cols, CV_8UC1 );
		cv::rectangle( bodyMask, bodyBox, cv::Scalar(255,255,255), -1 );

		// Segmenting Body 
		cv::Mat bodySegmentMask; 
		SegmentDistanceNeightbours( pointCloud, centerChestPixel, this->maxDistanceBodyPixel, bodyMask, bodySegmentMask); 

		bodyMasks.push_back( bodySegmentMask ); 
		outBodyRects.push_back( bodyBox ); 

		if( this->debugMode )
		{
			bgrImageCopy.setTo( cv::Scalar(200, 200, 0), bodySegmentMask); 
			cv::rectangle( bgrImageCopy, faceBoxes[i], cv::Scalar(255,0,0), 2); 
			cv::rectangle( bgrImageCopy, chestBox, cv::Scalar(0,255,0), 2); 
			cv::rectangle( bgrImageCopy, bodyBox, cv::Scalar(0,0,255), 2); 
			cv::circle( bgrImageCopy, centerChestPixel, 6, cv::Scalar(0,255,0), -1 ); 
		}
	}

	if( this->debugMode )
	{
		cv::imshow( "SegmentBody", bgrImageCopy);
		cv::waitKey(5); 
	}

	return bodyMasks; 
}

void PersonExtractor::SegmentDistanceNeightbours( cv::Mat& pointCloud, cv::Point2i seed, double maxDist, cv::Mat mask, cv::Mat& outSegmentationMask)
{
	outSegmentationMask = cv::Mat::zeros( pointCloud.rows, pointCloud.cols, CV_8UC1 ); 
	
	if( seed.x < 0 || seed.x >= pointCloud.cols || seed.y < 0 || seed.y >= pointCloud.rows )	
		return; 			

	cv::Mat labelsMask = cv::Mat::zeros(pointCloud.rows, pointCloud.cols, CV_8UC1); 
	std::queue< cv::Point2i > neighboursQueue; 

	neighboursQueue.push( seed ); 
	outSegmentationMask.at<uchar>( seed ) = 255; 
	labelsMask.at<uchar>( seed ) = 255;

	while( neighboursQueue.size() > 0 )
	{
		cv::Point2i currentPixel = neighboursQueue.front(); 
		neighboursQueue.pop();

		cv::Point3f currentPoint = pointCloud.at<cv::Vec3f>( currentPixel ); 

		// Neighbours			
		cv::Point2i up    = currentPixel + cv::Point2i(  1 ,  0); 
		cv::Point2i down  = currentPixel + cv::Point2i( -1 ,  0); 
		cv::Point2i left  = currentPixel + cv::Point2i(  0 , -1); 
		cv::Point2i right = currentPixel + cv::Point2i(  0 ,  1); 

		// Checking for Neighbours
		cv::Point2i neighPixel; 
		cv::Point3f neighPoint; 

		bool isInMask = false; 
		bool isNotLabeled = false; 
		bool isNearPoint = false;

		// Cheking for UP
		neighPixel = up;
		if( neighPixel.x>=0 && neighPixel.x<pointCloud.cols && neighPixel.y>=0 && neighPixel.y<pointCloud.rows ) 
		{
			neighPoint = pointCloud.at<cv::Vec3f>( neighPixel );
			
			isInMask = mask.at<uchar>(neighPixel) != 0 ;  // has normal up
			isNotLabeled = labelsMask.at<uchar>(neighPixel) == 0 ;
			isNearPoint = cv::norm( neighPoint - currentPoint ) < maxDist; 
			
			if( isInMask && isNotLabeled && isNearPoint ){
				neighboursQueue.push( neighPixel );

				labelsMask.at<uchar>( neighPixel ) = 255;
				outSegmentationMask.at<uchar>(neighPixel) = 255; 
			}
		}

		// Cheking for UP
		neighPixel = down;
		if( neighPixel.x>=0 && neighPixel.x<pointCloud.cols && neighPixel.y>=0 && neighPixel.y<pointCloud.rows ) 
		{
			neighPoint = pointCloud.at<cv::Vec3f>( neighPixel );
			
			isInMask = mask.at<uchar>(neighPixel) != 0 ;  // has normal up
			isNotLabeled = labelsMask.at<uchar>(neighPixel) == 0 ;
			isNearPoint = cv::norm( neighPoint - currentPoint ) < maxDist; 
			
			if( isInMask && isNotLabeled && isNearPoint ){
				neighboursQueue.push( neighPixel );

				labelsMask.at<uchar>( neighPixel ) = 255;
				outSegmentationMask.at<uchar>(neighPixel) = 255; 
			}
		}

				// Cheking for UP
		neighPixel = left;
		if( neighPixel.x>=0 && neighPixel.x<pointCloud.cols && neighPixel.y>=0 && neighPixel.y<pointCloud.rows ) 
		{
			neighPoint = pointCloud.at<cv::Vec3f>( neighPixel );
			
			isInMask = mask.at<uchar>(neighPixel) != 0 ;  // has normal up
			isNotLabeled = labelsMask.at<uchar>(neighPixel) == 0 ;
			isNearPoint = cv::norm( neighPoint - currentPoint ) < maxDist; 
			
			if( isInMask && isNotLabeled && isNearPoint ){
				neighboursQueue.push( neighPixel );

				labelsMask.at<uchar>( neighPixel ) = 255;
				outSegmentationMask.at<uchar>(neighPixel) = 255; 
			}
		}

		// Cheking for UP
		neighPixel = right;
		if( neighPixel.x>=0 && neighPixel.x<pointCloud.cols && neighPixel.y>=0 && neighPixel.y<pointCloud.rows ) 
		{
			neighPoint = pointCloud.at<cv::Vec3f>( neighPixel );
			
			isInMask = mask.at<uchar>(neighPixel) != 0 ;  // has normal up
			isNotLabeled = labelsMask.at<uchar>(neighPixel) == 0 ;
			isNearPoint = cv::norm( neighPoint - currentPoint ) < maxDist; 
			
			if( isInMask && isNotLabeled && isNearPoint ){
				neighboursQueue.push( neighPixel );

				labelsMask.at<uchar>( neighPixel ) = 255;
				outSegmentationMask.at<uchar>(neighPixel) = 255; 
			}
		}
	}

	cv::imshow("outSegmentationMask", outSegmentationMask); 
	

}