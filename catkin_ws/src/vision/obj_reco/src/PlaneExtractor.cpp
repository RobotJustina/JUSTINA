#include "PlaneExtractor.hpp"

PlaneExtractor::PlaneExtractor()
{
    this->debug = false;
}

std::vector<PlanarSegment> PlaneExtractor::GetHorizontalPlanesRANSAC(cv::Mat& imaBGR, cv::Mat& imaXYZ)
{
    // Output
    std::vector<PlanarSegment> planes; 

    /////
    ///// PARAMETERS
    /////

    // PARAMS: Normals Extraction
	int blurSize = 5; 
	double normalZThreshold = 0.85; 

	// PARAMS: Planes RANSAC 
	int maxIter = 1000; 
	double maxDistPointToPlane = 0.02; 
	int minPointsForPlane = imaXYZ.rows*imaXYZ.cols*0.025;

	// For removing floor and far far away points 
	cv::Mat imaXYZInRangeMask;
	cv::inRange(imaXYZ, cv::Scalar(0.1, -1.0, 0.2), cv::Scalar(5.0, 1.0, 2.0), imaXYZInRangeMask); 
	if(debug)
       imshow("imaXYZInRangeMask", imaXYZInRangeMask ); 

    /////
	///// Getting Normals 	
	/////

    cv::Mat imaXYZBlur;
	cv::blur(imaXYZ, imaXYZBlur, cv::Size(blurSize, blurSize));
	cv::Mat imaNormals = this->CalculateFastNormalsImage( imaXYZBlur ); 
    if(debug)
        imshow("imaNormals", imaNormals); 

	// Getting Mask of Normals pointing vertical
	cv::Mat vertNormalsMask;
	cv::inRange( imaNormals, cv::Scalar(-1.0, -1.0, normalZThreshold), cv::Scalar(1.0, 1.0, 2.0), vertNormalsMask ); 

	// Mask of horizontal normals and valid.  
	cv::Mat validNormalsMask = vertNormalsMask & imaXYZInRangeMask;
    if(debug)
        imshow("validNormalsMask", validNormalsMask);  

    /////
    ///// PLANE EXTRACTION 
    /////
    
    // Getting valid points indexes.
    std::vector<cv::Point> validPixels;
    cv::findNonZero(validNormalsMask, validPixels); 

    /////
    ///// PLANE EXTRACTION DIST SEGMENTATION
    /////
    
    cv::Mat imaValidXYZ; 
    if(debug)
        imaValidXYZ = imaXYZ.clone(); 

    std::vector<cv::Point3f> validPoints;
    for(int i=0; i<validPixels.size(); i++)
    {
        validPoints.push_back( imaXYZ.at<cv::Vec3f>(validPixels[i]) ) ; 
        imaValidXYZ.at<cv::Vec3f>( validPixels[i] ) = cv::Vec3f(0,1,0); 
    }

    if( debug)
        cv::imshow( "imaValidXYZ", imaValidXYZ); 

    std::vector< std::vector< int > > planesIndexes;
    planesIndexes = SegmentByDistancePlanes(validPoints, 0.05, 0.005); // <--- Se puede optimizar !!!! (#noTime)
    //planesIndexes = SegmentByDistance( validPoints, 0.01); 

    cv::Mat imaPlanesDistSeg; 
    if(debug)
        imaPlanesDistSeg = imaBGR.clone(); 

    std::vector<cv::Mat> planesMask; 
    for(int i=0; i<planesIndexes.size(); i++)
    {
        if( planesIndexes[i].size() < minPointsForPlane )
            continue; 

        cv::Mat mask=cv::Mat::zeros(imaBGR.rows, imaBGR.cols, CV_8UC1); 

        cv::Vec3b color = cv::Vec3b( rand()%256, rand()%256, rand()%256 ); 
        for(int j=0; j<planesIndexes[i].size(); j++)
        {
            int index = planesIndexes[i][j]; 
            cv::Point pixel = validPixels[ index ];

            imaPlanesDistSeg.at<cv::Vec3b>( pixel ) = color; 
            mask.at<uchar>( pixel ) = 255; 
        }
        
        PlanarSegment planar( imaBGR, imaXYZ, mask); 
        
        planes.push_back( planar); 
        planesMask.push_back( mask ); 
    }

    if(debug)
    {
        cv::imshow( "imaPlanesDistSeg", imaPlanesDistSeg ); 

        cv::Mat planeBGR; 
        if( planesMask.size() > 0 )
        {
            imaBGR.copyTo( planeBGR, planesMask[0] ); 
            cv::imshow("planeBGR", planeBGR ); 
        }
    
        // COLOR SEGMENTATION 
        if( planesMask.size() == 0 )
            return planes; 

        cv::Mat imaPlane; 
        imaBGR.copyTo(imaPlane, planesMask[0]); 
        cv::imshow("imaPlane", imaPlane); 

        cv::Mat imaHSV;
        cv::cvtColor( imaBGR, imaHSV, CV_BGR2HSV_FULL); 

        DrawHistogram( imaBGR, planesMask[0], "imaPlanesBGR"); 
        DrawHistogram( imaHSV, planesMask[0], "imaPlanesHSV"); 
    }
    
    return planes; 

    ///// PLANE EXTRACTION RANSAC
    /////

    cv::Mat imaPlanes; 
    if(debug)
        imaPlanes = imaBGR.clone(); 


	int iter = 0; 
	while(iter++ < maxIter && validPixels.size() > minPointsForPlane)
	 {
		cv::Point3f p1  = imaXYZ.at<cv::Vec3f>(validPixels[rand()%validPixels.size()]); 
		cv::Point3f p2  = imaXYZ.at<cv::Vec3f>(validPixels[rand()%validPixels.size()]); 
		cv::Point3f p3  = imaXYZ.at<cv::Vec3f>(validPixels[rand()%validPixels.size()]); 

		if( !Plane3D::AreValidPointsForPlane( p1, p2, p3))
			continue; 

        
        Plane3D candidatePlane( cv::Point3f(0,0,1),  p1 );
		//Plane3D candidatePlane( p1, p2, p3);

		// Heuristics 
		if( std::abs(candidatePlane.GetNormal().z < 0.99) )
			continue; 

        cv::Mat imaNormalsPlane; 
        if(debug)
            imaNormalsPlane = imaBGR.clone(); 

		// For all points, checking distance to candidate plane (getting inliers) 
        std::vector<cv::Point3f>    inliersPoints;
        std::vector<cv::Point>      inliersPixels;
        std::vector<cv::Point3f>    outliersPoints;
        std::vector<cv::Point>      outliersPixels;
		for( size_t i=0; i<validPixels.size(); i++)
		{
			cv::Point3f xyzPoint = imaXYZ.at<cv::Vec3f>(validPixels[i]);
			double distToPlane = candidatePlane.DistanceToPoint(xyzPoint);

			if( distToPlane < maxDistPointToPlane )
            {
                inliersPoints.push_back(xyzPoint);
                inliersPixels.push_back(validPixels[i]);
            }
            else
            {
                outliersPoints.push_back(xyzPoint);
                outliersPixels.push_back(validPixels[i]);
            }
        }
    
		if( inliersPoints.size() < minPointsForPlane )
			continue;

        if( debug )
        {
            cv::Vec3b color(rand()%256, rand()%256, rand()%256); 
            for( int i=0; i<inliersPixels.size(); i++)
                imaPlanes.at<cv::Vec3b>( inliersPixels[i] ) = color ; 
        }
        
		// Getting better plane using PCA
		cv::PCA pca( cv::Mat(inliersPoints).reshape(1), cv::Mat(), CV_PCA_DATA_AS_ROW); 
		cv::Point3f pcaNormal( pca.eigenvectors.at<float>(2,0), pca.eigenvectors.at<float>(2,1), pca.eigenvectors.at<float>(2,2) ); 
		cv::Point3f pcaPoint(pca.mean.at<float>(0,0), pca.mean.at<float>(0,1), pca.mean.at<float>(0,2));

		Plane3D refinedPlane( pcaNormal, pcaPoint ); 
        //planes.push_back( refinedPlane ); 

        validPixels = outliersPixels; 
    }

    if( debug )
        cv::imshow( "imaPlanes", imaPlanes); 

    return planes; 
}

cv::Mat PlaneExtractor::CalculateFastNormalsImage(cv::Mat imaXYZ, cv::Mat mask)
{
	cv::Mat normals = cv::Mat::zeros(imaXYZ.rows, imaXYZ.cols, CV_32FC3); 

	if( !mask.data )
		mask = cv::Mat::ones(imaXYZ.rows, imaXYZ.cols, CV_8UC1); 

	cv::Point3f pointi; 
	cv::Point3f topLeft; 
	cv::Point3f topRight; 
	cv::Point3f downLeft; 
	cv::Point3f downRight; 

	cv::Point3f normal_1; 
	cv::Point3f normal_2; 
	cv::Point3f normal; 
			
	cv::Point3f viewPoint(0.0,0.0,0.0);

	for( int idxRow = 1 ; idxRow < imaXYZ.rows - 1 ; idxRow++ )
	{
		for( int idxCol = 1 ; idxCol < imaXYZ.cols - 1 ; idxCol++ )
		{			
			if( mask.at<uchar>( idxRow,idxCol ) == 0.0 )
				continue; 
			
			// Getting Vectors
			pointi = imaXYZ.at<cv::Vec3f>(idxRow, idxCol);

			topLeft = imaXYZ.at<cv::Vec3f>(idxRow-1, idxCol-1);
			topRight = imaXYZ.at<cv::Vec3f>(idxRow-1, idxCol+1);
			downLeft = imaXYZ.at<cv::Vec3f>(idxRow+1, idxCol-1);
			downRight = imaXYZ.at<cv::Vec3f>(idxRow+1, idxCol+1);
			
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

std::vector< std::vector< int > > PlaneExtractor::SegmentByDistancePlanes(std::vector< cv::Point3f > xyzPointsList, double distThreshold, double maxDistZ)
{
	double ticks;
	ticks = cv::getTickCount(); 

	std::vector< int > labels(xyzPointsList.size(), 0); 
	std::vector< std::vector< int > > labelsVec; 

	if( xyzPointsList.size() == 0 )
		return labelsVec; 

	cv::Mat xyzPointMat = cv::Mat(xyzPointsList).reshape(1);
    cv::Mat xyMat = xyzPointMat.colRange(0,2).clone();  
    
	//std::cout << "1t=" << ((double)cv::getTickCount() - ticks) / cv::getTickFrequency() << std::endl; 
    
    //std::cout << "xyzPointMat.row" << xyzPointMat.row(0) << std::endl; 
    //std::cout << "xyMat.row" << xyMat.row(0) << std::endl; 

	ticks = cv::getTickCount(); 

	cv::flann::KDTreeIndexParams idxParams(1); 
	cv::flann::Index kdTree(xyMat, idxParams); 

	//std::cout << "2t=" << ((double)cv::getTickCount() - ticks) / cv::getTickFrequency() << std::endl; 

	// KD uses distance without squareRoot 
	double distSquare = distThreshold*distThreshold; 

	ticks = cv::getTickCount(); 

    double ticksSearchTime = 0.0; 
    int ticksSearchCnt = 0; 

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
			
            cv::Mat xyPoint = currentPoint.colRange(0,2);
            //std::cout << "xyPoint:" << xyPoint << std::endl; 

	        double ticks1 = cv::getTickCount(); 
            kdTree.radiusSearch( xyPoint, idxs, dists, distSquare, 16 );         
	        ticksSearchTime += ((double)cv::getTickCount() - ticks1) / cv::getTickFrequency();
            ticksSearchCnt++; 

            //std::cout << "radiusSearch finish: idxSize:" << idxs.size() << std::endl; 

			for(int i=0; i<(int)idxs.size(); i++)
			{
                float z1 = currentPoint.at<float>(0,2); 
                float z2 = xyzPointMat.row( idxs[i] ).at<float>(0,2);
                double distInZ = std::abs( z1 - z2 ); 
                //std::cout << "z1:" << z1 << " , z2:" << z2 << " , dist:" << distInZ << std::endl; 

				if( (labels[ idxs[i] ] == 0) && (distInZ < maxDistZ) )
				{
					labels[ idxs[i] ] = labelCnt; 
					nnQueue.push( xyzPointMat.row( idxs[i]) ); 
					labelCluster.push_back( idxs[i] );
				}
			}

            //std::cout << "for finish" << std::endl; 
		}

		labelsVec.push_back( labelCluster ); 

        //std::cout << "while finish" << std::endl; 
	    //cv::waitKey(-1); 
    }
    //std::cout << "search totalTime:" << ticksSearchTime << ", avgTime: " << ticksSearchTime / (double)ticksSearchCnt << " , cnt:" << ticksSearchCnt << std::endl; 
	//std::cout << "3t=" << ((double)cv::getTickCount() - ticks) / cv::getTickFrequency() << std::endl; 
	return labelsVec; 
}


std::vector< std::vector< int > > PlaneExtractor::SegmentByDistance(std::vector< cv::Point3f > xyzPointsList, double distThreshold)
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

			std::vector<float> dists(10000); 
			std::vector<int> idxs(10000); 
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

void PlaneExtractor::DrawHistogram(cv::Mat ima, cv::Mat mask, std::string histoName)
{
    std::vector<cv::Mat> bgr_planes;
    cv::split( ima, bgr_planes );

    int histSize = 256;

    float range[] = { 0, 256 } ;
    const float* histRange = { range };
    bool uniform = true; 
    bool accumulate = false;

    cv::Mat b_hist, g_hist, r_hist;
    cv::calcHist( &bgr_planes[0], 1, 0, mask, b_hist, 1, &histSize, &histRange, uniform, accumulate );
    cv::calcHist( &bgr_planes[1], 1, 0, mask, g_hist, 1, &histSize, &histRange, uniform, accumulate );
    cv::calcHist( &bgr_planes[2], 1, 0, mask, r_hist, 1, &histSize, &histRange, uniform, accumulate );

    int hist_w = 512; 
    int hist_h = 400;
    int bin_w = cvRound( (double) hist_w/histSize );

    cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );

    cv::normalize(b_hist, b_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
    cv::normalize(g_hist, g_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
    cv::normalize(r_hist, r_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );

    for( int i = 1; i < histSize; i++ )
    {
        cv::line(   histImage, 
                    cv::Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
                    cv::Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
                    cv::Scalar( 255, 0, 0), 2, 8, 0  );
        cv::line(   histImage, 
                    cv::Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
                    cv::Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
                    cv::Scalar( 0, 255, 0), 2, 8, 0  );
        cv::line(   histImage, 
                    cv::Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ) ,
                    cv::Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
                    cv::Scalar( 0, 0, 255), 2, 8, 0  );
    }

    cv::imshow( histoName, histImage); 
}
