#include "PlanarSegment.hpp"

PlanarSegment::PlanarSegment()
{}

PlanarSegment::PlanarSegment(Plane3D plane, std::vector<cv::Point3f> PointsXYZ, std::vector< cv::Point2i > indexes, std::vector<cv::Point2f> convexHull2D) 
{
	this->plane = Plane3D( plane ); 
	this->pointsXYZ = std::vector< cv::Point3f >( pointsXYZ ); 
	this->indexes = std::vector< cv::Point2i >( indexes ); 
	this->convexHull2D = std::vector<cv::Point2f>( convexHull2D ); 
}

std::vector< cv::Point3f > PlanarSegment::Get_PointsXYZ() const{
	return this->pointsXYZ; 
}

Plane3D PlanarSegment::Get_Plane() const{
	return this->plane; 
}

std::vector< cv::Point2i > PlanarSegment::Get_Indexes() const{
	return this->indexes; 
}

std::vector<cv::Point2f> PlanarSegment::Get_ConvexHull2D() const{
	return this->convexHull2D; 
}

double PlanarSegment::IsInside(cv::Point3f p){
	return cv::pointPolygonTest( this->convexHull2D, cv::Point2f( p.x, p.y ), true); 
}

//// INCOMPLETE !!! Only for detecting spot 

PlanarSegment::PlanarSegment(cv::Mat imaBGR, cv::Mat imaXYZ, cv::Mat mask)
{
    std::vector<cv::Point> validPixels;
    cv::findNonZero(mask, validPixels); 

    this->indexes = std::vector<cv::Point2i>(validPixels);

    //HSV 
    cv::Mat imaHSV; 
    cv::cvtColor( imaBGR, imaHSV, CV_BGR2HSV_FULL); 

    cv::Vec3i meanHSV(0,0,0); 
    cv::Vec3i stdDevHSV( 0,0,0 ); 

    int n = validPixels.size(); 
    for( int i=0; i<n; i++)
    {
        cv::Vec3b value = imaHSV.at<cv::Vec3b>( validPixels[i] ); 
        meanHSV += cv::Vec3i(value.val[0], value.val[1], value.val[2]); 
        stdDevHSV += cv::Vec3i( value.val[0]*value.val[0], value.val[1]*value.val[1], value.val[2]*value.val[2] ); 
    }

    meanHSV = meanHSV * ( 1.0 / n ); 
    stdDevHSV =  (1.0 / (n-1)) * (stdDevHSV - n*(meanHSV)) ;  
    stdDevHSV.val[0] = std::sqrt( (double)(stdDevHSV.val[0]) ); 
    stdDevHSV.val[1] = std::sqrt( (double)(stdDevHSV.val[1]) ); 
    stdDevHSV.val[2] = std::sqrt( (double)(stdDevHSV.val[2]) ); 

    this->meanHSV = meanHSV;
    this->stdDevHSV = stdDevHSV; 
    this->imaBGR = imaBGR; 
    this->imaXYZ = imaXYZ; 
    this->mask = mask; 
}

std::vector<cv::Point3f> PlanarSegment::DetectSpot(PlanarSegment planeSpot, cv::Mat& spotMask, std::vector<cv::Point>& centerPixels)
{
    double noStdDevs = 3; 

    cv::Mat spotsMask = cv::Mat::zeros( planeSpot.imaBGR.rows, planeSpot.imaBGR.cols, CV_8UC1); 
    
    cv::Mat hsvIma; 
    cv::cvtColor(planeSpot.imaBGR, hsvIma, CV_BGR2HSV_FULL);  
    
    int minH = this->meanHSV.val[0] -  this->stdDevHSV.val[0] * noStdDevs; 
    int maxH = this->meanHSV.val[0] +  this->stdDevHSV.val[0] * noStdDevs;

    for( int i=0; i< planeSpot.indexes.size(); i++)
    {
        cv::Point pixel =  planeSpot.indexes[i];  
        cv::Vec3b hsv = hsvIma.at<cv::Vec3b>(pixel); 
        int h = hsv.val[0];   
        
        if( h < minH || h > maxH )
            spotsMask.at<uchar>( pixel ) = 255; 
    }
    
    cv::erode( spotsMask, spotsMask, cv::getStructuringElement( cv::MORPH_CROSS, cv::Size(3,3) )  );  
    cv::erode( spotsMask, spotsMask, cv::getStructuringElement( cv::MORPH_CROSS, cv::Size(3,3) )  );  
    cv::dilate( spotsMask, spotsMask, cv::getStructuringElement( cv::MORPH_CROSS, cv::Size(3,3) )  );  
    cv::dilate( spotsMask, spotsMask, cv::getStructuringElement( cv::MORPH_CROSS, cv::Size(3,3) )  );  

    std::vector< std::vector<cv::Point> > contours;
    cv::findContours( spotsMask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point3f> centroids ; 
    for(int i=0; i<contours.size(); i++)
    {
        cv::Moments m = cv::moments( contours[i], true ); 
        cv::Point centerPixel( m.m10 / m.m00 , m.m01 / m.m00 ); 
        cv::Point3f centerXYZ = planeSpot.imaXYZ.at<cv::Vec3f>( centerPixel ); 

        centroids.push_back( centerXYZ ); 
        centerPixels.push_back( centerPixel ); 

        cv::circle( planeSpot.imaBGR, centerPixel, 5, cv::Scalar(0,0,255), -1); 
    }

    cv::imshow( "spotsMask", spotsMask ); 
    cv::imshow( "imaBGR center", planeSpot.imaBGR); 
    spotMask = spotsMask;
    
    return centroids; 
}

