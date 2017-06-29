#include "DetectedObject.hpp"

DetectedObject::DetectedObject( std::vector< cv::Point2i > indexes, std::vector< cv::Point3f > points3D, std::vector<cv::Point2f> points2D, float height, cv::Point3f centroid, cv::Mat oriMask ) 
{
	this->indexes = std::vector< cv::Point2i >( indexes ); 
	this->pointCloud = std::vector< cv::Point3f >( points3D ); 
	this->xyPoints2D = std::vector<cv::Point2f>( points2D ) ; 
	this->height = height; 
	this->centroid = cv::Point3f( centroid ); 
	this->oriMask = oriMask; 

    if( this->indexes.size() != 0 )
        this->boundBox = cv::boundingRect( this->indexes ); 
    
    if( this->xyPoints2D.size() != 0 )
    {
        this->shadowOriBoundBoxt2D = cv::minAreaRect( this-> xyPoints2D ); 
        cv::convexHull ( this->xyPoints2D, this->shadowCHull ); 
        cv::approxPolyDP( this->shadowCHull, this->shadowContour2D, 0.0001, true ); 
    }
}

DetectedObject::DetectedObject( cv::Mat bgrIma, cv::Mat xyzIma, cv::Mat validMask) 
{
    std::vector< cv::Point2i > indexes;
    std::vector< cv::Point3f > points3D;
    std::vector<cv::Point2f> points2D; 
    float height = 0.0; 
    
    int noPoints = 0; 
    cv::Point3f centroid(0.0, 0.0, 0.0); 

//   cv::imshow( "validMask", validMask ); 

    float minH =  99999999.9; 
    float maxH = -99999999.9;
    for( int i=0; i<validMask.rows; i++)
    {
        for(int j=0; j<validMask.cols; j++)
        {
            if( validMask.at<uchar>(i,j) < 1 )
                continue; 
            
            indexes.push_back(cv::Point2i(j,i));

            cv::Point3f pxyz = xyzIma.at< cv::Vec3f >( i, j ); 
            points3D.push_back( pxyz );         
            points2D.push_back( cv::Point2f( pxyz.x , pxyz.y ));  
            
            if( pxyz.z < minH )
                minH = pxyz.z; 
            if( pxyz.z > maxH )
                maxH = pxyz.z; 
        
            centroid += pxyz; 
            noPoints++; 
        }
    }  

	this->indexes = std::vector< cv::Point2i >( indexes ); 
	this->pointCloud = std::vector< cv::Point3f >( points3D ); 
	this->xyPoints2D = std::vector<cv::Point2f>( points2D ) ; 
	
    this->height = std::abs( maxH - minH ); 
	
    this->centroid = centroid / (float)noPoints; 
	this->oriMask = validMask; 

    if( this->indexes.size() != 0 )
    {
        this->boundBox = cv::boundingRect( this->indexes ); 
        this->image =  bgrIma( this->boundBox );
    }

    if( this->xyPoints2D.size() != 0 )
    {
        this->shadowOriBoundBoxt2D = cv::minAreaRect( this-> xyPoints2D ); 
        cv::convexHull ( this->xyPoints2D, this->shadowCHull ); 
        cv::approxPolyDP( this->shadowCHull, this->shadowContour2D, 0.0001, true ); 
    }
}

cv::Mat DetectedObject::GetImageWithMask()
{
    cv::Mat withMask;
    this->image.copyTo( withMask , oriMask( this->boundBox ));
    return withMask; 
}

bool DetectedObject::CompareByEuclidean(DetectedObject o1, DetectedObject o2) 
{
    double dist1 = o1.centroid.x*o1.centroid.x + o1.centroid.y*o1.centroid.y + o1.centroid.z*o1.centroid.z;
    double dist2 = o2.centroid.x*o2.centroid.x + o2.centroid.y*o2.centroid.y + o2.centroid.z*o2.centroid.z;

    return (dist1 < dist2);
}
