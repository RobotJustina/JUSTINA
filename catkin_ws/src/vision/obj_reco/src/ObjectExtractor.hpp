#pragma once
#include <iostream>

#include <opencv2/core/core.hpp>"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "justina_tools/JustinaTools.h"

#include "Plane3D.hpp"
#include "PlanarSegment.hpp"
#include "DetectedObject.hpp"

class ObjectExtractor
{
	public: 

        ObjectExtractor(std::string id, std::string path) const=0; 

        void DebugMode(bool enable); 
        std::vector< DetectedObject >  Extract( cv::Mat imaBGR, cv::Mat imaXYZ ) const = 0 ; // Virtual Method 
        
        void LoadParams();  
        void SaveParams();

	private:
		bool debug;
        std::string id;
        std::string path; 

        bool LoadFile( std::string path ) const=0; 
        bool SaveFile( std::string path ) const=0; 

}; 

ObjectExtractor::ObjectExtractor(std::string id)
{
    this->debug = false; 
    this->id = id; 
}

void ObjectExtractor::DebugMode(bool enable)
{
    this->debug = enable; 
}

public LoadParams()
{
    std::string( 
}
