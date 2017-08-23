#include <iostream>
#include <typeinfo>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/stitching.hpp>

class PanoMaker
{
    public:
        PanoMaker();

        bool dbMode;
		std::string configdir;
		
        void AddImage(cv::Mat& imaBGR, cv::Mat& imaXYZ);
        void ClearImages(); 
        int GetNoImages();
        
        bool MakePanoramic(cv::Mat& panoBGR, cv::Mat& panoXYZ);

    private:
        std::vector< cv::Mat > listBGR;
        std::vector< cv::Mat > listXYZ;

        bool useGPU;
        bool scale;

        cv::Stitcher::Mode mode = cv::Stitcher::PANORAMA;
};  

PanoMaker::PanoMaker()
{
    this->listBGR = std::vector<  cv::Mat >(); 
    this->listXYZ = std::vector<  cv::Mat >(); 

    this->dbMode = false; 
    this->useGPU = false; 
    this->scale = true; 
}

void PanoMaker::AddImage(cv::Mat& imaBGR, cv::Mat& imaXYZ)
{
    this->listBGR.push_back( imaBGR );
    this->listXYZ.push_back( imaXYZ ); 
}

void PanoMaker::ClearImages()
{
    this->listBGR.clear();
    this->listXYZ.clear();
}

int PanoMaker::GetNoImages()
{
    return (int)(this->listBGR.size());
}

bool PanoMaker::MakePanoramic(cv::Mat& panoBGR, cv::Mat& panoXYZ)
{
    panoBGR = cv::Mat::zeros(10, 10, CV_8UC3); 
    panoXYZ = cv::Mat::zeros(10, 10, CV_32FC3);

	// reading config file
	try {
		
		cv::FileStorage configFile(this->configdir + "/config/panoconfig.xml", cv::FileStorage::READ);
		if (configFile.isOpened()) {
			configFile["scale"] >> this->scale;
			
			configFile.release();
		}
		
	} catch(...) {
		this->scale = false;
		std::cout << "Can't read the config file. Using default config." << std::endl;
	}


    if( this->GetNoImages() < 1)
    {
        std::cout << "WARNING (MakePanoramic): No images to make panoramic. Add first !" << std::endl; 
        return false;
    }

    cv::Ptr< cv::Stitcher > stitcher = cv::Stitcher::create(this->mode, this->useGPU); 
    
    std::vector< std::vector< cv::Rect > > rois; 
    cv::Stitcher::Status status = stitcher->stitch( this->listBGR, rois, panoBGR); 

    std::string errMsg = ""; 
    if( status == cv::Stitcher::OK ) {
		if(this->scale) 
			cv::resize(panoBGR, panoBGR, cv::Size(panoBGR.cols * 2, panoBGR.rows * 2));
        return true; 
    } else if( status == cv::Stitcher::ERR_NEED_MORE_IMGS)
        errMsg = "Needed more images";
    else if( status == cv::Stitcher::ERR_HOMOGRAPHY_EST_FAIL )
        errMsg = "Homography fail"; 
    else if( status == cv::Stitcher::ERR_CAMERA_PARAMS_ADJUST_FAIL )
        errMsg = "Camera params adjust fail";
    
    std::cout << "ERROR (MakePanoramic): "<< errMsg << std::endl; 
    return false; 
}
