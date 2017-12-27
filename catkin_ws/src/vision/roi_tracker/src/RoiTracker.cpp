#include "RoiTracker.hpp"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTasks.h"

//vision_msgs::VisionFaceObjects faces;


/*bool RoiTracker::faceSort(vision_msgs::VisionFaceObject &i, vision_msgs::VisionFaceObject &j)
{
    return i.face_centroid.x < j.face_centroid.x;
}*/

RoiTracker::RoiTracker()
{
    this->init = false; 

    this->Debug = false; 
    this->noBins = 18; 

    this->frontLeftBot = cv::Scalar( 0.50, -0.30, 0.30 ); 
    this->backRightTop = cv::Scalar( 2.00,  0.30, 2.00 ); 

    this->overPercWidth  = 0.750;
    this->overPercHeight = 0.750;
    this->overNoRectsWidth  = 4;
    this->overNoRectsHeight = 4;  

    this->scaleFactor = 0.20; 
    this->scaleSteps = 3.00; 
    this->scaleMax = cv::Size(640,480); 
    this->scaleMin = cv::Size(64,128); 
    
    this->matchThreshold = 0.85;
}  

bool RoiTracker::InitTracking(cv::Mat imaBGR, cv::Mat imaXYZ, cv::Rect roiToTrack)
{ 
    cv::Mat mask = cv::Mat::ones( imaBGR.rows, imaBGR.cols, CV_8UC1);
    return InitTracking( imaBGR, imaXYZ, roiToTrack, mask ); 
}

bool RoiTracker::LoadParams( std::string configFile )
{
    
    
    try{
        // Getting configFile
        cv::FileStorage fs; 
        if( fs.open( configFile, fs.READ) )
        { 

            this->Debug = ( (int)fs["Debug"] == 0 ) ? false : true ; 
            this->noBins = (int)fs["noBins"]; 

            fs["frontLeftBot"] >> this-> frontLeftBot; 
            fs["backRightTop"] >> this-> backRightTop; 

            this->overPercWidth  = (float)fs["overPercWidth"];
            this->overPercHeight = (float)fs["overPercHeight"];
            this->overNoRectsWidth  = (int)fs["overNoRectsWidth"];
            this->overNoRectsHeight = (int)fs["overNoRectsHeight"];  

            this->scaleFactor = (float)fs["scaleFactor"]; 
            this->scaleSteps = (int)fs["scaleSteps"]; 
            fs["scaleMax"] >> this->scaleMax;
            fs["scaleMin"] >> this->scaleMin; 

            this->matchThreshold = (float)fs["matchThreshold"];

            fs.release(); 

            std::cout << ">> RoiTracker. Read configuration:" << configFile << std::endl; 
        }
        // if not exist, create it
        else
        {
            if(fs.open( configFile, fs.WRITE ) )
            {
                std::cout << ">> RoiTracker. Writing configFile:" << configFile << ".Creating it." << std::endl; 

                fs << "Debug" << ( this->Debug ? 1 : 0 ); 
                fs << "noBins" << this->noBins; 

                fs << "frontLeftBot" << this-> frontLeftBot; 
                fs << "backRightTop" << this-> backRightTop; 

                fs << "overPercWidth" << this->overPercWidth;
                fs << "overPercHeight" << this->overPercHeight;
                fs << "overNoRectsWidth" << this->overNoRectsWidth;
                fs << "overNoRectsHeight" << this->overNoRectsHeight ;  

                fs << "scaleFactor" << this->scaleFactor; 
                fs << "scaleSteps" << this->scaleSteps; 
                fs << "scaleMax" << this->scaleMax;
                fs << "scaleMin" << this->scaleMin; 

                fs << "matchThreshold" << this->matchThreshold;

                fs.release(); 
            }
        }
    }
    catch(...)
    {
        std::cout << "Exception while openning file. Using default params..." << std::endl;
        return false; 
    }

    return true; 
}

bool RoiTracker::InitTracking(cv::Mat imaBGR, cv::Mat imaXYZ, cv::Rect roiToTrack, cv::Mat mask)
{
    /// Segmenting by 3D info 
    cv::Mat imaRoi = imaBGR( roiToTrack ); 
    cv::Mat maskRoi = mask( roiToTrack ); 
    if( Debug ) 
        imshow("imaRoi", imaRoi); 


    this->histoToTrack = CalculateHistogram( imaRoi, maskRoi );
    if( Debug )
        std::cout << "HistoToTrack:" << histoToTrack << std::endl; 

    this->roiToTrack = roiToTrack; 

    this->init = true; 
    return this->init; 
}

bool RoiTracker::InitFront(cv::Mat imaBGR, cv::Mat imaXYZ)
{
    cv::Mat mask;
    cv::inRange(imaXYZ, this->frontLeftBot , this->backRightTop, mask);   

    int noPixels = cv::countNonZero( mask );
    
    if( Debug)
        std::cout << "No Pixels for init:" << noPixels << std::endl;

    //if( noPixels < 10000 )
    if( noPixels < 5000 )
    {
        this->init = false; 
        return false; 
    }

    cv::Rect roi = cv::boundingRect( mask ); 
    if( Debug )
    {
        cv::Mat imaRoi = imaBGR.clone();
        cv::rectangle( imaRoi, roi, cv::Scalar(0,255,0), 3 ); 
        cv::imshow( "imaRoi", imaRoi ); 
    }

    std::cout   << "Roi to Track:"
                << " tl.x=" << roi.tl().x 
                << " tl.y=" << roi.tl().y 
                << " size.width=" << roi.size().width 
                << " size.height=" << roi.size().height
                << std::endl;  
    
    cv::Mat maskToUse;
    //maskToUse = mask; 
    maskToUse = cv::Mat::ones(mask.rows, mask.cols, CV_8UC1 );

    this->InitTracking( imaBGR, imaXYZ, roi, maskToUse );  
    return true; 
}

bool RoiTracker::Update(cv::Mat imaBGR, cv::Mat imaXYZ, cv::Rect& nextRoi, double& confidence)
{ 
    confidence = 0.0; 

    if( this->roiToTrack.size() == cv::Size() )
    {
        std::cout << "ERROR!!! roiToTrack.size is equal to  0" << std::endl;
        return false; 
    }
    if( this->roiToTrack.tl().x < 0 || this->roiToTrack.tl().y >= imaBGR.cols )
    {
        std::cout << "ERROR!!! roiToTrack.tl outside of image" << std::endl;
        return false; 
    }
    if( this->roiToTrack.br().x < 0 || this->roiToTrack.br().y >= imaBGR.rows )
    {
        std::cout << "ERROR!!! roiToTrack.br outside of image" << std::endl;
        return false; 
    }

    //std::vector< cv::Rect > rois =  this->GetSearchRois( this->roiToTrack, imaBGR );
    std::vector< cv::Rect > rois = GetSearchRoisMultiscale( this->roiToTrack, imaBGR );
    
    std::vector< cv::Mat > histos; 
    std::vector< double > matches; 

    double bestMatch = -9999999999999.9;
    int bestIndex = 0; 
    for(int i=0; i< rois.size(); i++)
    {
        cv::Mat roiIma; 
        try
        {
            roiIma = imaBGR( rois[i] );
        }
        catch(...)
        {
            std::cout << ">>>>> EXCEPTION!! at roiIma" << std::endl; 
            continue; 
        }

        cv::Mat histo;
        try
        {
            histo = CalculateHistogram( roiIma ); 
        }
        catch(...)
        {
            std::cout << ">>>>> EXCEPTION!! at  CalculateHistogram" << std::endl; 
            continue; 
        }

        double match = cv::compareHist( this->histoToTrack, histo, cv::HISTCMP_INTERSECT );

        histos.push_back( histo );
        matches.push_back( match ); 
    
        if( match > bestMatch )
        {
            bestMatch = match; 
            bestIndex = i;
        }
    }

    bool success; 
    if( bestMatch > this->matchThreshold )
    {
        nextRoi = rois[bestIndex];
        confidence = bestMatch; 
        this->roiToTrack = nextRoi; 
        success = true; 
    }
    else
    {
        nextRoi = this->roiToTrack; 
        confidence = bestMatch; 
        this->roiToTrack = this->roiToTrack; 
        success = false; 
    }

    /*if( Debug )
    {
        cv::Mat bestMatch = imaBGR.clone();
        // Best Match
        cv::rectangle(bestMatch, rois[bestIndex], cv::Scalar(0,0,255), 2);
        // Next Roi
        cv::rectangle(bestMatch, nextRoi, cv::Scalar(255,0,0), 4);  
        // Text
        std::stringstream ss;
        ss << "Match%: " << matches[bestIndex] << " (Success:" << success << ")"; 
        cv::putText( bestMatch, ss.str(), cv::Point(10,30), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0,0,255), 2); 
       
        cv::imshow("bestMatch", bestMatch); 
    }*/

    return success; 
}

cv::Mat RoiTracker::CalculateHistogram(cv::Mat bgrIma, cv::Mat mask)
{
	cv::Mat hsvImage; 
	cv::cvtColor( bgrIma, hsvImage, CV_BGR2HSV_FULL ); 

	cv::Mat hueHisto;
	int chan[] = { 0 }; 
	int histSize[] = { this->noBins };
	float hueRange[] = { 0, 255 };
	const float* ranges[] = { hueRange };
	
	// color histrogram
	cv::Mat hueMask; 
	cv::inRange( hsvImage, cv::Scalar( 0, 50, 50), cv::Scalar(255, 255, 205), hueMask);
    cv::Mat colorMask = mask & hueMask; 

	calcHist(&hsvImage, 1, chan, colorMask, hueHisto, 1, histSize, ranges, true, false);

	// Whites count
	cv::Mat satMask; 
	cv::inRange( hsvImage, cv::Scalar( 0, 0, 205), cv::Scalar(255, 50, 255), satMask);
	cv::Mat whiteMask = mask & satMask; 
	int whitePx = cv::countNonZero( whiteMask ); 

	// Blacks count
	cv::Mat valMask; 
	cv::inRange( hsvImage, cv::Scalar( 0, 0, 0), cv::Scalar(255, 255, 50), valMask); // BUG ? 
	cv::Mat blackMask = mask & valMask; 
	int blackPx = cv::countNonZero( blackMask ); 

    /// CREATING NEW HISTOGRAM FOR INCLUIDING COLOR, WHITE AND BLACK
	cv::Mat completeHisto = cv::Mat( hueHisto.rows+2, hueHisto.cols, hueHisto.type() ); 
	for( int i=0; i< hueHisto.rows; i++)
		completeHisto.at<float>(i,0) = hueHisto.at<float>( i, 0); 
	
	completeHisto.at<float>( hueHisto.rows   , 0 ) = blackPx; 
	completeHisto.at<float>( hueHisto.rows+1 , 0 ) = whitePx; 

	cv::normalize(completeHisto, completeHisto, 1.0, 0.0, cv::NORM_L1); 
	return completeHisto; 
} 

cv::Mat RoiTracker::CalculateHistogram(cv::Mat bgrIma)
{
    cv::Mat mask = cv::Mat::ones( bgrIma.rows, bgrIma.cols, CV_8UC1); 
    return this->CalculateHistogram( bgrIma, mask); 
}

std::vector< cv::Rect > RoiTracker::GetSearchRois( cv::Rect centerRoi, cv::Mat bgrIma )
{ 
    std::vector< cv::Rect > rois; 
    cv::Mat searchRois; 
    if( Debug )
        searchRois = bgrIma.clone();

    int overlapWidth_inPixels    =  (int)( ((double)centerRoi.size().width)   * overPercWidth );
    int overlapHeight_inPixels   =  (int)( ((double)centerRoi.size().height)  * overPercHeight );

    //std::cout << "ovW_px:" << overlapWidth_inPixels << " ovH_px:" << overlapHeight_inPixels << std::endl; 

    int noOverlapWidth_inPixels     = centerRoi.size().width    - overlapWidth_inPixels; 
    int noOverlapHeight_inPixels    = centerRoi.size().height   - overlapHeight_inPixels; 

    //std::cout << "NoovW_px:" << noOverlapWidth_inPixels << " NoovH_px:" << noOverlapHeight_inPixels << std::endl; 

    int firstCol =  centerRoi.tl().x - noOverlapWidth_inPixels   * overNoRectsWidth;
    int firstRow =  centerRoi.tl().y - noOverlapHeight_inPixels  * overNoRectsHeight;

    int lastCol =   firstCol + noOverlapWidth_inPixels  * (overNoRectsWidth  * 2 + 1);
    int lastRow =   firstRow + noOverlapHeight_inPixels * (overNoRectsHeight * 2 + 1);  

    //std::cout << "firstRow:"    << firstRow << " firstCol:"  << firstCol << std::endl; 
    //std::cout << "lastRow:"     << lastRow  << " lastCol:"   << lastCol  << std::endl; 

    for(int i=firstRow; i<lastRow; i=i+noOverlapHeight_inPixels)
    {
        for(int j=firstCol; j<lastCol; j=j+noOverlapWidth_inPixels)
        {
            cv::Point topLeft( j , i );
            cv::Size size = centerRoi.size(); 
            cv::Rect rect = cv::Rect( topLeft , size ); 

            //std::cout << "Tl: " << rect.tl() << " , Br: " << rect.br() << std::endl;  

            if( rect.tl().x <= 0 || rect.tl().x >= bgrIma.cols )
                continue;
            if( rect.tl().y <= 0 || rect.tl().y >= bgrIma.rows )
                continue;  
            if( rect.br().x <= 0 || rect.br().x >= bgrIma.cols )
                continue;
            if( rect.br().y <= 0 || rect.br().y >= bgrIma.rows )
                continue;  

            if( Debug )
            {
                //std::cout << "Rect: " << rect << std::endl;  
                cv::Scalar color = cv::Scalar( 255%i , 0, 0 );
                cv::rectangle( searchRois, rect, color, 3); 
            }

            //std::cout << "              > Adding..." << std::endl; 
            rois.push_back( rect ); 
        }
    }

    //if( Debug )
    //{
    //    cv::rectangle( searchRois , this->roiToTrack, cv::Scalar(0,255,0), 3); 
    //    cv::imshow( "SearchRois", searchRois ); 
    //}

    return rois; 
}

std::vector< cv::Rect > RoiTracker::GetSearchRoisMultiscale( cv::Rect centerRoi, cv::Mat bgrIma )
{ 
    cv::Mat searchRois; 
    cv::Mat centerRoisIma; 
    if( Debug )
    {
        searchRois = bgrIma.clone();
        centerRoisIma = bgrIma.clone();
    }

    cv::Size scaleIncrement; 
    scaleIncrement.width = (int)(((double)centerRoi.width) * this->scaleFactor); 
    scaleIncrement.height = (int)(((double)centerRoi.height) * this->scaleFactor); 
   
    cv::Point centerPoint = centerRoi.tl() + cv::Point( (int)(((double)centerRoi.width) / 2.0) , (int)(((double)centerRoi.height) / 2.0) ); 

    //std::cout << "scaleIncrement:" << scaleIncrement << std::endl; 
    //std::cout << "centerPoint:" << centerPoint << std::endl; 

    // Adding original roi
    std::vector< cv::Rect > centerRois;
    centerRois.push_back( centerRoi ); // First Roi 
    if( Debug )
    {
        //std::cout << "CenterRoi: " << centerRoi << std::endl; 
        cv::rectangle( centerRoisIma, centerRoi, cv::Scalar(0,255,0), 2);
    }

    // Creating center Rois. 

    cv::Rect roi;
    // UPSCALING
    roi = centerRoi;  
    for( int i =0;  i<this->scaleSteps; i++)
    {
        cv::Size size = roi.size() + scaleIncrement; 
        cv::Point tl = centerPoint - cv::Point( (int)(((double)size.width) / 2.0) , (int)(((double)size.height) / 2.0) );

        roi = cv::Rect( tl, size ); 
        
        if( roi.size().width >= this->scaleMax.width || roi.size().height >= this->scaleMax.height )
            break;

        centerRois.push_back( roi );
        if( Debug )
        {
            //std::cout << "UpScale roi: " << roi << std::endl; 
            cv::rectangle( centerRoisIma, roi, cv::Scalar(255,255,0), 2);
        }
    }

    // DOWNSCALING 
    roi = centerRoi;
    for( int i =0;  i<this->scaleSteps; i++)
    {
        cv::Size size = roi.size() - scaleIncrement; 
        cv::Point tl = centerPoint - cv::Point( (int)(((double)size.width) / 2.0) , (int)(((double)size.height) / 2.0) );

        roi = cv::Rect( tl, size ); 

        if( roi.size().width <= this->scaleMin.width || roi.size().height <= this->scaleMin.height )
            break;

        centerRois.push_back( roi ); 
        if( Debug )
        {
            //std::cout << "DwScale roi: " << roi << std::endl; 
            cv::rectangle( centerRoisIma, roi, cv::Scalar(0,255,255), 2);
        }
    }

    /*if(Debug)
        cv::imshow( "centerRoisIma", centerRoisIma ); */
    
    //cv::waitKey(-1);

    std::vector< cv::Rect > rois; 
    for( int k=0; k<centerRois.size(); k++)
    {
        centerRoi = centerRois[k];

        //std::cout << "CenterRoi " << k << " : " << centerRoi << std::endl; 
        std::vector< cv::Rect > scaleRois = GetSearchRois( centerRoi , bgrIma ); 
        rois.insert( rois.end(), scaleRois.begin(), scaleRois.end()); 

        if( Debug )
        {
            for( int i=0; i<scaleRois.size(); i++)
                cv::rectangle( searchRois, scaleRois[i], cv::Scalar(0,255,0), 2);

            //cv::imshow( "SearchRoisMultiscale", searchRois ); 
            //cv::waitKey(-1); 
        }
    } 

    return rois; 
}





// FIRST ATTEMPO TO GET ROIS 
/*
    std::vector< cv::Rect > rois;

    int overWidth = 8; 
    int overHeigth = 8; 

    int overNoWidth = 8;
    int overNoHeight = 8; 
  
    int overSizeWidth = this->roiToTrack.size().width / overWidth; 
    int overSizeHeight = this->roiToTrack.size().height / overHeigth; 

    int firstCol = this->roiToTrack.tl().x - overNoWidth*overSizeWidth; 
    int lastCol  = this->roiToTrack.tl().x + overNoWidth*overSizeWidth; 

    int firstRow = this->roiToTrack.tl().y - overNoHeight*overSizeHeight;
    int lastRow = this->roiToTrack.tl().y + overNoHeight*overSizeHeight;

    cv::Mat bgrCopy;
    if( Debug )
        bgrCopy = bgrIma.clone(); 
    
    for( int i = firstCol ; i <= lastCol ; i=i+overSizeWidth )
    {
        cv::Point topLeft; 
        topLeft.x = i; 

        for( int j= firstRow ; j<= lastRow ; j=j+overSizeHeight )
        {
            topLeft.y = j;

            cv::Rect rect = cv::Rect( topLeft, this->roiToTrack.size() ); 

            if( rect.tl().x < 0 || rect.tl().x >= bgrIma.cols )
                continue;
            if( rect.tl().y < 0 || rect.tl().y >= bgrIma.rows )
                continue;  

            if( rect.br().x < 0 || rect.br().x >= bgrIma.cols )
                continue;
            if( rect.br().y < 0 || rect.br().y >= bgrIma.rows )
                continue;  

            rois.push_back( rect ); 

            if( Debug )
            {
                cv::Scalar color = cv::Scalar( 255%i , 0, 0 );
                cv::rectangle( bgrCopy, rect, color, 3); 
            }
        }
    }

    if( Debug )
    {
        cv::rectangle( bgrCopy , this->roiToTrack, cv::Scalar(0,255,0), 3); 
        cv::imshow( "SearchRois", bgrCopy ); 
    }

    return rois; 
*/
