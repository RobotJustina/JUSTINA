#include "RoiTracker.hpp"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTasks.h"

//vision_msgs::VisionFaceObjects faces;


/*bool RoiTracker::faceSort(vision_msgs::VisionFaceObject &i, vision_msgs::VisionFaceObject &j)
{
    return i.face_centroid.x < j.face_centroid.x;
}*/
// correlation_tracker tracker(unsigned long filter_size = 6,
//             unsigned long num_scale_levels = 5,
//             unsigned long scale_window_size = 23,
//             double regularizer_space = 0.001,
//             double nu_space = 0.025,
//             double regularizer_scale = 0.001,
//             double nu_scale = 0.025,
//             double scale_pyramid_alpha = 1.020);

RoiTracker::RoiTracker()
{
    this->init = false;

    this->Debug = true;
    this->noBins = 18;
    this->noExperiences=1200;

    this->frontLeftBot = cv::Scalar( 0.50, -0.30, 0.30 );
    this->backRightTop = cv::Scalar( 2.00,  0.30, 2.00 );

    this->overPercWidth  = 0.90;
    this->overPercHeight = 0.90;
    this->overNoRectsWidth  = 4;
    this->overNoRectsHeight = 3;

    this->scaleFactorIncrement = 0.10;
    this->scaleFactorDecrement = 0.10;
    this->scaleSteps = 4.00;
    this->scaleMax = cv::Size(100,100);
    this->scaleMin = cv::Size(10,10);

    this->matchThreshold = 0.92;
    this->Exper=0;
}

bool RoiTracker::InitTracking(cv::Mat imaBGR, cv::Mat imaXYZ,  cv::Rect roiToTrack)
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
            this->noExperiences =(int)fs["noExperiences"];

            fs["frontLeftBot"] >> this-> frontLeftBot;
            fs["backRightTop"] >> this-> backRightTop;

            this->overPercWidth  = (float)fs["overPercWidth"];
            this->overPercHeight = (float)fs["overPercHeight"];
            this->overNoRectsWidth  = (int)fs["overNoRectsWidth"];
            this->overNoRectsHeight = (int)fs["overNoRectsHeight"];

            this->scaleFactorIncrement = (float)fs["scaleFactorIncrement"];
            this->scaleFactorDecrement = (float)fs["scaleFactorDecrement"];
            this->scaleSteps = (int)fs["scaleSteps"];
            fs["scaleMax"] >> this->scaleMax;
            fs["scaleMin"] >> this->scaleMin;

            this->matchThreshold = (float)fs["matchThreshold"];

            fs.release();

            //std::cout << ">> RoiTracker. Read configuration:" << configFile << std::endl;
        }
        // if not exist, create it
        else
        {
            if(fs.open( configFile, fs.WRITE ) )
            {
                std::cout << ">> RoiTracker. Writing configFile:" << configFile << ".Creating it." << std::endl;

                fs << "Debug" << ( this->Debug ? 1 : 0 );
                fs << "noBins" << this->noBins;
                fs << "noExperiences" << this->noExperiences;

                fs << "frontLeftBot" << this-> frontLeftBot;
                fs << "backRightTop" << this-> backRightTop;

                fs << "overPercWidth" << this->overPercWidth;
                fs << "overPercHeight" << this->overPercHeight;
                fs << "overNoRectsWidth" << this->overNoRectsWidth;
                fs << "overNoRectsHeight" << this->overNoRectsHeight ;

                fs << "scaleFactorIncrement" << this->scaleFactorIncrement;
                fs << "scaleFactorDecrement" << this->scaleFactorDecrement;
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
    /*if( Debug )
        imshow("imaRoi", imaRoi);*/

    cv::Mat Temp;
 	Temp= CalculateHistogram( imaRoi, maskRoi );
    this->histoToTrack.push_back(Temp.t());

    //if( Debug )
        //std::cout << "HistoToTrack:" << histoToTrack << std::endl;

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
    if( noPixels < 4000 )
    {
        this->init = false;
        return false;
    }

    cv::Rect roi = cv::boundingRect( mask );
    /*if( Debug )
    {
        cv::Mat imaRoi = imaBGR.clone();
        cv::rectangle( imaRoi, roi, cv::Scalar(0,255,0), 3 );
//////////////////////////////////////// correlation_tracker

        cv::imshow( "imaRoi", imaRoi );
    }*/

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


bool RoiTracker::IfPerson(cv::Mat imaBGR){

	/********************/
    // Carga el archivo xml para detectar caras:
    if (!face_cascade.load("/usr/local/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml"))
    {
    ///usr/local/share/OpenCV/haarcascades{
    cout << "Cannot load face xml!" << endl;
    return -1;
    }
	/********************/
    cv::cvtColor(imaBGR	, gray, CV_BGR2GRAY);
    cv::equalizeHist(gray, gray);
    face_cascade.detectMultiScale(gray, faces, 1.2, 3); // Detectamos las caras presentes en la imagen

    if(faces.size()>0){ // si se encuentran caras

  		cv::Rect r = faces[0];
  		r.y=r.y+150;
  		r.x=r.x+30;
  		r.width=r.width;
  		r.height=r.height;

    // Load the first frame.
    /*    array2d<unsigned char> dlibImage;
        dlib::assign_image(dlibImage, dlib::cv_image<bgr_pixel>(imaBGR));
        cv::imshow("Track_Correlation",imaBGR);
        tracker.start_track(dlibImage, centered_rect(point((int)((r.x+r.width)/2),(int)((r.y+r.height)/2)), r.width, r.height));
        int ancho=((r.y+r.height)/2);
        cout<<" Valor height:" << r.height <<endl;
        cout<<" Valor ancho:" << ancho <<endl;

        cv::rectangle( imaBGR, r, cv::Scalar(0,255,0), 2);
        cv::imshow("Despues Tracker",imaBGR);*/
		this->roiToTrack = r;
    	return true;
    }else
    {
    	return false;
    }

}

bool RoiTracker::Train(cv::Mat imaBGR, cv::Mat imaXYZ,bool& enableTrain,bool& enableTracker )
{

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

    success = false;

    std::vector< cv::Rect > rois = GetTrainRoisMultiscale( this->roiToTrack, imaBGR );

    for(int i=0; i< rois.size(); i++)
    {
        cv::Mat roiIma;
        try
        {
            roiIma = imaBGR( rois[i] );
        }
        catch(...)
        {
            std::cout << ">>>>> EXCEPTION!! at roiIma train" << std::endl;
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

        this->histoToTrack.push_back( histo.t() );
		Exper++;
    }

    if(this->noExperiences<Exper){
    	enableTrain=false;
    	enableTracker=true;
    }

	return success;
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
            std::cout << ">>>>> EXCEPTION!! at roiIma Update" << std::endl;
            continue;
        }

        cv::Mat histo;
        try
        {
        	cv::Mat Temp;
            Temp = CalculateHistogram( roiIma );
            histo.push_back(Temp.t());
        }
        catch(...)
        {
            std::cout << ">>>>> EXCEPTION!! at  CalculateHistogram" << std::endl;
            continue;
        }

        double match = CompareHist(histo);

        histos.push_back( histo );
        matches.push_back( match );

        if( match > bestMatch )
        {
            bestMatch = match;
            bestIndex = i;
        }
    }

    nextRoi = rois[bestIndex];


    if( bestMatch > this->matchThreshold )
    {
      if(confidence<bestMatch)
      {
        confidence = bestMatch;
        cout<<"-----------confidence Update: "<<confidence<<endl;
        this->roiToTrack = nextRoi;
        success = true;
      }else
      {
          success=false;
      }
    }
    else
    {
        nextRoi = this->roiToTrack;
        confidence = bestMatch;
        this->roiToTrack = this->roiToTrack;
        success = false;
    }


    return success;
}

dlib::rectangle RoiTracker::openCVRectToDlib(cv::Rect r)
{
 return dlib::rectangle((long)r.tl().x, (long)r.tl().y, (long)r.br().x - 1, (long)r.br().y - 1);
}
  cv::Rect RoiTracker::dlibRectangleToOpenCV(dlib::rectangle r)
{
  return cv::Rect(cv::Point2i(r.left(), r.top()), cv::Point2i(r.right() + 1, r.bottom() + 1));
}

bool RoiTracker::UpdateROI(cv::Mat imaBGR, cv::Mat imaXYZ, cv::Rect& nextRoi, double& confidence)
{
    confidence = 0.0;

int NumberSearch=3;
int CountSearch=0;

int PosxRandom;
int PosyRandom;

cv::Rect roiToTrackTemp;

roiToTrackTemp=this->roiToTrack;


  while(CountSearch<=NumberSearch)
  {
  	PosxRandom=10+std::rand()%(621-1);  //10-300
  	PosyRandom=10+std::rand()%(401-1);

  	roiToTrackTemp.x=PosxRandom;
  	roiToTrackTemp.y=PosyRandom;

  	CountSearch++;
  	success = true;
    if( this->roiToTrack.size() == cv::Size() )
    {
        std::cout << "ERROR!!! roiToTrack.size is equal to  0" << std::endl;
        success = false;
    }
    if( this->roiToTrack.tl().x < 0 || this->roiToTrack.tl().y >= imaBGR.cols )
    {
        std::cout << "ERROR!!! roiToTrack.tl outside of image" << std::endl;
        success = false;
    }
    if( this->roiToTrack.br().x < 0 || this->roiToTrack.br().y >= imaBGR.rows )
    {
        std::cout << "ERROR!!! roiToTrack.br outside of image" << std::endl;
        success = false;
    }
    if(success != false)
    {

	    std::vector< cv::Rect > rois = GetSearchRoisMultiscale( roiToTrackTemp, imaBGR );

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
	            std::cout << ">>>>> EXCEPTION!! at roiIma Update" << std::endl;
	            continue;
	        }

	        cv::Mat histo;
	        try
	        {
	        	cv::Mat Temp;
	            Temp = CalculateHistogram( roiIma );
	            histo.push_back(Temp.t());
	        }
	        catch(...)
	        {
	            std::cout << ">>>>> EXCEPTION!! at  CalculateHistogram" << std::endl;
	            continue;
	        }

	        double match = CompareHist(histo);

	        histos.push_back( histo );
	        matches.push_back( match );

	        if( match > bestMatch )
	        {
	            bestMatch = match;
	            bestIndex = i;
	        }
	    }
      //cout<<"-----------bestmach ROI: "<<bestMatch<<endl;
      //cout<<"-----------matchThreshold ROI: "<<this->matchThreshold<<endl;

	    if( bestMatch > this->matchThreshold )
	    {
        if( confidence <  bestMatch  )
        {
            confidence = bestMatch;
            //std::cout  << "-----------confidence ROI: " << confidence << endl;

  	        nextRoi = rois[bestIndex];
  	        confidence = bestMatch;
  	        this->roiToTrack = nextRoi;
  	        success = true;
            return success;
        }
	    }
	    else
	    {
	        nextRoi = this->roiToTrack;
	        confidence = bestMatch;
	        this->roiToTrack = this->roiToTrack;
	        success = false;

	    }
    }
  }

    return success;
}



double RoiTracker::CompareHist(cv::Mat &Histo){

	double bestMatch=0.0;

	for(int i=0;i<this->histoToTrack.rows;i++){
		double match = cv::compareHist( this->histoToTrack.row(i), Histo, cv::HISTCMP_INTERSECT );

		    if( bestMatch < match )
		    {
		    	bestMatch=match;
		    }
	}

	return bestMatch;
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

    int overlapWidth_inPixels    =  (int)( ((double)centerRoi.size().width)   * overPercWidth );
    int overlapHeight_inPixels   =  (int)( ((double)centerRoi.size().height)  * overPercHeight );

    int noOverlapWidth_inPixels     = centerRoi.size().width    - overlapWidth_inPixels;
    int noOverlapHeight_inPixels    = centerRoi.size().height   - overlapHeight_inPixels;

    int firstCol =  centerRoi.tl().x - noOverlapWidth_inPixels   * overNoRectsWidth;
    int firstRow =  centerRoi.tl().y - noOverlapHeight_inPixels  * overNoRectsHeight;



    int lastCol =   firstCol + noOverlapWidth_inPixels  * (overNoRectsWidth  * 2 + 1);
    int lastRow =   firstRow + noOverlapHeight_inPixels * (overNoRectsHeight * 2 + 1);


    for(int i=firstRow; i<lastRow; i=i+noOverlapHeight_inPixels)
    {
        for(int j=firstCol; j<lastCol; j=j+noOverlapWidth_inPixels)
        {
            cv::Point topLeft( j , i );
            cv::Size size = centerRoi.size();
            cv::Rect rect = cv::Rect( topLeft , size );

            if( rect.tl().x <= 0 || rect.tl().x >= bgrIma.cols )
                continue;
            if( rect.tl().y <= 0 || rect.tl().y >= bgrIma.rows )
                continue;
            if( rect.br().x <= 0 || rect.br().x >= bgrIma.cols )
                continue;
            if( rect.br().y <= 0 || rect.br().y >= bgrIma.rows )
                continue;

            rois.push_back( rect );
        }
    }

    return rois;
}

std::vector< cv::Rect > RoiTracker::GetSearchRoisMultiscale( cv::Rect centerRoi, cv::Mat bgrIma )
{
    cv::Mat searchRois;

    if( Debug )
    {
        searchRois = bgrIma.clone();
    }

    cv::Size scaleIncrement;
    cv::Size scaleDecrement;
    scaleIncrement.width = (int)(((double)centerRoi.width) * this->scaleFactorIncrement);
    scaleIncrement.height = (int)(((double)centerRoi.height) * this->scaleFactorIncrement);

    scaleDecrement.width = (int)(((double)centerRoi.width) * this->scaleFactorDecrement);
    scaleDecrement.height = (int)(((double)centerRoi.height) * this->scaleFactorDecrement);

    cv::Point centerPoint = centerRoi.tl() + cv::Point( (int)(((double)centerRoi.width) / 2.0) , (int)(((double)centerRoi.height) / 2.0) );

    // Adding original roi
    std::vector< cv::Rect > centerRois;
    centerRois.push_back( centerRoi ); // First Roi

    cv::Rect roi;
    // UPSCALING
    roi = centerRoi;
    cv::Size size; 	// this change
    for( int i =0;  i<this->scaleSteps; i++)
    {
        size = roi.size() + scaleIncrement * i;
        cv::Point tl = centerPoint - cv::Point( (int)(((double)size.width) / 2.0) , (int)(((double)size.height) / 2.0) );

        roi = cv::Rect( tl, size );

        if( roi.size().width >= this->scaleMax.width || roi.size().height >= this->scaleMax.height )
            break;

        centerRois.push_back( roi );
    }

    // DOWNSCALING
    roi = centerRoi;
    for( int i =1;  i<this->scaleSteps; i++)
    {
        size = roi.size() - scaleDecrement * i;
        cv::Point tl = centerPoint - cv::Point( (int)(((double)size.width) / 2.0) , (int)(((double)size.height) / 2.0) );

        roi = cv::Rect( tl, size );

        if( roi.size().width <= this->scaleMin.width || roi.size().height <= this->scaleMin.height )
            break;

        centerRois.push_back( roi );
    }


    std::vector< cv::Rect > rois;
    for( int k=0; k<centerRois.size(); k++)
    {
        centerRoi = centerRois[k];

        std::vector< cv::Rect > scaleRois = GetSearchRois( centerRoi , bgrIma );
        rois.insert( rois.end(), scaleRois.begin(), scaleRois.end());

        if( Debug )
        {
            for( int i=0; i<scaleRois.size(); i++)
                cv::rectangle( searchRois, scaleRois[i], cv::Scalar(0,0,0), 2);

            cv::imshow( "SearchRoisMultiscale", searchRois );
        }
    }

    return rois;

}

std::vector< cv::Rect > RoiTracker::GetTrainRoisMultiscale( cv::Rect centerRoi, cv::Mat bgrIma )
{
    cv::Mat searchRois;
    cv::Mat centerRoisIma;
    if( Debug )
    {
        searchRois = bgrIma.clone();
    }

    cv::Size scaleIncrement;
    cv::Size scaleDecrement;
    scaleIncrement.width = (int)(((double)centerRoi.width) * this->scaleFactorIncrement);
    scaleIncrement.height = (int)(((double)centerRoi.height) * this->scaleFactorIncrement);

    scaleDecrement.width = (int)(((double)centerRoi.width) * this->scaleFactorDecrement);
    scaleDecrement.height = (int)(((double)centerRoi.height) * this->scaleFactorDecrement);

    cv::Point centerPoint = centerRoi.tl() + cv::Point( (int)(((double)centerRoi.width) / 2.0) , (int)(((double)centerRoi.height) / 2.0) );

    // Adding original roi
    std::vector< cv::Rect > centerRois;
    centerRois.push_back( centerRoi ); // First Roi

    // Creating center Rois.
    cv::Rect roi;
    // UPSCALING
    roi = centerRoi;
    cv::Size size;
    for( int i =1;  i<this->scaleSteps; i++)
    {
        size = roi.size() + scaleIncrement * i;
        cv::Point tl = centerPoint - cv::Point( (int)(((double)size.width) / 2.0) , (int)(((double)size.height) / 2.0) );

        roi = cv::Rect( tl, size );

        if( roi.size().width >= this->scaleMax.width || roi.size().height >= this->scaleMax.height )
            break;

        centerRois.push_back( roi );
    }

    // DOWNSCALING
    roi = centerRoi;

    for( int i =1;  i<this->scaleSteps; i++)
    {
        size = roi.size() - scaleDecrement*i;
        cv::Point tl = centerPoint - cv::Point( (int)(((double)size.width) / 2.0) , (int)(((double)size.height) / 2.0) );

        roi = cv::Rect( tl, size );

        if( roi.size().width <= this->scaleMin.width || roi.size().height <= this->scaleMin.height )
            break;

        centerRois.push_back( roi );
    }

    std::vector< cv::Rect > rois;
    for( int k=0; k<centerRois.size(); k++)
    {
        centerRoi = centerRois[k];

        std::vector< cv::Rect > scaleRois = GetSearchRois( centerRoi , bgrIma );
        rois.insert( rois.end(), scaleRois.begin(), scaleRois.end());

        if( Debug )
        {
            for( int i=0; i<scaleRois.size(); i++)
                cv::rectangle( searchRois, scaleRois[i], cv::Scalar(255,0,0), 2);

            cv::imshow( "centerRoisIma_Train", searchRois );
        }
    }

    return rois;
    //return centerRois;
}
