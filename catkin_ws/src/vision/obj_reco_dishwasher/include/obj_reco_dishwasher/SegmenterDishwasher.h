#ifndef _SEGMENTADOR_H_
#define _SEGMENTADOR_H_

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"


using namespace cv;

class SegmenterDishwasher{

public:


    static Mat colorSegmentHSV(Mat bgr,Mat element, int minvalueH, int maxvalueH, int minvalueS, int maxvalueS, int minvalueV, int maxvalueV){

            Scalar minHSV(minvalueH,minvalueS,minvalueV);
            Scalar maxHSV(maxvalueH, maxvalueS, maxvalueV);

            return  colorSegmentHSV(bgr,element, minHSV, maxHSV);

        }

        static Mat colorSegmentHSV(Mat bgr,Mat element, Scalar minHSV, Scalar maxHSV)
        {
            Mat hsv;
            Mat maskHSV;

            cvtColor(bgr, hsv, CV_BGR2HSV);
            inRange(hsv, minHSV, maxHSV, maskHSV);
            dilate(maskHSV,maskHSV, element);
            erode(maskHSV,maskHSV, element);

            return maskHSV;

        }


        static Mat colorSegmentHLS(Mat bgr,Mat element, int minvalueH, int maxvalueH, int minvalueL, int maxvalueL, int minvalueS, int maxvalueS)
        {
            Scalar minHLS(minvalueH,minvalueL,minvalueS);
            Scalar maxHLS(maxvalueH, maxvalueL, maxvalueS);

            return  SegmenterDishwasher::colorSegmentHLS(bgr,element, minHLS, maxHLS);
        }



        static Mat colorSegmentHLS(Mat bgr,Mat element, Scalar minHLS, Scalar maxHLS)
        {
            Mat hls;
            Mat maskHLS;

            cvtColor(bgr, hls, CV_BGR2HLS);
            inRange(hls, minHLS, maxHLS, maskHLS);
            dilate(maskHLS,maskHLS, element);
            erode(maskHLS,maskHLS, element);

            return maskHLS;
        }



        static Mat colorSegmentH(Mat bgr,Mat element, int minvalueH, int maxvalueH)
        {

            Mat hls_s[3];   //destination array
            Mat hls;
            Mat h;
            Mat maskH;

            cvtColor(bgr, hls, CV_BGR2HLS);
            split(hls,hls_s);//split source

            h=hls_s[0];

            inRange(h, minvalueH, maxvalueH,maskH);
			dilate(maskH,maskH, element);
			erode(maskH,maskH, element);
			dilate(maskH,maskH, element);
			erode(maskH,maskH, element);
			dilate(maskH,maskH, element);

            return maskH;
        }


        static Mat colorSegmentBGR(Mat bgr,Mat element, int minvalueB, int maxvalueB, int minvalueG, int maxvalueG, int minvalueR, int maxvalueR)
        {
            Scalar minBGR(minvalueB,minvalueG,minvalueR);
            Scalar maxBGR(maxvalueB, maxvalueG, maxvalueR);

            return  SegmenterDishwasher::colorSegmentBGR(bgr,element, minBGR, maxBGR);;
        }


        static Mat colorSegmentBGR(Mat bgr,Mat element, Scalar minBGR, Scalar maxBGR )
        {
            Mat maskBGR;

            inRange(bgr, minBGR, maxBGR, maskBGR);
            dilate(maskBGR,maskBGR, element);
            erode(maskBGR,maskBGR, element);

            return maskBGR;
        }
};



#endif
