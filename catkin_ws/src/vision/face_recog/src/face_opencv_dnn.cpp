#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <map>
#include <stdlib.h>
#include <cstdlib>
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include <ros/package.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "justina_tools/JustinaTools.h"
#include "geometry_msgs/Point.h"
#include "tf/transform_listener.h"
#include <cv_bridge/cv_bridge.h>

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/dnn.hpp"

using namespace cv;
using namespace std;
using namespace cv::dnn;

const size_t inWidth = 300;
const size_t inHeight = 300;
const double inScaleFactor = 1.0;
const Scalar meanVal(104.0, 177.0, 123.0);



const char* params
    = "{ proto          | deploy.prototxt | model configuration }"
      "{ model          | res10_300x300_ssd_iter_140000.caffemodel | model weights }"
;
 
int main(int argc, char** argv) {

    CommandLineParser parser(argc, argv, params);
    ros::init(argc, argv, "face_opencv_dnn");
    ros::NodeHandle n;
    VideoCapture stream1(0);   //0 is the id of video device.0 if you have only one camera.
    ros::Rate loop(30);


    

    String modelConfiguration = parser.get<string>("proto");
    String modelBinary = parser.get<string>("model");


    //Initialize the Network
    dnn::Net net = readNetFromCaffe(modelConfiguration, modelBinary);



    if (!stream1.isOpened()) { //check if video device has been initialised
        cout << "cannot open camera";
    }


    Mat cameraFrame, blob;
    //unconditional loop
    while(ros::ok() && cv::waitKey(1) != 'q') {
        
        if (cameraFrame.channels() == 4)
            cvtColor(cameraFrame, cameraFrame, COLOR_BGRA2BGR);

        stream1.read(cameraFrame);
        imshow("cam", cameraFrame);

        //cv::dnn::blobFromImage(cameraFrame, blob, 1.0f, Size(224, 224), Scalar(104, 117, 123), false);
        Mat inputBlob = blobFromImage(cameraFrame, 1.0f, Size(224, 224),Scalar(104, 117, 123));


        ros::spinOnce();
        loop.sleep();
    }
    cv::destroyAllWindows();   
}