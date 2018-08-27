// Standard includes
#include <algorithm>
#include <iterator>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/video.hpp"

#include <iostream>
#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Transform.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include "tf/tf.h"
#include "tf/tfMessage.h"


// NAMESPACEs
    using namespace cv;
    using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "send_image");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/camera/rgb", 1);


/**VARIABLES MAT*/
    Mat source;
    Mat output;
    Mat descriptorsTemp;
    Mat frame, edges;
    cv::startWindowThread();

    ros::Rate loop(20);
    VideoCapture cap(1);


    while (nh.ok())
    {

      cap>>frame;
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

      if(pub.getNumSubscribers() > 0)
      {
        cout<<"Send image."<<endl;
        pub.publish(msg);
      }

      ros::spinOnce();
      loop.sleep();
    }

    return 0;
}
