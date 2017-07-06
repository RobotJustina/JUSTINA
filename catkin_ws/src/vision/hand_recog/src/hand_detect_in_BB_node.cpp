#include "ros/ros.h"

#include "std_msgs/Bool.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud2.h"

#include "justina_tools/JustinaTools.h"

ros::Subscriber subPointCloud;
ros::NodeHandle * nh_ptr;
geometry_msgs::Point32 refPoint;
bool enableDetect = false;
bool detected = false;

bool initThreshold = false;
//This is the for hardcode for the optimal PCL in bounding box
//int threshhold = 5000;
int threshold = 0;

// NEAREST_DETECT
bool enaNearestDetect = false; 
bool debug = true; 
cv::Scalar frontLeftBotBBPoint = cv::Scalar(0.25, -0.5, 0.7);
cv::Scalar backRigthTopBBPoint = cv::Scalar(1.25,  0.5, 2.0); 


typedef struct BoundingBox {
	float w, h, l;
	cv::Point3f center;
} BoundingBox;

geometry_msgs::Point32 NeaerestDetect(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    cv::Mat imaBGR; 
    cv::Mat imaXYZ; 
	JustinaTools::PointCloud2Msg_ToCvMat(msg, imaBGR, imaXYZ);

    geometry_msgs::Point32 edgePoint; 

    // Segmenting by BBox 
    cv::Mat validMask; 
    cv::inRange(imaXYZ, frontLeftBotBBPoint, backRigthTopBBPoint, validMask);  

    cv::Mat validXYZ;
    imaXYZ.copyTo( validXYZ, validMask);
    int pclCount = countNonZero( validMask );

    cv::Mat bgrWithMask;
    imaBGR.copyTo( bgrWithMask, validMask ); 

    cv::Mat bgrCanny;
    int thresh = 100;
    int max_thresh = 255;
    cv::RNG rng(12345);
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

  // RGB2GRAY ?
  // cvtColor( bgrWithMask, bgrWithMask, CV_BGR2GRAY );
  // Canny Edges
  //Canny( bgrWithMask, bgrCanny, thresh, max_thresh, 3 );
  // Contours
  findContours( validMask, contours, cv::RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

  // Draw contours
  cv::Mat drawing = cv::Mat::zeros( validMask.size(), CV_8UC3 );
  for( int i = 0; i< contours.size(); i++ )
     {
       cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contours, i, color, 2 );//, 8, 0, cv::Point() );
     }


    if( debug ) 
    {
	imshow( "Contours", drawing );
//        cv::Mat bgrWithMask;
//        imaBGR.copyTo( bgrWithMask, validMask ); 
        cv::imshow( "bgrWithMask", bgrWithMask );
        std::cout << "Numbers of pts in BBox: " << pclCount << std::endl; 
    }

    return edgePoint; 
}

void callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {

    if( enaNearestDetect )
        NeaerestDetect( msg ); 
    
    cv::Mat imaBGR;
	cv::Mat imaPCL;
	int pclCount = 0;

	cv::Point3f handRobotPosition;
	handRobotPosition.x = refPoint.x;
	handRobotPosition.y = refPoint.y;
	handRobotPosition.z = refPoint.z;

	BoundingBox bb;
	bb.center = handRobotPosition;
	bb.w = 0.15;
	bb.h = 0.3;
	bb.l = 0.45;

	JustinaTools::PointCloud2Msg_ToCvMat(msg, imaBGR, imaPCL);
	for (int i = 0; i < imaPCL.rows; i++) {
		for (int j = 0; j < imaPCL.cols; j++) {
			cv::Point3f point = imaPCL.at<cv::Point3f>(i, j);
			if (point.x >= bb.center.x 
					&& point.x <= bb.center.x + bb.l / 2
					&& point.y >= bb.center.y - bb.w / 2
					&& point.y <= bb.center.y + bb.w / 2
					&& point.z >= bb.center.z - bb.h / 2
					&& point.z <= bb.center.z + bb.h / 2) {
				pclCount++;
			} else {
				imaBGR.at<cv::Vec3b>(i, j) = cv::Vec3b(0.0, 0.0, 0.0);
			}
		}
	}

	if(!initThreshold && pclCount > 300){
		initThreshold = true;
		threshold = pclCount;
		std::cout << "HandDetect.->threshhold:" << threshold << std::endl; 
		return;
	}

	cv::imshow("Hand Detect", imaBGR);
	std::cout << "HandDetect.->Number of pcl in BB:" << pclCount << std::endl;

	if (pclCount > 1.75 * threshold) {
		detected = true;
		std::cout << "HandDetect.->The Bounding box is fill" << std::endl;
	}
	else
		detected = false;


}

void callbackStartRecog(const geometry_msgs::Point32::ConstPtr& msg) {
	std::cout << "HandDetect.->Starting Hand Detect in BB..."
			<< std::endl;
	subPointCloud = nh_ptr->subscribe(
			"/hardware/point_cloud_man/rgbd_wrt_robot", 1, callbackPointCloud);
	refPoint.x = msg->x;
	refPoint.y = msg->y;
	refPoint.z = msg->z;
	enableDetect = true;
	detected = false;
	initThreshold = false;
	threshold = 0;

    enaNearestDetect = true;
}

void callbackStopRecog(const std_msgs::Empty::ConstPtr& msg) {
	std::cout << "HandDetect.->Stoping Hand Detect in BB..."
			<< std::endl;
	subPointCloud.shutdown();
	cv::destroyAllWindows();
	refPoint.x = 0.0;
	refPoint.y = 0.0;
	refPoint.z = 0.0;
	enableDetect = false;
	detected = false;
	initThreshold = true;
	threshold = 0;

    enaNearestDetect = false; 
}




int main(int argc, char ** argv) {
  std::cout << "INITIALIZING NODE hand_detect_in_BB"
			<< std::endl;
	ros::init(argc, argv, "hand_detect_in_BB_node");

	ros::NodeHandle n;
	nh_ptr = &n;

	ros::Subscriber subStartRecog = n.subscribe(
			"/vision/hand_detect_in_bb/start_recog", 1, callbackStartRecog);
	ros::Subscriber subStopRecog = n.subscribe(
			"/vision/hand_detect_in_bb/stop_recog", 1, callbackStopRecog);
	ros::Publisher pubHandInFront = n.advertise<std_msgs::Bool>(
			"/vision/hand_detect_in_bb/hand_in_front", 1);

    ros::Publisher pub_nearestDetect = n.advertise< geometry_msgs::Point32 >( "vision/hand_detect_in_BB/nearest_detect", 1);   
	
    /*subPointCloud = n.subscribe("/hardware/point_cloud_man/rgbd_wrt_robot", 1,
	 callbackPointCloud);*/

	ros::Rate rate(15);

	int key = 0;
	while (ros::ok() && key != 27) {

		if(enableDetect){
			std_msgs::Bool msg;
			msg.data = detected;
			pubHandInFront.publish(msg);
		}

		rate.sleep();
		ros::spinOnce();
		key = cvWaitKey(10);
	}

	return true;
}
