#include "ros/ros.h"

#include "std_msgs/Bool.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud2.h"

#include "justina_tools/JustinaTools.h"

ros::Subscriber subPointCloud;
ros::NodeHandle * nh_ptr;
geometry_msgs::Point32 refPoint;
int threshhold = 5000;
bool enableDetect = false;
bool detected = false;

typedef struct BoundingBox {
	float w, h, l;
	cv::Point3f center;
} BoundingBox;

void callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
	cv::Mat imaBGR;
	cv::Mat imaPCL;
	int pcl = 0;

	cv::Point3f handRobotPosition;
	handRobotPosition.x = refPoint.x;
	handRobotPosition.y = refPoint.y;
	handRobotPosition.z = refPoint.z;

	BoundingBox bb;
	bb.center = handRobotPosition;
	bb.w = 0.15;
	bb.h = 0.2;
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
				pcl++;
			} else {
				imaBGR.at<cv::Vec3b>(i, j) = cv::Vec3b(0.0, 0.0, 0.0);
			}
		}
	}
	cv::imshow("Hand Detect", imaBGR);
	std::cout << "HandOverRecognition.->Number of pcl in BB:" << pcl << std::endl;

	if (pcl > threshhold) {
		detected = true;
		std::cout << "The Bounding box is fill" << std::endl;
	}
	else
		detected = false;
}

void callbackStartRecog(const geometry_msgs::Point32::ConstPtr& msg) {
	std::cout << "HandoverRecognition.->Starting Hand Detect in BB..."
			<< std::endl;
	subPointCloud = nh_ptr->subscribe(
			"/hardware/point_cloud_man/rgbd_wrt_robot", 1, callbackPointCloud);
	refPoint.x = msg->x;
	refPoint.y = msg->y;
	refPoint.z = msg->z;
	enableDetect = true;
	detected = false;
}

void callbackStopRecog(const std_msgs::Empty::ConstPtr& msg) {
	std::cout << "HandoverRecognition.->Stoping Hand Detect in BB..."
			<< std::endl;
	subPointCloud.shutdown();
	cv::destroyAllWindows();
	refPoint.x = 0.0;
	refPoint.y = 0.0;
	refPoint.z = 0.0;
	enableDetect = false;
	detected = false;
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
