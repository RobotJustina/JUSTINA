#include "PcManNode.h"

PcManNode::PcManNode():capture(CV_CAP_OPENNI)
{
}

PcManNode::~PcManNode()
{
}

bool PcManNode::InitNode(ros::NodeHandle* n, bool debugMode)
{
    //ROS Initialization
    this->debugMode = debugMode;
    this->n = n;
    this->pubKinectFrame = n->advertise<sensor_msgs::PointCloud2>("/hardware/point_cloud_man/rgbd_wrt_kinect",1);
    this->pubRobotFrame = n->advertise<sensor_msgs::PointCloud2>("/hardware/point_cloud_man/rgbd_wrt_robot", 1);
    this->srvRgbdKinect = n->advertiseService("/hardware/point_cloud_man/get_rgbd_wrt_kinect", &PcManNode::kinectRgbd_callback, this);
    this->srvRgbdRobot = n->advertiseService("/hardware/point_cloud_man/get_rgbd_wrt_robot", &PcManNode::robotRgbd_callback, this);

    //Kinect Initialization
    std::cout << "PointCloudMan.-> Triying to initialize kinect sensor... " << std::endl;
	if(!this->capture.isOpened())
	{
		std::cout << "PointCloudMan.-> Cannot open kinect :'(" << std::endl;
		return false;
	}
	this->capture.set(CV_CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION, CV_CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION_ON);
    std::cout << "PointCloudMan.-> Kinect sensor started :D" << std::endl;
    return true;
}

void PcManNode::spin()
{
    while(ros::ok() && cv::waitKey(15) != 27)
	{
		this->capture.grab();
		this->capture.retrieve(this->depthMap, CV_CAP_OPENNI_POINT_CLOUD_MAP);
		this->capture.retrieve(this->bgrImage, CV_CAP_OPENNI_BGR_IMAGE);
        if(this->pubKinectFrame.getNumSubscribers() > 0)
        {
            //Transform cvMat to PointCloud2 and publish the message
        }
        if(this->pubRobotFrame.getNumSubscribers() > 0)
        {
            //Transform cvMat to PointCloud2 and transform to robot coordinates.
            //Use tf_listener to get the transformation
        }
        ros::spinOnce();
    }
}

sensor_msgs::PointCloud2 PcManNode::cvMat2PointCloud2(cv::Mat& srcBgr, cv::Mat& srcDepth)
{
}

bool PcManNode::kinectRgbd_callback(point_cloud_manager::get_rgbd::Request &req, point_cloud_manager::get_rgbd::Response &res)
{
    res.point_cloud = this->cvMat2PointCloud2(this->bgrImage, this->depthMap);
    return true;
}

bool PcManNode::robotRgbd_callback(point_cloud_manager::get_rgbd::Request &req, point_cloud_manager::get_rgbd::Response &res)
{
}
