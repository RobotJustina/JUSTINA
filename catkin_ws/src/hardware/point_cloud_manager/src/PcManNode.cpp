#include "PcManNode.h"

PcManNode::PcManNode()
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
    this->interface = new pcl::OpenNIGrabber();
    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind (&PcManNode::point_cloud_callback, this, _1);
    this->interface->registerCallback(f);
    this->interface->start();

    //Wait for transform
    this->kinectFrame = "kinect_link";
    this->baseFrame = "base_link";
    tf_listener.waitForTransform(baseFrame, kinectFrame, ros::Time(0), ros::Duration(5.0));
    
    return true;
}

void PcManNode::spin()
{
    ros::Rate loop(20);
    tf::StampedTransform transformTf;
    while(ros::ok())
	{
		if(this->pubKinectFrame.getNumSubscribers() > 0)
        {
            pcl::toROSMsg(*this->cloudKinect, this->msgCloudKinect);
            this->pubKinectFrame.publish(this->msgCloudKinect);
        }
        if(this->pubRobotFrame.getNumSubscribers() > 0)
        {
            tf_listener.lookupTransform(baseFrame, kinectFrame, ros::Time(0), transformTf);
            Eigen::Affine3d transformEigen;
            //tf::transformTFToEigen(transformTf, transformEigen);

            pcl::transformPointCloud(*this->cloudKinect, *this->cloudRobot, transformEigen);
            this->cloudRobot->header.frame_id = baseFrame;
            pcl::toROSMsg(*this->cloudRobot, this->msgCloudRobot);

            this->pubKinectFrame.publish(this->msgCloudRobot);
        }
        loop.sleep();
        ros::spinOnce();
    }
}

void PcManNode::point_cloud_callback(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &c)
{
    this->cloudKinect = c;
}

bool PcManNode::kinectRgbd_callback(point_cloud_manager::get_rgbd::Request &req, point_cloud_manager::get_rgbd::Response &res)
{
    pcl::toROSMsg(*this->cloudKinect, this->msgCloudKinect);
    this->pubKinectFrame.publish(this->msgCloudKinect);
}

bool PcManNode::robotRgbd_callback(point_cloud_manager::get_rgbd::Request &req, point_cloud_manager::get_rgbd::Response &res)
{
    tf::StampedTransform transformTf;
    tf_listener.lookupTransform(baseFrame, kinectFrame, ros::Time(0), transformTf);
    Eigen::Affine3d transformEigen;
    //tf::transformTFToEigen(transformTf, transformEigen);
    
    pcl::transformPointCloud(*this->cloudKinect, *this->cloudRobot, transformEigen);
    this->cloudRobot->header.frame_id = baseFrame;
    pcl::toROSMsg(*this->cloudRobot, this->msgCloudRobot);
    
    this->pubKinectFrame.publish(this->msgCloudRobot);
}
