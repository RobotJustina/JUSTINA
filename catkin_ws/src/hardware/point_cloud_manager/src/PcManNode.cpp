#include "PcManNode.h"

PcManNode::PcManNode()
    //cloudKinect()//, viewer("POINT CLOUD MANAGER By Marcosoft")
{
    this->saveCloud = false;
    this->cloudFilePath = "";
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
    this->subSavePointCloud = n->subscribe("/hardware/point_cloud_man/save_cloud", 1, &PcManNode::callback_save_cloud, this);
    this->subStopSavingCloud = n->subscribe("/hardware/point_cloud_man/stop_saving_cloud", 1, &PcManNode::callback_stop_saving_cloud, this);
    this->srvRgbdKinect = n->advertiseService("/hardware/point_cloud_man/get_rgbd_wrt_kinect", &PcManNode::kinectRgbd_callback, this);
    this->srvRgbdRobot = n->advertiseService("/hardware/point_cloud_man/get_rgbd_wrt_robot", &PcManNode::robotRgbd_callback, this);

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
    std::cout << "PointCloudMan.->Msg: " << this->msgCloudKinect.width << "  " << this->msgCloudKinect.height << std::endl;
    //Kinect Initialization
    std::cout << "PointCloudMan.->Trying to initialize kinect..." << std::endl;
    pcl::Grabber* interface = new pcl::OpenNIGrabber();
    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind (&PcManNode::point_cloud_callback, this, _1);
    interface->registerCallback(f);
    interface->start();
    std::cout << "PointCloudMan.->Kinect initialized succesfully :D" << std::endl;
    
    while(ros::ok())
	{
        loop.sleep();
        ros::spinOnce();
    }
    std::cout << "PointCloudMan.->Stopping kinect..." << std::endl;
    interface->stop();
}

void PcManNode::point_cloud_callback(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &c)
{
    if(this->pubKinectFrame.getNumSubscribers() > 0)
    {
        //std::cout << "PointCloudMan.->Publishing cloud wrt kinect" << std::endl;
        pcl::toROSMsg(*c, this->msgCloudKinect);
        this->msgCloudKinect.header.frame_id = "kinect_link";
        this->pubKinectFrame.publish(this->msgCloudKinect);
    }
    if(this->pubRobotFrame.getNumSubscribers() > 0)
    {
        tf::StampedTransform transformTf;
        tf_listener.lookupTransform(baseFrame, kinectFrame, ros::Time(0), transformTf);
        Eigen::Affine3d transformEigen;
        tf::transformTFToEigen(transformTf, transformEigen);

        pcl::transformPointCloud(*c, *this->cloudRobot, transformEigen);
        //this->cloudRobot->header.frame_id = "base_link";
        //pcl::toROSMsg(*this->cloudRobot, this->msgCloudRobot);

        //this->pubKinectFrame.publish(this->msgCloudRobot);
    }
    if(this->saveCloud)
        pcl::io::savePCDFileBinary(this->cloudFilePath, *c);
}

bool PcManNode::kinectRgbd_callback(point_cloud_manager::get_rgbd::Request &req, point_cloud_manager::get_rgbd::Response &resp)
{
    if(!this->cloudKinect)
        return false;
    pcl::toROSMsg(*this->cloudKinect, this->msgCloudKinect);
    resp.point_cloud = this->msgCloudKinect;
}

bool PcManNode::robotRgbd_callback(point_cloud_manager::get_rgbd::Request &req, point_cloud_manager::get_rgbd::Response &resp)
{
    if(!this->cloudKinect)
        return false;
    tf::StampedTransform transformTf;
    tf_listener.lookupTransform(baseFrame, kinectFrame, ros::Time(0), transformTf);
    Eigen::Affine3d transformEigen;
    tf::transformTFToEigen(transformTf, transformEigen);
    
    pcl::transformPointCloud(*this->cloudKinect, *this->cloudRobot, transformEigen);
    this->cloudRobot->header.frame_id = baseFrame;
    pcl::toROSMsg(*this->cloudRobot, this->msgCloudRobot);
    
    resp.point_cloud = this->msgCloudRobot;
}

void PcManNode::callback_save_cloud(const std_msgs::String::ConstPtr& msg)
{
    if(msg->data.compare("") == 0)
    {
        std::cout << "PointCloudMan.->Cannot save point cloud: Invalid file name. " << std::endl;
        return;
    }
    if(boost::algorithm::ends_with(msg->data, ".pcd"))
        this->cloudFilePath = msg->data;
    else
        this->cloudFilePath = msg->data + ".pcd";
    this->saveCloud = true;
}

void PcManNode::callback_stop_saving_cloud(const std_msgs::Empty::ConstPtr& msg)
{
    this->saveCloud = false;
}
