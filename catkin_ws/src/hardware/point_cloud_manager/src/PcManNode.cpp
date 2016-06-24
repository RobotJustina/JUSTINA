#include "PcManNode.h"

PcManNode::PcManNode():
    cloudRobot(new pcl::PointCloud<pcl::PointXYZRGBA>)//, viewer("POINT CLOUD MANAGER By Marcosoft")
{
    this->saveCloud = false;
    this->cloudFilePath = "";
}

PcManNode::~PcManNode()
{
}

bool PcManNode::InitNode(ros::NodeHandle* n, std::string default_path)
{
    //ROS Initialization
    this->default_path = default_path;
    if(!boost::algorithm::ends_with(this->default_path, "/"))
        this->default_path += "/";
    this->n = n;
    this->pubKinectFrame = n->advertise<sensor_msgs::PointCloud2>("/hardware/point_cloud_man/rgbd_wrt_kinect",1);
    this->pubRobotFrame = n->advertise<sensor_msgs::PointCloud2>("/hardware/point_cloud_man/rgbd_wrt_robot", 1);
    this->pubRobotFrameDownsampled = n->advertise<sensor_msgs::PointCloud2>("/hardware/point_cloud_man/rgbd_wrt_robot_downsampled", 1);
    this->subSavePointCloud = n->subscribe("/hardware/point_cloud_man/save_cloud", 1, &PcManNode::callback_save_cloud, this);
    this->subStopSavingCloud = n->subscribe("/hardware/point_cloud_man/stop_saving_cloud", 1, &PcManNode::callback_stop_saving_cloud, this);
    this->srvRgbdKinect = n->advertiseService("/hardware/point_cloud_man/get_rgbd_wrt_kinect", &PcManNode::kinectRgbd_callback, this);
    this->srvRgbdRobot = n->advertiseService("/hardware/point_cloud_man/get_rgbd_wrt_robot", &PcManNode::robotRgbd_callback, this);

    //Wait for transform
    tf_listener.waitForTransform("base_link", "kinect_link", ros::Time(0), ros::Duration(5.0));
    
    return true;
}

void PcManNode::spin()
{
    ros::Rate loop(30);
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
        ros::spinOnce();
        loop.sleep();
    }
    std::cout << "PointCloudMan.->Stopping kinect..." << std::endl;
    interface->stop();
}

void PcManNode::point_cloud_callback(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &c)
{           
    pcl::toROSMsg(*c, this->msgCloudKinect);
    this->msgCloudKinect.header.frame_id = "kinect_link";
    if(this->pubKinectFrame.getNumSubscribers() > 0)
    {
        this->pubKinectFrame.publish(this->msgCloudKinect);
    }
    if(this->pubRobotFrame.getNumSubscribers() > 0)
    {
        tf::StampedTransform transformTf;
        try{
            tf_listener.lookupTransform("base_link", "kinect_link", ros::Time(0), transformTf);
            Eigen::Affine3d transformEigen;
            tf::transformTFToEigen(transformTf, transformEigen);
            pcl::transformPointCloud(*c, *this->cloudRobot, transformEigen);
            pcl::toROSMsg(*this->cloudRobot, this->msgCloudRobot);
            this->msgCloudRobot.header.frame_id = "base_link";
            this->pubRobotFrame.publish(this->msgCloudRobot);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }
    if(this->pubRobotFrameDownsampled.getNumSubscribers() > 0)
    {
        try{
            pcl::PointCloud<pcl::PointXYZRGBA> downsampled;
            sensor_msgs::PointCloud2 msgDownsampled;
            downsampled.width = c->width/3;
            downsampled.height = c->height/3;
            downsampled.is_dense = c->is_dense;
            downsampled.points.resize(downsampled.width*downsampled.height);
            for(int i=0; i < downsampled.width; i++)
                for(int j=0; j < downsampled.height; j++)
    	      downsampled.points[j*downsampled.width + i] = c->points[3*(j*c->width + i)];
            tf::StampedTransform transformTf;
            tf_listener.lookupTransform("base_link", "kinect_link", ros::Time(0), transformTf);
            Eigen::Affine3d transformEigen;
            tf::transformTFToEigen(transformTf, transformEigen);
            pcl::transformPointCloud(downsampled, downsampled, transformEigen);
            pcl::toROSMsg(downsampled, msgDownsampled);
            msgDownsampled.header.frame_id = "base_link";
            this->pubRobotFrameDownsampled.publish(msgDownsampled);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }
    if(this->saveCloud)
        pcl::io::savePCDFileBinary(this->cloudFilePath, *c);
}

bool PcManNode::kinectRgbd_callback(point_cloud_manager::GetRgbd::Request &req, point_cloud_manager::GetRgbd::Response &resp)
{
    resp.point_cloud = this->msgCloudKinect;
}

bool PcManNode::robotRgbd_callback(point_cloud_manager::GetRgbd::Request &req, point_cloud_manager::GetRgbd::Response &resp)
{
    pcl_ros::transformPointCloud("base_link", this->msgCloudKinect, this->msgCloudRobot, tf_listener);
    this->msgCloudRobot.header.frame_id = "base_link";
    resp.point_cloud = this->msgCloudRobot;
}

void PcManNode::callback_save_cloud(const std_msgs::String::ConstPtr& msg)
{
    if(!boost::filesystem::portable_posix_name(msg->data))
    {
        std::cout << "PointCloudMan.->Cannot save point cloud: Invalid file name. " << std::endl;
        return;
    }
    if(boost::algorithm::ends_with(msg->data, ".pcd"))
        this->cloudFilePath = msg->data;
    else
        this->cloudFilePath = msg->data + ".pcd";

    this->cloudFilePath = this->default_path + this->cloudFilePath;
    std::cout << "PointCloudMan.->Start saving point cloud at: " << this->cloudFilePath << std::endl;
    this->saveCloud = true;
}

void PcManNode::callback_stop_saving_cloud(const std_msgs::Empty::ConstPtr& msg)
{
    std::cout << "PointCloudMan.->Stop saving point cloud" << std::endl;
    this->saveCloud = false;
}
