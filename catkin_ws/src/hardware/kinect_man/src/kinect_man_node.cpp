#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "point_cloud_manager/GetRgbd.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "tf/transform_listener.h"
#include "pcl_ros/transforms.h"
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

cv::Mat depthMap;
cv::Mat bgrImage;
tf::TransformListener* tf_listener;
sensor_msgs::PointCloud2::Ptr msgFromBag;
bool use_oni = false;
bool use_bag = false;
int downsample_by = 1;

ros::Publisher pubImageIROS;
ros::Publisher pubDepthIROS;

void initialize_rosmsg(sensor_msgs::PointCloud2& msg, int width, int height, std::string frame_id)
{
    msg.header.frame_id = frame_id;
    msg.width  = width;
    msg.height = height;
    msg.is_bigendian = false;
    msg.point_step   = 16;
    msg.row_step     = 16 * msg.width;
    msg.is_dense     = false;
    sensor_msgs::PointField f;
    f.name     = "x";
    f.offset   = 0;
    f.datatype = 7;
    f.count    = 1;
    msg.fields.push_back(f);
    f.name     = "y";
    f.offset   = 4;
    msg.fields.push_back(f);
    f.name     = "z";
    f.offset   = 8;
    msg.fields.push_back(f);
    f.name     = "rgba";
    f.offset   = 12;
    f.datatype = 6;
    msg.fields.push_back(f);
    msg.data.resize(msg.row_step * msg.height);
}

void cvmat_2_rosmsg(cv::Mat& depth, cv::Mat& bgr, sensor_msgs::PointCloud2& msg)
{
    //This function ONLY COPIES POINT DATA. For all headers, use initialize_msg();
    int idx = bgr.rows * bgr.cols;
    
    for(int i=0; i < idx; i++)
    {
        memcpy(&msg.data[i*16], &depth.data[12*i], 12);
        memcpy(&msg.data[i*16 + 12], &bgr.data[3*i], 3);
        msg.data[16*i + 15] = 255;
	float* y = (float*)&msg.data[16*i + 4];
        *y *= -1;
    }
}

void downsample_by_3(sensor_msgs::PointCloud2& src, sensor_msgs::PointCloud2& dst)
{
    for(int i=0; i < dst.width; i++)
        for(int j=0; j < dst.height; j++)
            memcpy(&dst.data[16*(j*dst.width + i)], &src.data[48*(j*src.width + i)], 16);
}

void downsample_pcl(sensor_msgs::PointCloud2& src, sensor_msgs::PointCloud2& dst, int downsample_by)
{
    for(int i=0; i < dst.width; i++)
        for(int j=0; j < dst.height; j++)
            memcpy(&dst.data[16*(j*dst.width + i)], &src.data[16 * downsample_by *(j*src.width + i)], 16);
}

bool kinectRgbd_callback(point_cloud_manager::GetRgbd::Request &req, point_cloud_manager::GetRgbd::Response &resp)
{
    if(!use_bag)
    {
        initialize_rosmsg(resp.point_cloud, 640, 480, "kinect_link");
        pubDepthIROS.publish(resp.point_cloud);
        cvmat_2_rosmsg(depthMap, bgrImage, resp.point_cloud);
        resp.point_cloud.header.stamp = ros::Time::now();
        return true;
    }
    else
    {
        if(msgFromBag == NULL) return false;
        resp.point_cloud = *msgFromBag;
        return true;
    }
}

bool robotRgbd_callback(point_cloud_manager::GetRgbd::Request &req, point_cloud_manager::GetRgbd::Response &resp)
{
    if(!use_bag)
    {
        initialize_rosmsg(resp.point_cloud, 640, 480, "kinect_link");
        pubDepthIROS.publish(resp.point_cloud);
        cvmat_2_rosmsg(depthMap, bgrImage, resp.point_cloud);
        pcl_ros::transformPointCloud("base_link", resp.point_cloud, resp.point_cloud, *tf_listener);
	resp.point_cloud.header.frame_id = "base_link";
        return true;
    }
    else
    {
        if(msgFromBag == NULL) return false;
        resp.point_cloud = *msgFromBag;
        tf_listener->waitForTransform("base_link", "kinect_link", msgFromBag->header.stamp, ros::Duration(0.5));
        pcl_ros::transformPointCloud("base_link", resp.point_cloud, resp.point_cloud, *tf_listener);
        return true;
    }
}

bool robotRgbdDownsampled_callback(point_cloud_manager::GetRgbd::Request &req, point_cloud_manager::GetRgbd::Response &resp)
{
    sensor_msgs::PointCloud2 msgCloudKinect;
    sensor_msgs::PointCloud2 msgCloudRobot; 
    initialize_rosmsg(msgCloudKinect, 640, 480, "kinect_link");
    int widthDownSample = (int) (640 / downsample_by);
    int heightDownSample = (int) (480 / downsample_by);
    initialize_rosmsg(resp.point_cloud, widthDownSample, heightDownSample, "base_link");
    if(!use_bag)
        cvmat_2_rosmsg(depthMap, bgrImage, msgCloudKinect);
    else
    {
        if(msgFromBag == NULL) return false;
        msgCloudKinect = *msgFromBag;
    }
    tf_listener->waitForTransform("base_link", "kinect_link", msgCloudKinect.header.stamp, ros::Duration(0.5));
    pcl_ros::transformPointCloud("base_link", msgCloudKinect, msgCloudRobot, *tf_listener);
    downsample_pcl(msgCloudRobot, resp.point_cloud, downsample_by);
    return true;
}

bool kinectRgbdDownsampled_callback(point_cloud_manager::GetRgbd::Request &req, point_cloud_manager::GetRgbd::Response &resp)
{
    sensor_msgs::PointCloud2 msgCloudKinect;
    initialize_rosmsg(msgCloudKinect, 640, 480, "kinect_link");
    int widthDownSample = (int) (640 / downsample_by);
    int heightDownSample = (int) (480 / downsample_by);
    initialize_rosmsg(resp.point_cloud, widthDownSample, heightDownSample, "base_link");
    if(!use_bag)
        cvmat_2_rosmsg(depthMap, bgrImage, msgCloudKinect);
    else
    {
        if(msgFromBag == NULL) return false;
        msgCloudKinect = *msgFromBag;
    }
    downsample_pcl(msgCloudKinect, resp.point_cloud, downsample_by);
    return true;
}

int main(int argc, char** argv)
{
    std::string file_name = "";
    use_oni = false;
    use_bag = false;
    for(int i=0; i < argc; i++)
    {
        std::string strParam(argv[i]);
        if(strParam.compare("--oni") == 0)
        {
            use_oni = true;
            file_name = argv[++i];
        }
        if(strParam.compare("--bag") == 0)
        {
            use_bag = true;
            file_name = argv[++i];
        }
        if(strParam.compare("--downsample_by") == 0)
        {
            downsample_by = atoi(argv[++i]); 
        }
    }
    
    std::cout << "INITIALIZING KINECT MANAGER BY MARCOSOF ..." << std::endl;
    if(use_oni) std::cout << "KinectMan.->Using ONI file: " << file_name << std::endl;
    else if(use_bag) std::cout << "KinectMan.->Using BAG file: " << file_name << std::endl;
    else std::cout << "KinectMan.->Using real kinect..." << std::endl;
    
    ros::init(argc, argv, "kinect_man");
    ros::NodeHandle n;
    ros::Publisher pubKinectFrame =n.advertise<sensor_msgs::PointCloud2>("/hardware/point_cloud_man/rgbd_wrt_kinect",1);
    ros::Publisher pubRobotFrame  =n.advertise<sensor_msgs::PointCloud2>("/hardware/point_cloud_man/rgbd_wrt_robot", 1);
    ros::Publisher pubDownsampled =n.advertise<sensor_msgs::PointCloud2>("/hardware/point_cloud_man/rgbd_wrt_robot_downsampled",1);
    pubImageIROS = n.advertise<sensor_msgs::Image>("/erlc/rgb_1/image", 1);
    pubDepthIROS = n.advertise<sensor_msgs::PointCloud2>("/erlc/depth_0/pointcloud", 1);
    ros::ServiceServer srvRgbdKinect = n.advertiseService("/hardware/point_cloud_man/get_rgbd_wrt_kinect", kinectRgbd_callback);
    ros::ServiceServer srvRgbdRobot  = n.advertiseService("/hardware/point_cloud_man/get_rgbd_wrt_robot", robotRgbd_callback);
    ros::ServiceServer srvRgbdKinectDownsampled  = n.advertiseService("/hardware/point_cloud_man/get_rgbd_wrt_kinect_downsampled", kinectRgbdDownsampled_callback);
    ros::ServiceServer srvRgbdRobotDownsampled  = n.advertiseService("/hardware/point_cloud_man/get_rgbd_wrt_robot_downsampled", robotRgbdDownsampled_callback);
    sensor_msgs::PointCloud2 msgCloudKinect;
    sensor_msgs::PointCloud2 msgCloudRobot; 
    sensor_msgs::PointCloud2 msgDownsampled;
    tf_listener = new tf::TransformListener();
    ros::Rate loop(30);
    tf_listener->waitForTransform("base_link", "kinect_link", ros::Time(0), ros::Duration(10.0));
    initialize_rosmsg(msgCloudKinect, 640, 480, "kinect_link");
    int widthDownSample = (int) (640 / downsample_by);
    int heightDownSample = (int) (480 / downsample_by);
    initialize_rosmsg(msgDownsampled, widthDownSample, heightDownSample, "base_link");

    if(!use_bag)
    {
        cv::VideoCapture capture;
        if(use_oni) std::cout << "KinectMan.->Trying to open oni file: " << file_name << std::endl;
        else std::cout << "KinectMan.->Triying to initialize kinect sensor... " << std::endl;
        if(use_oni) capture.open(file_name);
        else capture.open(CV_CAP_OPENNI);
        
        if(!capture.isOpened())
        {
            std::cout << "KinectMan.->Cannot open kinect :'(" << std::endl;
            return 1;
        }
        capture.set(CV_CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION, CV_CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION_ON);
        std::cout << "KinectMan.->Kinect sensor started :D" << std::endl;
        
        while(ros::ok())
        {
            if(!capture.grab())
            {
                loop.sleep();
                //ros::spinOnce();
                continue;
            }
            capture.retrieve(depthMap, CV_CAP_OPENNI_POINT_CLOUD_MAP);
            capture.retrieve(bgrImage, CV_CAP_OPENNI_BGR_IMAGE);
            if(pubKinectFrame.getNumSubscribers()>0 || pubRobotFrame.getNumSubscribers()>0 || pubDownsampled.getNumSubscribers()>0)
                cvmat_2_rosmsg(depthMap, bgrImage, msgCloudKinect);
            if(pubRobotFrame.getNumSubscribers()>0 || pubDownsampled.getNumSubscribers()>0)
                pcl_ros::transformPointCloud("base_link", msgCloudKinect, msgCloudRobot, *tf_listener);
            
            if(pubKinectFrame.getNumSubscribers() > 0)
                pubKinectFrame.publish(msgCloudKinect);
            if(pubRobotFrame.getNumSubscribers() > 0)
                pubRobotFrame.publish(msgCloudRobot);
            if(pubDownsampled.getNumSubscribers() > 0)
            {
                downsample_pcl(msgCloudRobot, msgDownsampled, downsample_by);
                pubDownsampled.publish(msgDownsampled);
            }
            
            ros::spinOnce();
            loop.sleep();
        }
    }
    else
    {
        rosbag::Bag bag;
        bag.open(file_name, rosbag::bagmode::Read);
        rosbag::View view(bag, rosbag::TopicQuery("/hardware/point_cloud_man/rgbd_wrt_kinect"));
        while(ros::ok())
        {
            foreach(rosbag::MessageInstance const m, view)
            {
                msgFromBag = m.instantiate<sensor_msgs::PointCloud2>();
                if(msgFromBag == NULL)
                {
                    loop.sleep();
                    continue;
                }
                msgFromBag->header.stamp = ros::Time::now();
                if(pubRobotFrame.getNumSubscribers()>0 || pubDownsampled.getNumSubscribers()>0)
                {
                    tf_listener->waitForTransform("base_link", "kinect_link", msgFromBag->header.stamp, ros::Duration(0.5));
                    pcl_ros::transformPointCloud("base_link", *msgFromBag, msgCloudRobot, *tf_listener);
                }
                
                if(pubKinectFrame.getNumSubscribers() > 0)
                    pubKinectFrame.publish(*msgFromBag);
                if(pubRobotFrame.getNumSubscribers() > 0)
                    pubRobotFrame.publish(msgCloudRobot);
                if(pubDownsampled.getNumSubscribers() > 0)
                {
                    downsample_pcl(msgCloudRobot, msgDownsampled, downsample_by);
                    pubDownsampled.publish(msgDownsampled);
                }
                ros::spinOnce();
                loop.sleep();
            }
        }
        bag.close();
    }
    delete tf_listener;
    return 0;
}
