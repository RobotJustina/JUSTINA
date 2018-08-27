#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/transforms.h"
#include "tf/transform_listener.h"
#include "tf_conversions/tf_eigen.h"
#include "point_cloud_manager/GetRgbd.h"


enum Processor { cl, gl, cpu };

bool protonect_shutdown = false; // Whether the running application should shut down.

cv::Mat rgbmat, depthmat;
tf::TransformListener * tf_listener; 
libfreenect2::Registration* registration;
libfreenect2::Frame undistorted(512, 424, 4);
libfreenect2::Frame registered(512, 424, 4);
libfreenect2::Frame depth2rgb(1920, 1082, 4);


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

void registration_to_cloud(sensor_msgs::PointCloud2 &msg, libfreenect2::Registration * registration, libfreenect2::Frame * ptr_undistorted, libfreenect2::Frame * ptr_registered){
    int indx = 0;
    for(int j = 0; j < ptr_registered->height; j++){
        for(int i = 0; i < ptr_registered->width; i++){
            float x,y,z,rgb; 
            registration->getPointXYZRGB(ptr_undistorted, ptr_registered, j, ptr_registered->width - i, x, y, z, rgb );
            const uint8_t *p = reinterpret_cast<uint8_t*>(&rgb);
            x *= -1;
            memcpy(&msg.data[indx*16], &x, 4);
            memcpy(&msg.data[indx*16 + 4], &y, 4);
            memcpy(&msg.data[indx*16 + 8], &z, 4);
            msg.data[indx*16 + 12] = p[0];
            msg.data[indx*16 + 13] = p[1];
            msg.data[indx*16 + 14] = p[2];
            msg.data[indx*16 + 15] = 255;
            indx++;
        }
    }
}


void downsample_by_3(sensor_msgs::PointCloud2& src, sensor_msgs::PointCloud2& dst)
{
    for(int i=0; i < dst.width; i++)
        for(int j=0; j < dst.height; j++)
            memcpy(&dst.data[16*(j*dst.width + i)], &src.data[48*(j*src.width + i)], 16);
}

void sigint_handler(int s)
{
    protonect_shutdown = true;
}


/// [main]
/**
 * Main application entry point.
 *
 * Accepted argumemnts:
 * - cpu Perform depth processing with the CPU.
 * - gl  Perform depth processing with OpenGL.
 * - cl  Perform depth processing with OpenCL.
 */

bool kinectRgbd_callback(point_cloud_manager::GetRgbd::Request &req, point_cloud_manager::GetRgbd::Response &resp)
{
    initialize_rosmsg(resp.point_cloud, 512, 424, "kinect_link");
    registration_to_cloud(resp.point_cloud, registration, &undistorted, &registered); 
    //cvmat_2_rosmsg(depthmat, rgbmat, resp.point_cloud);
    resp.point_cloud.header.stamp = ros::Time::now();
    return true;
}

bool robotRgbd_callback(point_cloud_manager::GetRgbd::Request &req, point_cloud_manager::GetRgbd::Response &resp)
{

    initialize_rosmsg(resp.point_cloud, 512, 424, "kinect_link");
    registration_to_cloud(resp.point_cloud, registration, &undistorted, &registered); 
    //cvmat_2_rosmsg(depthmat, rgbmat, resp.point_cloud);
    pcl_ros::transformPointCloud("base_link", resp.point_cloud, resp.point_cloud, *tf_listener);
    resp.point_cloud.header.frame_id = "base_link";
    return true;
}



int main(int argc, char** argv)
{
    std::cout << "INITIALIZING KINECT2 MANAGER BY HUGO..." << std::endl;
    std::string program_path(argv[0]);
    std::cerr << "Kinect2Man.->Freenect2 Version: " << LIBFREENECT2_VERSION << std::endl;
    std::cerr << "Kinect2Man.->Usage: " << program_path << " [-gpu=<id>] [gl | cl | cuda | cpu] " << std::endl;

    ros::init(argc, argv, "kinect2_manager");
    ros::NodeHandle n;
    tf_listener = new tf::TransformListener();
    ros::Rate rate(10);
    
    tf_listener->waitForTransform("base_link", "kinect_link", ros::Time(0), ros::Duration(10.0));
    /* Logger levels
     * 
     None = 0,
     Error = 1,
     Warning = 2,
     Info = 3,
     Debug = 4,
     * */

    //! [Setup logger level]
    libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Error));
    //! [Setup logger level]


    //! [context]
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;
    //! [context]


    //! DEFAULT PIPELINE
    if (argc <= 1) 
        pipeline = new libfreenect2::OpenGLPacketPipeline(); //OPEN GL



    int deviceId = -1;

    //! [Arguments]
    for(int argI = 1; argI < argc; ++argI)
    {
        const std::string arg(argv[argI]);

        if(arg == "-help" || arg == "--help" || arg == "-h" || arg == "-v" || arg == "--version" || arg == "-version")
        {
            // Just let the initial lines display at the beginning of main
            return 0;
        }
        else if(arg.find("-gpu=") == 0)
        {
            if (pipeline)
            {
                std::cerr << "Kinect2Man.-> -gpu must be specified before pipeline argument" << std::endl;
                return -1;
            }
            deviceId = atoi(argv[argI] + 5);
        }
        else if(arg == "cpu")
        {
            if(!pipeline)
                /// [pipeline]
                pipeline = new libfreenect2::CpuPacketPipeline();
            /// [pipeline]
            std::cout << "Kinect2Man.-> Using CPU pipeline." << std::endl;
        }
        else if(arg == "gl")
        {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
            if(!pipeline)
                pipeline = new libfreenect2::OpenGLPacketPipeline();

            std::cout << "Kinect2Man.-> Using OPENGL pipeline." << std::endl;
#else
            std::cout << "Kinect2Man.-> OpenGL pipeline is not supported!" << std::endl;
#endif
        }
        else if(arg == "cl")
        {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
            if(!pipeline)
                pipeline = new libfreenect2::OpenCLPacketPipeline(deviceId);
            std::cout << "Kinect2Man.-> Using OPENCL pipeline." << std::endl;
#else
            std::cout << "Kinect2Man.-> OpenCL pipeline is not supported!" << std::endl;
#endif
        }
        else if(arg == "cuda")
        {
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
            if(!pipeline)
                pipeline = new libfreenect2::CudaPacketPipeline(deviceId);
            std::cout << "Kinect2Man.-> Using CUDA pipeline." << std::endl;
#else
            std::cout << "Kinect2Man.-> CUDA pipeline is not supported!" << std::endl;
#endif
        } 
        else
        {
            std::cout << "Kinect2Man.-> Unknown argument: " << arg << std::endl;
        }
    }
    //! [Arguments]


    //! [discovery]
    if(freenect2.enumerateDevices() == 0)
    {
        std::cout << "Kinect2Man.-> NO DEVIDE CONNECTED!" << std::endl;
        return -1;
    }

    std::string serial = freenect2.getDefaultDeviceSerialNumber();

    std::cout << "Kinect2Man.->SERIAL: " << serial << std::endl;
    //! [discovery]

    if(pipeline)
    {
        //! [open]
        dev = freenect2.openDevice(serial, pipeline);
        //! [open]
    } 
    else 
    {
        dev = freenect2.openDevice(serial);
        std::cout << "Kinect2Man.-> Using default pipeline." << std::endl;
    }

    if(dev == 0)
    {
        std::cout << "Kinect2Man.-> FAILURE OPPENING DEVICE!" << std::endl;
        return -1;
    }

    //! [max/min depth and filtering configuration]
    libfreenect2::Freenect2Device::Config config;
    config.EnableBilateralFilter = true;
    config.EnableEdgeAwareFilter = true;
    config.MinDepth = 0.2; 
    config.MaxDepth = 8.0; // 12 m
    dev->setConfiguration(config);  
    //! [max/min depth and filtering configuration]



    signal(SIGINT, sigint_handler);
    protonect_shutdown = false;

    //! [listeners]
    libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Depth | libfreenect2::Frame::Ir);
    libfreenect2::FrameMap frames;

    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);
    //! [listeners]

    //! [start]
    dev->start();

    std::cout << "Kinect2Man.->device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "Kinect2Man.->device firmware: " << dev->getFirmwareVersion() << std::endl;
    //! [start]

    //! [registration setup]
    libfreenect2::Freenect2Device::IrCameraParams irParams = dev->getIrCameraParams();
    registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    
    // According to Registration->apply documentation, the 6th parameter should be 1920x1082 
    //! [registration setup]

    cv::Mat depthmatUndistorted, irmat, rgbd, rgbd2;
    pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
    pointCloud.width = 512;
    pointCloud.height = 424;
    pointCloud.points.resize(pointCloud.width*pointCloud.height);

    ros::Publisher pubKinectFrame = n.advertise<sensor_msgs::PointCloud2>("/hardware/point_cloud_man/rgbd_wrt_kinect",1);
    ros::Publisher pubRobotFrame  = n.advertise<sensor_msgs::PointCloud2>("/hardware/point_cloud_man/rgbd_wrt_robot", 1);
    ros::Publisher pubDownsampled = n.advertise<sensor_msgs::PointCloud2>("/hardware/point_cloud_man/rgbd_wrt_robot_downsampled",1);
    ros::ServiceServer srvRgbdKinect = n.advertiseService("/hardware/point_cloud_man/get_rgbd_wrt_kinect", kinectRgbd_callback);;
    ros::ServiceServer srvRgbdRobot = n.advertiseService("/hardware/point_cloud_man/get_rgbd_wrt_robot", robotRgbd_callback);;
    sensor_msgs::PointCloud2 msgCloudKinect;
    sensor_msgs::PointCloud2 msgCloudRobot; 
    sensor_msgs::PointCloud2 msgDownsampled;
    

    initialize_rosmsg(msgCloudKinect, 512, 424, "kinect_link");
    initialize_rosmsg(msgDownsampled, 170, 141, "base_link");

    //! [loop start]
    while(!protonect_shutdown && ros::ok())
    {


        //libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        //libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        //libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
        //! [loop start]

        //! [Converts frames acquired to OpenCV Mats]
        //cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
        //cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);
        //cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);    
        //! [Converts frames acquired to OpenCV Mats]


        /** registration->apply(). Map color images onto depth images
         * @param rgb Color image (1920x1080 BGRX)
         * @param depth Depth image (512x424 float)
         * @param[out] undistorted Undistorted depth image
         * @param[out] registered Color image for the depth image (512x424)
         * @param (Optional) enable_filter Filter out pixels not visible to both cameras. Default value true.
         * @param[out] (Optional) bigdepth If not `NULL`, return mapping of depth onto colors (1920x1082 float). **1082** not 1080, with a blank top and bottom row.
         * @param[out] (Optional) color_depth_map Index of mapped color pixel for each depth pixel (512x424).
         */
        //! [registration]
        //registration->apply(rgb, depth, &undistorted, &registered, false);//, true, &depth2rgb);
        //! [registration]

        //! [Converts registered/undistorted frames to OpenCV Mats]
        //cv::Mat depthmatUndistorted(undistorted.height, undistorted.width, CV_32FC1, undistorted.data);
        //cv::Mat rgbd(registered.height, registered.width, CV_8UC4, registered.data);
        //cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);
        //! [Converts registered/undistorted frames to OpenCV Mats]
        //std::cout << "Depth: " << depthmatUndistorted.at<float>(256, 212) << "Color: " << int(rgbd.data[512*212*4 + 256*4]) << " " << int(rgbd.data[512*212*4 + 256*4 + 1]) << " " << int(rgbd.data[512*212*4 + 256*4 + 2]) << std::endl;
        // Shows OpenCV Mats
        //float max_depth = 5000.0f;

        //cv::imshow("rgb", rgbmat);
        //cv::imshow("ir", irmat / max_depth); 
        //cv::imshow("depth", depthmat / max_depth);
        //Copy the registered data to a point cloud

        if(pubKinectFrame.getNumSubscribers()>0 || pubRobotFrame.getNumSubscribers()>0 || pubDownsampled.getNumSubscribers()>0){
            if (!listener.waitForNewFrame(frames, 5*1000)) // 5 seconds
            {
                std::cout << "Kinect2Man.->Timeout! Exit!" << std::endl;
                return -1;
            }
            libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
            libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
            registration->apply(rgb, depth, &undistorted, &registered, false);//, true, &depth2rgb);
            cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
            cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat); 

            //cv::Mat depthmatUndistorted(undistorted.height, undistorted.width, CV_32FC1, undistorted.data);
            //cv::Mat rgbd(registered.height, registered.width, CV_8UC4, registered.data);
            registration_to_cloud(msgCloudKinect, registration, &undistorted, &registered); 
            listener.release(frames); // Free memory used
        }

        if(pubRobotFrame.getNumSubscribers()>0 || pubDownsampled.getNumSubscribers()>0)
            pcl_ros::transformPointCloud("base_link", msgCloudKinect, msgCloudRobot, *tf_listener);
        if(pubKinectFrame.getNumSubscribers() > 0)
            pubKinectFrame.publish(msgCloudKinect);
        if(pubRobotFrame.getNumSubscribers() > 0)
            pubRobotFrame.publish(msgCloudRobot);
        if(pubDownsampled.getNumSubscribers() > 0){
            downsample_by_3(msgCloudRobot, msgDownsampled);
            pubDownsampled.publish(msgDownsampled);
        }

        //ROS STUFF
        //pcl::toROSMsg(pointCloud, msgCloudKinect);
        /*if(pubRobotFrame.getNumSubscribers() > 0)
          {
          tf::StampedTransform transformTf;
          tf_listener.lookupTransform("base_link", "kinect_link", ros::Time(0), transformTf);
          Eigen::Affine3d transformEigen;
          tf::transformTFToEigen(transformTf, transformEigen);
          pcl::transformPointCloud(pointCloud, pointCloud, transformEigen);
          pcl::toROSMsg(pointCloud, msgCloudRobot);
          msgCloudRobot.header.frame_id = "base_link";
          }*/
        //

        //cv::imshow("undistorted", depthmatUndistorted / max_depth);
        //cv::imshow("registered", rgbd);
        //cv::imshow("depth2RGB", rgbd2 / max_depth);


        /*int key = cv::waitKey(1);
        protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape*/

        //! [loop end]
        ros::spinOnce();
        rate.sleep();
    }
    //! [loop end]

    //! [stop]
    dev->stop();
    dev->close();
    //! [stop]

    delete registration;

    return 0;
}
