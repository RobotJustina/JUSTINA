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


enum Processor { cl, gl, cpu };

bool protonect_shutdown = false; // Whether the running application should shut down.

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

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING KINECT2 MANAGER BY JOSÉ LUIS..." << std::endl;
    std::string program_path(argv[0]);
	std::cerr << "Kinect2Man.->Freenect2 Version: " << LIBFREENECT2_VERSION << std::endl;
	std::cerr << "Kinect2Man.->Usage: " << program_path << " [-gpu=<id>] [gl | cl | cuda | cpu] " << std::endl;

    ros::init(argc, argv, "kinect2_manager");
    ros::NodeHandle n;
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
    } else {
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
    config.MinDepth = 0.1; 
    config.MaxDepth = 12.0; // 12 m
    dev->setConfiguration(config);	
	//! [max/min depth and filtering configuration]



    signal(SIGINT, sigint_handler);
    protonect_shutdown = false;

    //! [listeners]
    libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color |
                                                  libfreenect2::Frame::Depth |
                                                  libfreenect2::Frame::Ir);
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
    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4);
    libfreenect2::Frame registered(512, 424, 4);
    libfreenect2::Frame depth2rgb(1920, 1082, 4); 
    // According to Registration->apply documentation, the 6th parameter should be 1920x1082 
    //! [registration setup]

    cv::Mat rgbmat, depthmat, depthmatUndistorted, irmat, rgbd, rgbd2;
    pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
    pointCloud.width = 512;
    pointCloud.height = 424;
    pointCloud.points.resize(pointCloud.width*pointCloud.height);

    ros::Publisher pubKinectFrame = n.advertise<sensor_msgs::PointCloud2>("/hardware/point_cloud_man/rgbd_wrt_kinect",1);
    ros::Publisher pubRobotFrame  = n.advertise<sensor_msgs::PointCloud2>("/hardware/point_cloud_man/rgbd_wrt_robot", 1);
    ros::ServiceServer srvRgbdKinect;
    ros::ServiceServer srvRgbdRobot;
    sensor_msgs::PointCloud2 msgCloudKinect;
    sensor_msgs::PointCloud2 msgCloudRobot;
    tf::TransformListener tf_listener;

    //! [loop start]
    while(!protonect_shutdown && ros::ok())
    {
	
        if (!listener.waitForNewFrame(frames, 5*1000)) // 5 seconds
        {
            std::cout << "Kinect2Man.->Timeout! Exit!" << std::endl;
            return -1;
        }

        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        //libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
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
        registration->apply(rgb, depth, &undistorted, &registered, false);//, true, &depth2rgb);
        //! [registration]
        
        //! [Converts registered/undistorted frames to OpenCV Mats]
	cv::Mat depthmatUndistorted(undistorted.height, undistorted.width, CV_32FC1, undistorted.data);
        cv::Mat rgbd(registered.height, registered.width, CV_8UC4, registered.data);
        //cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);
        //! [Converts registered/undistorted frames to OpenCV Mats]
        //std::cout << "Depth: " << depthmatUndistorted.at<float>(256, 212) << "Color: " << int(rgbd.data[512*212*4 + 256*4]) << " " << int(rgbd.data[512*212*4 + 256*4 + 1]) << " " << int(rgbd.data[512*212*4 + 256*4 + 2]) << std::endl;
        // Shows OpenCV Mats
        //float max_depth = 5000.0f;
        
        //cv::imshow("rgb", rgbmat);
        //cv::imshow("ir", irmat / max_depth); 
        //cv::imshow("depth", depthmat / max_depth);
        //Copy the registered data to a point cloud
        for(size_t i=0; i < pointCloud.width; i++)
        { 
 	    for(size_t j=0; j< pointCloud.height; j++)
            {
               // float uz = depthmatUndistorted.at<float>(j, i) / 1000.0f ;
               // size_t idx = j*pointCloud.width + i;
	       // 
               // pointCloud.points[idx].x = -(i + 0.5 - irParams.cx) / irParams.fx * uz;
               // pointCloud.points[idx].y =  (j + 0.5 - irParams.cy) / irParams.fy * uz;
               // pointCloud.points[idx].z = uz;
               // pointCloud.points[idx].b = rgbd.data[idx*4];
               // pointCloud.points[idx].g = rgbd.data[idx*4 + 1];
               // pointCloud.points[idx].r = rgbd.data[idx*4 + 2];

	       float x,y,z,rgb; 
	       registration->getPointXYZRGB(&undistorted, &registered, j, i, x, y, z, rgb );  
               size_t idx = j*pointCloud.width + i;
               pointCloud.points[idx].x = x;
               pointCloud.points[idx].y = y;
               pointCloud.points[idx].z = z;
	       
               const uint8_t *p = reinterpret_cast<uint8_t*>(&rgb);
               pointCloud.points[idx].b = p[0];
               pointCloud.points[idx].g = p[1];
               pointCloud.points[idx].r = p[2];
            }
         }       
        
        //ROS STUFF
        pcl::toROSMsg(pointCloud, msgCloudKinect);
        msgCloudKinect.header.frame_id = "kinect_link";
        if(pubKinectFrame.getNumSubscribers() > 0)
        {
            pubKinectFrame.publish(msgCloudKinect);
        }
        if(pubRobotFrame.getNumSubscribers() > 0)
        {
            tf::StampedTransform transformTf;
            tf_listener.lookupTransform("base_link", "kinect_link", ros::Time(0), transformTf);
            Eigen::Affine3d transformEigen;
            tf::transformTFToEigen(transformTf, transformEigen);
            pcl::transformPointCloud(pointCloud, pointCloud, transformEigen);
            pcl::toROSMsg(pointCloud, msgCloudRobot);
            msgCloudRobot.header.frame_id = "base_link";
            pubRobotFrame.publish(msgCloudRobot);
        }
        //
		
        //cv::imshow("undistorted", depthmatUndistorted / max_depth);
        cv::imshow("registered", rgbd);
        //cv::imshow("depth2RGB", rgbd2 / max_depth);
              
        
        int key = cv::waitKey(1);
        protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape
        
        //! [loop end]
        listener.release(frames); // Free memory used
        ros::spinOnce();
    }
    //! [loop end]

    //! [stop]
    dev->stop();
    dev->close();
    //! [stop]

    delete registration;

    return 0;
}
