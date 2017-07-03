#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <justina_tools/JustinaTools.h>

ros::NodeHandle* node;

ros::ServiceClient cli_rgbdRobot;
ros::Subscriber sub_enaTrackByXYZ; 
ros::Subscriber sub_enaTrackByRect;
ros::Subscriber sub_enaMoveHead;
ros::Subscriber sub_type; 
ros::Subscriber sub_pointCloudRobot;

std::string type = "";
bool enaTrackByRect = false;

cv::Rect2d roi;

bool GetImagesFromJustina(cv::Mat& imaBGR, cv::Mat& imaPCL)
{
    point_cloud_manager::GetRgbd srv;
    if(!cli_rgbdRobot.call(srv))
        return false;

    JustinaTools::PointCloud2Msg_ToCvMat(srv.response.point_cloud, imaBGR, imaPCL);
    return true; 
}

void cb_sub_enaMoveHead(const std_msgs::Bool::ConstPtr& msg)
{
    return;
}

void cb_sub_pointCloudRobot(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if( enaTrackByRect )
    {
        cv::Mat imaBGR, imaXYZ;
        if( !GetImagesFromJustina( imaBGR, imaXYZ ) )
            return;
    }
}

void cb_sub_enaTrackByRect(const std_msgs::Bool::ConstPtr& msg)
{
    if( enaTrackByRect = msg->data )
    {
        cv::Mat imaBGR, imaXYZ;
        if( !GetImagesFromJustina( imaBGR, imaXYZ ) )
            return;
        
        sub_pointCloudRobot = node -> subscribe("/hardware/point_cloud_man/rgbd_wrt_robot", 1, cb_sub_pointCloudRobot);         
    }
}


int main(int argc, char** argv)
{
	std::cout << " >>>>> INIT PANO MAKER NODE <<<<<" << std::endl; 
	
	ros::init(argc, argv, "roi_tracker_node"); 
	ros::NodeHandle n;
    node = &n;
	ros::Rate loop(60); 

    cli_rgbdRobot   = n.serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot");

    sub_enaTrackByRect  = n.subscribe("/vision/roi_tracker/enable_track_byRect" , 1, cb_sub_enaTrackByRect); 
    sub_enaMoveHead     = n.subscribe("/vision/roi_tracker/enable_move_head"    , 1, cb_sub_enaMoveHead); 

	while(ros::ok)
	{
		ros::spinOnce();
		loop.sleep();

        if( cv::waitKey(1) == 'q' )
            break;
    }
 
    sub_enaTrackByXYZ.shutdown(); 
    sub_enaTrackByRect.shutdown(); 
    sub_enaMoveHead.shutdown();
    sub_type.shutdown();

    cv::destroyAllWindows();

    return 0; 
}
