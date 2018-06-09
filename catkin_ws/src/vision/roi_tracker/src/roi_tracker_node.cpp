#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTasks.h"

#include <boost/filesystem.hpp>

#include <justina_tools/JustinaTools.h>
#include <justina_tools/JustinaTasks.h>
#include <vision_msgs/TrackedObject.h>

#include "LMS.hpp"

#include "RoiTracker.hpp"

bool debugMode = true;
std::string rosNodeName = "roi_tracker_node";
std::string prompt = "  > RoiTracker Node. " ;

std::string configFileName = "configFile.xml";
std::string configDir  = "";
std::string configPath ;

ros::NodeHandle* node;

ros::ServiceClient cli_rgbdRobot;
ros::Subscriber sub_pointCloudRobot;
ros::Publisher pub_rvizMarkers;
ros::Publisher pub_roiPose;

ros::ServiceServer srv_initTrackInFront;
ros::ServiceServer srv_stopTrackInFront;
ros::Publisher pub_trackInFront;
bool enableTrackInFront = false;
bool enableTrain = false;

ros::ServiceServer srv_enableMoveHead;
bool enableMoveHead = false;

RoiTracker roiTracker;
cv::Rect trackedRoi;
cv::Point3f trackedCentroid;
double trackedConfidence;
cv::Point3f centroidLast;

/********************/
    //OBJETOS DEL LMS FILTER
    LMS PosxI(10,0.01,0.01);
    LMS PosyI(10,0.01,0.01);


bool GetImagesFromJustina(cv::Mat& imaBGR, cv::Mat& imaPCL)
{
    point_cloud_manager::GetRgbd srv;
    if(!cli_rgbdRobot.call(srv))
        return false;

    JustinaTools::PointCloud2Msg_ToCvMat(srv.response.point_cloud, imaBGR, imaPCL);
    return true;
}

void cb_sub_pointCloudRobot(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    cv::Mat imaBGR;
    cv::Mat imaXYZ;
    JustinaTools::PointCloud2Msg_ToCvMat(msg, imaBGR, imaXYZ);

    if( enableTrackInFront )
    {
        cv::Mat imaCopy;
        if( debugMode )
            imaCopy = imaBGR.clone();

            vision_msgs::TrackedObject trackedObj;

        cv::Rect roi;
        double confidence;
        if( roiTracker.Update( imaBGR, imaXYZ, roi, confidence ) )
        {
            trackedObj.isFound = true;
            if( debugMode )
                cv::rectangle( imaCopy, roi, cv::Scalar(0,255,0), 2);
        }
        else
        {
        	if( roiTracker.UpdateROI( imaBGR, imaXYZ, roi, confidence ) )
        	{
            	trackedObj.isFound = true;
            	if( debugMode )
                cv::rectangle( imaCopy, roi, cv::Scalar(0,0,0), 2);
	        }
	        else
	        {
	            trackedObj.isFound = false;
	            if( debugMode )
	                cv::rectangle( imaCopy, roi, cv::Scalar(0,0,255), 2);
	        }
        }

        cv::Point centroidPixels = (roi.tl() + roi.br())/2;
        /**LMS Imagen**/
        PosxI.UpdateW(centroidPixels.x);
        PosyI.UpdateW(centroidPixels.y);
        centroidPixels.x=PosxI.Stimate();
        centroidPixels.y=PosyI.Stimate();

        if(debugMode)
        	std::cout<<"confidence="<<confidence<<endl;


        cv::Point3f centroid = imaXYZ.at< cv::Vec3f >( centroidPixels );

        if(trackedObj.isFound == true && abs(roiTracker.centroidLast.x-centroid.x)>0.2 && abs(roiTracker.centroidLast.x-centroid.x)<1.7)
        {
            trackedObj.position.x = centroid.x;
            trackedObj.position.y = centroid.y;
            trackedObj.position.z = centroid.z;
            centroidLast=centroid;
        }
        else
        {
            trackedObj.position.x = roiTracker.centroidLast.x;
            trackedObj.position.y = roiTracker.centroidLast.y;
            trackedObj.position.z = roiTracker.centroidLast.z;
        }

        trackedObj.confidence = confidence;

        pub_trackInFront.publish( trackedObj );

        visualization_msgs::Marker marker_roi;

        marker_roi.header.frame_id = "base_link";
        marker_roi.header.stamp = ros::Time();
        marker_roi.ns = "roi_pose";
        marker_roi.id = 0;
        marker_roi.type = visualization_msgs::Marker::SPHERE;
        marker_roi.action = visualization_msgs::Marker::ADD;
        marker_roi.pose.position.x = trackedObj.position.x;
        marker_roi.pose.position.y = trackedObj.position.y;
        marker_roi.pose.position.z = trackedObj.position.z;
        marker_roi.pose.orientation.x=0.0;
        marker_roi.pose.orientation.y=0.0;
        marker_roi.pose.orientation.z=0.0;
        marker_roi.pose.orientation.w=1.0;
        marker_roi.scale.x = 0.1;
        marker_roi.scale.y = 0.1;
        marker_roi.scale.z = 0.1;
        marker_roi.color.a = 1.0;
        marker_roi.color.r = 0.0;
        marker_roi.color.g = 1.0;
        marker_roi.color.b = 0.0;
        marker_roi.lifetime = ros::Duration(1.0);

        pub_roiPose.publish(marker_roi);

        cv::imshow( "trackInFront", imaCopy );
        return;
    }
    if( enableTrain )
    {
        cout<<"Train"<<endl;
        cv::Mat imaCopy;
        if( debugMode )
            imaCopy = imaBGR.clone();

        vision_msgs::TrackedObject trackedObj;

        cv::Rect roi;
        double confidence;
        if( roiTracker.Train( imaBGR, imaXYZ,enableTrain,enableTrackInFront) )
        {
            trackedObj.isFound = true;
            if( debugMode )
                cv::rectangle( imaCopy, roi, cv::Scalar(0,255,0), 2);
        }
        else
        {
            trackedObj.isFound = false;
            if( debugMode )
                cv::rectangle( imaCopy, roi, cv::Scalar(0,0,255), 2);
        }

    }
}

bool cb_srv_initTrackInFront(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
{
    cv::Mat imaBGR, imaXYZ;

    if( !GetImagesFromJustina( imaBGR, imaXYZ ) )
    {
        resp.success = false;
        resp.message = "Can't get images from GetImagesFromJustina";

        std::cout << prompt <<"ERROR! :" << resp.message << std::endl;
    }
    else
    {
        roiTracker.LoadParams( configPath );
        if( roiTracker.InitFront(imaBGR, imaXYZ) )
        {
        	if(roiTracker.IfPerson(imaBGR)){
		          resp.success = true;
              resp.message = "success";

	            enableTrackInFront = false;
	            enableTrain = true;
	            sub_pointCloudRobot = node -> subscribe("/hardware/point_cloud_man/rgbd_wrt_robot", 1, cb_sub_pointCloudRobot);
        	}else{
        		resp.success = false;
	            resp.message = "Can't init tracker. Anyone.";

            	std::cout << prompt <<"ERROR! :" << resp.message << std::endl;
        	}

        }
        else
        {
            resp.success = false;
            resp.message = "Cant init tracker. Nothing in front.";

            std::cout << prompt <<"ERROR! :" << resp.message << std::endl;
        }
    }

    return resp.success;
}

bool cb_srv_stopTrackInFront(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
    std::cout << " >>>>> STOP ROI TRACKER NODE <<<<<" << std::endl;
    sub_pointCloudRobot.shutdown();
    enableTrackInFront = false;
    try{
        cv::destroyAllWindows();
    }
    catch(...){}
    return true;
}

bool cb_srv_enableMoveHead(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
    enableMoveHead = req.data;
    resp.success = true;
    return true;
}

// MAIN
int main(int argc, char** argv)
{
	std::cout << " >>>>> INIT ROI TRACKER NODE <<<<<" << std::endl;

	ros::init(argc, argv, "roi_tracker_node");
	ros::NodeHandle n;
    node = &n;
	ros::Rate loop(30);

    configDir = ros::package::getPath("roi_tracker") + "/ConfigDir";
    if( !boost::filesystem::exists( configDir ) )
        boost::filesystem::create_directory( configDir );
    configPath = configDir + "/" +  configFileName;


    cli_rgbdRobot           = n.serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot");
    pub_roiPose             = n.advertise<visualization_msgs::Marker> ("/hri/visualization_marker",0);

    srv_initTrackInFront    = n.advertiseService("/vision/roi_tracker/init_track_inFront", cb_srv_initTrackInFront) ;
    srv_stopTrackInFront    = n.advertiseService("/vision/roi_tracker/stop_track_inFront", cb_srv_stopTrackInFront) ;
    pub_trackInFront        = n.advertise< vision_msgs::TrackedObject >("/vision/roi_tracker/tracking_inFront", 1);

    srv_enableMoveHead      = n.advertiseService("/vision/roi_tracker/enable_move_head", cb_srv_enableMoveHead);

  	while(ros::ok && cv::waitKey(1) != 'q')
  	{
  		loop.sleep();
          ros::spinOnce();
    }
    cv::destroyAllWindows();
}
