#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float32MultiArray.h"
#include "vision_msgs/FindLines.h"
#include "justina_tools/JustinaTools.h"
#include "linefinder.h"

float headTilt = 0;

bool callbackFindLines(vision_msgs::FindLines::Request &req, vision_msgs::FindLines::Response &resp)
{
    std::cout << "LineFinder.->Trying to find lines with headTilt = " << headTilt << std::endl;
    cv::Mat bgrImg;
    cv::Mat xyzCloud;
    JustinaTools::PointCloud2Msg_ToCvMat(req.point_cloud, bgrImg, xyzCloud);

    pointOfViewParameters povParams;

    povParams.color = true; //Color or BW debug image
    povParams.fullData = true; //If false, then use a limited height range

    //Define 3D search boinding box in front of the camera
    povParams.areaLimits.maxX = 1; //Width
    povParams.areaLimits.maxZ = 2; // Depth
    povParams.areaLimits.minY = 1.0; //Min height
    povParams.areaLimits.maxY = 1.5; //Max height

    povParams.angle = 0;//headTilt * 180 / 3.14159265358979323846; //Camera pitch angle in degrees
    std::cout << "LineFinder.->Changing perspective" << std::endl;
    changeViewPerspective ( bgrImg, xyzCloud, povParams);
    std::cout << "LineFinder.->Perspective changed" << std::endl;
    bool dist3d = true; //Calculate the line equation in pixel [FALSE ] or metric [TRUE] units
    std::cout << "LineFinder.->Extracting front line" << std::endl;
    cv::Mat line = frontLine(povParams, dist3d);
    std::cout << "LineFinder.->Front line extracted.." << std::endl;
    std::cout << "LineFinder.->Getting points from extracted line."<< std::endl;
    cv::Mat xyzLine, point;
    xyzLine.push_back(getCloudPoint(line.row(0)));
    xyzLine.push_back(getCloudPoint(line.row(1)));

    std::cout << "LineFinder.->X1: " << xyzLine.at<double>(0,0) << ", Y1: " << xyzLine.at<double>(0,1) << ", Z1: " << xyzLine.at<double>(0,2) << std::endl;
    std::cout << "LineFinder.->X2: " << xyzLine.at<double>(1,0) << ", Y2: " << xyzLine.at<double>(1,1) << ", Z2: " << xyzLine.at<double>(1,2) << std::endl;

    cv::imshow("LineFinder - SRC", povParams.src);
    cv::imshow("LineFinder - POV", povParams.front);

    geometry_msgs::Point p;
    p.x = xyzLine.at<double>(0,0);
    p.y = xyzLine.at<double>(0,1);
    p.z = xyzLine.at<double>(0,2);
    resp.lines.push_back(p);
    p.x = xyzLine.at<double>(1,0);
    p.y = xyzLine.at<double>(1,1);
    p.z = xyzLine.at<double>(1,2);
    resp.lines.push_back(p);
    
    return true;
}

void callbackHeadPos(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data.size() != 2)
        return;
    headTilt = msg->data[0];
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING LINE FINDER BY TENSHI AND CORRECTED BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "line_finder");
    ros::NodeHandle n;
    ros::ServiceServer srvFindLines = n.advertiseService("/vision/line_finder/find_lines_ransac", callbackFindLines);
    ros::Subscriber subHeadPos = n.subscribe("/hardware/head/current_pose", 1, callbackHeadPos);
    ros::Rate loop(10);
    
    while(ros::ok() && cv::waitKey(15) != 27)
    {
        ros::spinOnce();
        loop.sleep();
    }
}
