#include <iostream>
#include <cmath>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"
#include "justina_tools/JustinaTools.h"

sensor_msgs::LaserScan laserScan;
nav_msgs::Path lastPath;
int currentPathIdx = 0;

ros::NodeHandle* nh;
ros::Subscriber subPointCloud;

void callbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laserScan = *msg;
}

void callbackPath(const nav_msgs::Path::ConstPtr& msg)
{
    lastPath = *msg;
    currentPathIdx = 20;
}

void callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    cv::Mat bgrImg;
    cv::Mat xyzCloud;
    JustinaTools::PointCloud2Msg_ToCvMat(msg, bgrImg, xyzCloud);
    //std::cout << "ObsDetector.->Received: width: " << bgrImg.cols << " height: " << bgrImg.rows << std::endl;
    cv::imshow("OBSTACLE DETECTOR BY MARCOSOFT", bgrImg);
}

void callbackEnable(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data)
    {
        std::cout << "ObsDetector.->Starting obstacle detection using point cloud..." << std::endl;
        subPointCloud = nh->subscribe("/hardware/point_cloud_man/rgbd_wrt_robot_downsampled", 1, callbackPointCloud);
        cv::namedWindow("OBSTACLE DETECTOR BY MARCOSOFT", cv::WINDOW_AUTOSIZE);
    }
    else
    {
        std::cout << "ObsDetector.->Stopping obstacle detection using point cloud..." << std::endl;
        subPointCloud.shutdown();
        cv::destroyWindow("OBSTACLE DETECTOR BY MARCOSOFT");
    }
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING OBSTACLE DETECTOR (ONLY LASER) NODE BY MARCOSOFT... " << std::endl;
    ros::init(argc, argv, "obs_detect");
    ros::NodeHandle n;
    nh = &n;
    ros::Subscriber subLaserScan = n.subscribe("/hardware/scan", 1, callbackLaserScan);
    ros::Subscriber subPath = n.subscribe("/navigation/mvn_pln/last_calc_path", 1, callbackPath);
    ros::Subscriber subEnable = n.subscribe("/navigation/obs_avoid/enable", 1, callbackEnable);
    ros::Publisher pubObstacleInFront = n.advertise<std_msgs::Bool>("/navigation/obs_avoid/obs_in_front", 1);
    ros::Publisher pubCollisionRisk = n.advertise<std_msgs::Bool>("/navigation/obs_avoid/collision_risk", 1);
    tf::TransformListener tf_listener;
    ros::Rate loop(30);

    std_msgs::Bool msgObsInFront;
    std_msgs::Bool msgCollisionRisk;
    
    tf::StampedTransform tf;
    tf::Quaternion q;

    bool obsInFront = false;
    bool collisionRisk = false;
    float minSearchAngle = 0;  //This angles will be determined according to the point which
    float maxSearchAngle = 0;  //is 1.0 [m] (20 path-steps) ahead the robot. They are used for determining
    float searchDistance = 0.7;//collision risk. ObsInFront is calculated always with the same angles
    int searchMinCounting = 0;
    int searchCounter = 0;
    float robotX = 0;
    float robotY = 0;
    float robotTheta = 0;
    int obsInFrontCounter = 0;
    
    laserScan.angle_increment = 3.14/512.0; //Just to have something before the first callback
    lastPath.poses.push_back(geometry_msgs::PoseStamped()); //Just to have something before the first callback
    tf_listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(5.0));

    while(ros::ok() && cv::waitKey(15) != 27)
    {
        //Getting robot position
        tf_listener.lookupTransform("map", "base_link", ros::Time(0), tf);
        robotX = tf.getOrigin().x();
        robotY = tf.getOrigin().y();
        q = tf.getRotation();
        robotTheta = atan2((float)q.z(), (float)q.w()) * 2;
        //Calculating position 20 path-steps ahead the robot
        if(currentPathIdx > (lastPath.poses.size()-1))
            currentPathIdx = lastPath.poses.size() - 1;
        float lookAheadX = lastPath.poses[currentPathIdx].pose.position.x;
        float lookAheadY = lastPath.poses[currentPathIdx].pose.position.y;
        float distToNextPose = sqrt((lookAheadX - robotX)*(lookAheadX - robotX) + (lookAheadY - robotY)*(lookAheadY - robotY));
        float lookAheadAngle = atan2(lookAheadY - robotY, lookAheadX - robotX) - robotTheta;
        if(lookAheadAngle > M_PI) lookAheadAngle -= 2*M_PI;
        if(lookAheadAngle <= -M_PI) lookAheadAngle += 2*M_PI;
        searchDistance = 0.75;//distToNextPose;
        if(searchDistance > 0.7) searchDistance = 0.7;
        if(searchDistance < 0.15) searchDistance = 0.15;
        minSearchAngle = -0.7;//lookAheadAngle - 0.65/searchDistance/2; //Search angle is calculated such that, at the given search distance,
        maxSearchAngle = 0.7;//lookAheadAngle + 0.65/searchDistance/2; //an arc of 0.65m (a litle bit more than the robot width) is covered
        if(minSearchAngle > M_PI) minSearchAngle -= 2*M_PI;
        if(minSearchAngle <= -M_PI) minSearchAngle += 2*M_PI;
        if(maxSearchAngle > M_PI) maxSearchAngle -= 2*M_PI;
        if(maxSearchAngle <= -M_PI) maxSearchAngle += 2*M_PI;
        searchMinCounting = 30;//(int)(0.6/searchDistance/laserScan.angle_increment*0.15); //I think (but I'm not sure) this will detect a 0.15m sized object
        //Checking for obstacles in front and collisions
        obsInFrontCounter = 0;
        searchCounter = 0;
        for(int i=0; i < laserScan.ranges.size(); i++)
        {
            float currentAngle = laserScan.angle_min + i*laserScan.angle_increment;
            if(laserScan.ranges[i] < 0.55 && currentAngle > -0.6 && currentAngle < 0.6)
                obsInFrontCounter++;
            if(laserScan.ranges[i] < searchDistance && currentAngle > minSearchAngle && currentAngle < maxSearchAngle)
                searchCounter++;
        }
        obsInFront = obsInFrontCounter > 60;
        collisionRisk = searchCounter > searchMinCounting;
        //if(obsInFront)
        //  std::cout << "ObsDetector.->Obstacle in front!!!" << std::endl;
        //if(collisionRisk)
        //  std::cout << "ObsDetector.->collision Risk!!!!" << std::endl;
        //Moving currentPathIdx to always point 1m ahead
        while(distToNextPose < 1.0 && ++currentPathIdx < lastPath.poses.size())
        {
            lookAheadX = lastPath.poses[currentPathIdx].pose.position.x;
            lookAheadY = lastPath.poses[currentPathIdx].pose.position.y;
            distToNextPose = sqrt((lookAheadX - robotX)*(lookAheadX - robotX) + (lookAheadY - robotY)*(lookAheadY - robotY));
        }
        //Publishing if there is an obstacle 20 path-steps ahead the robot (possible collision)
        msgCollisionRisk.data = collisionRisk;
        pubCollisionRisk.publish(msgCollisionRisk);
        //Publishing if there is an obstacle in front of the robot
        msgObsInFront.data = obsInFront;
        pubObstacleInFront.publish(msgObsInFront);
        ros::spinOnce();
        loop.sleep();
    }
}
