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
bool enable = false;

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
    enable = msg->data;
}

bool isThereAnObstacleInFront()
{
    int obsInFrontCounter = 0;
    for(int i=0; i < laserScan.ranges.size(); i++)
    {
        float currentAngle = laserScan.angle_min + i*laserScan.angle_increment;
        if(laserScan.ranges[i] < 0.45 && currentAngle > -0.6 && currentAngle < 0.6)
            obsInFrontCounter++;
    }
    return obsInFrontCounter > 60;
}

int getLookAheadPathIdx(float robotX, float robotY)
{
    //Returns the index of the point in the path, to which the robot should be pointing to
    float distToNextPose = 0;
    if(currentPathIdx >= lastPath.poses.size())
        currentPathIdx = lastPath.poses.size() - 1;
    do
    {
        float lookAheadX = lastPath.poses[currentPathIdx].pose.position.x;
        float lookAheadY = lastPath.poses[currentPathIdx].pose.position.y;
        distToNextPose = sqrt((lookAheadX - robotX)*(lookAheadX - robotX) + (lookAheadY - robotY)*(lookAheadY - robotY));
    }while(distToNextPose < 1.0 && ++currentPathIdx < lastPath.poses.size());
    if(currentPathIdx >= lastPath.poses.size())
        currentPathIdx = lastPath.poses.size() - 1;
    return currentPathIdx;
}

bool collisionRiskWithLaser(int pointAheadIdx, float robotX, float robotY, float robotTheta)
{
    float aheadX = lastPath.poses[pointAheadIdx].pose.position.x;
    float aheadY = lastPath.poses[pointAheadIdx].pose.position.y;
    float errorX = aheadX - robotX;
    float errorY = aheadY - robotY;
    float errorAngle = atan2(errorY, errorX) - robotTheta;
    float dist = sqrt(errorX*errorX + errorY*errorY);
    if(errorAngle > M_PI) errorAngle -= 2*M_PI;
    if(errorAngle <= -M_PI) errorAngle += 2*M_PI;

    //The idea is to search in an arc of 0.7
    if(dist < 0.15) dist = 0.15;
    if(dist > 0.85) dist = 0.85;
    
    float searchAngle = 0.7 / dist;
    float minSearchAngle = errorAngle - searchAngle / 2;
    float maxSearchAngle = errorAngle + searchAngle / 2;
    if(minSearchAngle > M_PI) minSearchAngle -= 2*M_PI;
    if(minSearchAngle <= -M_PI) minSearchAngle += 2*M_PI;
    if(maxSearchAngle > M_PI) maxSearchAngle -= 2*M_PI;
    if(maxSearchAngle <= -M_PI) maxSearchAngle += 2*M_PI;

    int minCounter = (int)(searchAngle / laserScan.angle_increment * 0.2);
    int counter = 0;
    for(int i=0; i < laserScan.ranges.size(); i++)
    {
        float angle = laserScan.angle_min + i*laserScan.angle_increment;
        if(angle > minSearchAngle && angle < maxSearchAngle && laserScan.ranges[i] < dist)
            counter++;
    }
    //std::cout << "ObsDetect.->: " << minSearchAngle << "  " << maxSearchAngle << "  " << dist << "  " << minCounter << std::endl;
    return counter >= minCounter;
}

bool collisionRiskWithKinect(int pointAheadIdx, float robotX, float robotY, float robotTheta)
{
    return false;
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
    
    float robotX = 0;
    float robotY = 0;
    float robotTheta = 0;
    
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
        int aheadIdx = getLookAheadPathIdx(robotX, robotY);
        //std::cout << "ObstacleDetector.->Next path index: " << getLookAheadPathIdx(robotX, robotY) << std::endl;

        if(enable){
            msgCollisionRisk.data = collisionRiskWithLaser(aheadIdx, robotX, robotY, robotTheta) ||
                                    collisionRiskWithKinect(aheadIdx, robotX, robotY, robotTheta);
            pubCollisionRisk.publish(msgCollisionRisk);
        }

        //Check if there is an obstacle in front
        msgObsInFront.data = isThereAnObstacleInFront();
        pubObstacleInFront.publish(msgObsInFront);

        ros::spinOnce();
        loop.sleep();
    }
}
