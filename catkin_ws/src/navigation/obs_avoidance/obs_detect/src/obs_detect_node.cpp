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
#include "geometry_msgs/PointStamped.h"
#include "tf/transform_listener.h"
#include "justina_tools/JustinaTools.h"

sensor_msgs::LaserScan laserScan;
nav_msgs::Path lastPath;
cv::Mat bgrImg;
cv::Mat xyzCloud;
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
    JustinaTools::PointCloud2Msg_ToCvMat(msg, bgrImg, xyzCloud);
    //std::cout << "ObsDetector.->Received: width: " << bgrImg.cols << " height: " << bgrImg.rows << std::endl;
    //cv::imshow("OBSTACLE DETECTOR BY MARCOSOFT", bgrImg);
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
    
    if(dist < 0.15) dist = 0.15;
    if(dist > 0.6) dist = 0.6;
    //The idea is to search in an arc of 0.7
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
    //if(counter >= minCounter)
      //std::cout << "ObsDetect.->Collision risk detected with láser: min-max-counting: " << minSearchAngle << "  "
      //          << maxSearchAngle << "  " << counter << std::endl;
    return counter >= minCounter;
}

bool collisionRiskWithKinect(int pointAheadIdx, float robotX, float robotY, float robotTheta, float& collisionX, float& collisionY)
{
    if(bgrImg.cols < 1 || bgrImg.rows < 1)
        return false;

    //std::cout << "ObsDetect.->Starting detection" << std::endl;
    
    //std::cout << "ObsDetect.->Point ahead calculated" << std::endl;
    //Searchs for possibles collisions only when robot is already pointing to several points ahead in the current path
    //i.e. when the error angle is around zero
    //std::cout << "ObsDetect.->Point cloud size: " << xyzCloud.cols <<"x" << xyzCloud.rows<< std::endl;
    //Since coordinates are wrt robot, it searches only in a rectangle in front of the robot
    float minX = 0.25;
    float maxX = 0.8;
    float minY = -0.25;
    float maxY = 0.25;
    int counter = 0;
    float meanX = 0;
    float meanY = 0;
    for(int i=0; i< xyzCloud.cols; i++)
        for(int j=0; j< xyzCloud.rows; j++)
        {
            cv::Vec3f p = xyzCloud.at<cv::Vec3f>(j,i);
            if(p[2] < 0.05)
            {
                bgrImg.data[3*(j*bgrImg.cols + i)] = 0;
                bgrImg.data[3*(j*bgrImg.cols + i) + 1] = 0;
                bgrImg.data[3*(j*bgrImg.cols + i) + 2] = 0;
            }
            if(p[0] >= minX && p[0] <= maxX && p[1] >= minY && p[1] <= maxY && p[2] >= 0.05)
	      {
                counter++;
		meanX += p[0];
		meanY += p[1];
	      }
        }
    //std::cout << "ObsDetect.->Color modified" << std::endl;
    cv::imshow("OBSTACLE DETECTOR BY MARCOSOFT", bgrImg);

    float aheadX = lastPath.poses[pointAheadIdx].pose.position.x;
    float aheadY = lastPath.poses[pointAheadIdx].pose.position.y;
    float errorX = aheadX - robotX;
    float errorY = aheadY - robotY;
    float errorAngle = atan2(errorY, errorX) - robotTheta;
    if(errorAngle > M_PI) errorAngle -= 2*M_PI;
    if(errorAngle <= -M_PI) errorAngle += 2*M_PI;

    //if(fabs(errorAngle) > 0.17)
    //    return false;

    //if(counter > 50)
    //    std::cout << "ObsDetect.->Collision risk detected with kinect: angle-counting: " << errorAngle << "  " << counter << std::endl;
    collisionX = counter > 100 ? meanX / counter : 0;
    collisionY = counter > 100 ? meanY / counter : 0;
    return counter > 100;
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
    ros::Publisher pubCollisionPoint = n.advertise<geometry_msgs::PointStamped>("/navigation/obs_avoid/collision_point", 1);
    tf::TransformListener tf_listener;
    ros::Rate loop(30);

    std_msgs::Bool msgObsInFront;
    std_msgs::Bool msgCollisionRisk;
    geometry_msgs::PointStamped msgCollisionPoint;
    msgCollisionPoint.header.frame_id = "base_link";
    
    tf::StampedTransform tf;
    tf::Quaternion q;
    
    float robotX = 0;
    float robotY = 0;
    float robotTheta = 0;
    float collisionX;
    float collisionY;
    
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

        if(enable)
        {
            msgCollisionRisk.data = collisionRiskWithLaser(aheadIdx, robotX, robotY, robotTheta) ||
	      collisionRiskWithKinect(aheadIdx, robotX, robotY, robotTheta, collisionX, collisionY);
            //msgCollisionRisk.data = collisionRiskWithKinect(aheadIdx, robotX, robotY, robotTheta);
	    msgCollisionPoint.point.x = collisionX;
	    msgCollisionPoint.point.y = collisionY;
        }
        else
            msgCollisionRisk.data = false;
        pubCollisionRisk.publish(msgCollisionRisk);
	pubCollisionPoint.publish(msgCollisionPoint);

        //Check if there is an obstacle in front
        msgObsInFront.data = isThereAnObstacleInFront();
        pubObstacleInFront.publish(msgObsInFront);

        ros::spinOnce();
        loop.sleep();
    }
}
