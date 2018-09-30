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
#include "geometry_msgs/Twist.h"
#include "tf/transform_listener.h"
#include "justina_tools/JustinaTools.h"

sensor_msgs::LaserScan laserScan;
nav_msgs::Path lastPath;
nav_msgs::Path lastPathTransform;
cv::Mat bgrImg;
cv::Mat xyzCloud;
int currentPathIdx = 0;
bool enable = false;
bool enableDoorDetector = false;
float current_speed_linear = 0;
float current_speed_angular = 0;

ros::NodeHandle* nh;
ros::Subscriber subPointCloud;
ros::ServiceClient cltRgbdRobotDownsampled;
tf::TransformListener * tf_listener;

float minX = 0.3;
float maxX = 0.9;
float minY = -0.25;
float maxY = 0.25;
float z_threshold = 0.05;
int is_obst_counter = 30;

bool getKinectDataFromJustina( cv::Mat& imaBGR, cv::Mat& imaPCL)
{
    point_cloud_manager::GetRgbd srv;
    if(!cltRgbdRobotDownsampled.call(srv))
    {
        //std::cout << "obs_detect_node.->Cannot get point cloud" << std::endl;
        return false;
    }
    JustinaTools::PointCloud2Msg_ToCvMat(srv.response.point_cloud, imaBGR, imaPCL);
    return true; 
}

void callbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laserScan = *msg;
}

void callbackPath(const nav_msgs::Path::ConstPtr& msg)
{
    lastPath = *msg;
    lastPathTransform = *msg;
    tf::StampedTransform transformTf;
    tf_listener->lookupTransform("kinect_link", "map", ros::Time(0), transformTf);
    for(int j = 0; j < lastPathTransform.poses.size(); j++){
        lastPathTransform.header.frame_id = "kinect_link";
        float xpath = lastPathTransform.poses[j].pose.position.x;
        float ypath = lastPathTransform.poses[j].pose.position.y;
        float zpath = lastPathTransform.poses[j].pose.position.z;
        tf::Vector3 v(xpath, ypath, zpath);
        v = transformTf * v;
        lastPathTransform.poses[j].pose.position.x = v.getX();
        lastPathTransform.poses[j].pose.position.y = v.getY();
        lastPathTransform.poses[j].pose.position.z = v.getZ();
    }
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
        // subPointCloud = nh->subscribe("/hardware/point_cloud_man/rgbd_wrt_robot_downsampled", 1, callbackPointCloud);
        cv::namedWindow("OBSTACLE DETECTOR BY MARCOSOFT", cv::WINDOW_AUTOSIZE);
    }
    else
    {
        std::cout << "ObsDetector.->Stopping obstacle detection using point cloud..." << std::endl;
        subPointCloud.shutdown();
        try
        {
            cv::destroyWindow("OBSTACLE DETECTOR BY MARCOSOFT");
        }
        catch(cv::Exception e)
        {
            std::cerr << e.what() << std::endl;
        }
        catch(...)
        {
            std::cerr << "ObsDetector.->I dont know what is the fucking problem." << std::endl;
        }
    }
    enable = msg->data;
}

void callbackEnableDoorDetector(const std_msgs::Bool::ConstPtr& msg)
{
    enableDoorDetector = msg->data;
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

bool newCollisionRiskWithKinect(int pointAheadIdx, float robotX, float robotY, float robotTheta, float& collisionX, float& collisionY){
    cv::Mat xyzCloudSeg;
    std::vector< cv::Point2i > indexes; 
    getKinectDataFromJustina(bgrImg, xyzCloud);
    xyzCloud.copyTo(xyzCloudSeg);
    tf::StampedTransform transformTf;
    tf_listener->lookupTransform("kinect_link", "base_link", ros::Time(0), transformTf);
    float justina_hight = transformTf.getOrigin().getY();
    std::cout << "justina_height.->" << justina_hight << std::endl;
    if(bgrImg.cols < 1 || bgrImg.rows < 1)
        return false;

    int counter = 0;
    float meanX = 0;
    float meanY = 0;
    for(int i = 0; i< xyzCloudSeg.cols; i++)
        for(int j=0; j< xyzCloudSeg.rows; j++)
        {
            cv::Vec3f p = xyzCloudSeg.at<cv::Vec3f>(j,i);
            /*if(p[2] < z_threshold)
            {
                bgrImg.data[3*(j*bgrImg.cols + i)] = 0;
                bgrImg.data[3*(j*bgrImg.cols + i) + 1] = 0;
                bgrImg.data[3*(j*bgrImg.cols + i) + 2] = 0;
            }*/
            // if(!(p[0] >= minX && p[0] <= maxX && p[1] >= minY && p[1] <= maxY && p[2] >= z_threshold && p[2] < 1.0))
            // if(!(p[0] >= minX && p[0] <= maxX && p[2] >= z_threshold && p[2] < 1.0))
            // if(!(p[2] >= minX && p[2] <= maxX && p[0] >= minY && p[0] <= maxY && p[1] <= (justina_hight - z_threshold) && p[1] > (justina_hight - 1.0)))
            if(!(p[2] >= minX && p[2] <= maxX && p[1] <= (justina_hight - z_threshold) && p[1] > (justina_hight - 1.0)))
            {
                xyzCloudSeg.at<cv::Vec3f>(j, i)[0] = 0.0; 
                xyzCloudSeg.at<cv::Vec3f>(j, i)[1] = 0.0; 
                xyzCloudSeg.at<cv::Vec3f>(j, i)[2] = 0.0; 
                bgrImg.data[3*(j*bgrImg.cols + i)] = 0;
                bgrImg.data[3*(j*bgrImg.cols + i) + 1] = 0;
                bgrImg.data[3*(j*bgrImg.cols + i) + 2] = 0;
            }
            else if(p[0] != 0 && p[1] != 0 && p[2] != 0){
                counter++;
                meanX += p[0];
                meanY += p[1];
                indexes.push_back( cv::Point(j, i) );
            }
        }
    cv::imshow("OBSTACLE DETECTOR BY MARCOSOFT", bgrImg);
    if(current_speed_linear < 0.1)
        return false;
    if(counter <= is_obst_counter)
        return false;
    counter = 0;
    for(int i = 0; i < indexes.size(); i++){
        /*float x = xyzCloudSeg.at<cv::Vec3f>(indexes[i])[0];
        float y = xyzCloudSeg.at<cv::Vec3f>(indexes[i])[1];
        float z = 0.0;*/
        float x = xyzCloudSeg.at<cv::Vec3f>(indexes[i])[0];
        float y = 0.0;
        float z = xyzCloudSeg.at<cv::Vec3f>(indexes[i])[2];
        for(int j = 0; j < lastPathTransform.poses.size(); j++){
            /*float xpath = lastPathTransform.poses[j].pose.position.x;
            float ypath = lastPathTransform.poses[j].pose.position.y;
            float zpath = 0.0;*/
            float xpath = lastPathTransform.poses[j].pose.position.x;
            float ypath = 0.0;
            float zpath = lastPathTransform.poses[j].pose.position.z;
            cv::Vec3f pathPoint(xpath, ypath, zpath);
            cv::Vec3f pclPoint(x, y, z);
            float norma = sqrt( pow(pclPoint[0] - pathPoint[0], 2) + pow(pclPoint[1] - pathPoint[1], 2) + pow(pclPoint[2] - pathPoint[2], 2) );
            // std::cout << "Norma.->" << norma << std::endl;
            if(norma <= 0.26)
                counter++;
        }
    }
    collisionX = counter > is_obst_counter ? meanX / counter : 0;
    collisionY = counter > is_obst_counter ? meanY / counter : 0;
    if(current_speed_linear < 0.1)
        return false;
    float distanceToCollision = sqrt(pow(collisionX, 2) + pow(collisionY, 2));
    // TODO TEST IF THIS IS THE CORRECT DISTANCE AND PUT IN THE LAUNCH CONFIGURATION
    if(distanceToCollision > 0.6)
        return false;
    if(counter > is_obst_counter)
        return true;
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

    if(dist < 0.23) dist = 0.23;
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
        if(angle > minSearchAngle && angle < maxSearchAngle && laserScan.ranges[i] < dist && laserScan.ranges[i] > 0.23)
            counter++;
    }
    //std::cout << "ObsDetect.->: " << minSearchAngle << "  " << maxSearchAngle << "  " << dist << "  " << minCounter << std::endl;
    if(counter >= minCounter)
        std::cout << "ObsDetect.->Collision risk detected with láser: min-max-counting: " << minSearchAngle << "  "
            << maxSearchAngle << "  " << counter << std::endl;
    return counter >= minCounter;
}

bool collisionRiskWithKinect(int pointAheadIdx, float robotX, float robotY, float robotTheta, float& collisionX, float& collisionY)
{
    getKinectDataFromJustina(bgrImg, xyzCloud);
    if(bgrImg.cols < 1 || bgrImg.rows < 1)
        return false;

    //std::cout << "ObsDetect.->Starting detection" << std::endl;

    //std::cout << "ObsDetect.->Point ahead calculated" << std::endl;
    //Searchs for possibles collisions only when robot is already pointing to several points ahead in the current path
    //i.e. when the error angle is around zero
    //std::cout << "ObsDetect.->Point cloud size: " << xyzCloud.cols <<"x" << xyzCloud.rows<< std::endl;
    //Since coordinates are wrt robot, it searches only in a rectangle in front of the robot
    int counter = 0;
    float meanX = 0;
    float meanY = 0;
    for(int i=0; i< xyzCloud.cols; i++)
        for(int j=0; j< xyzCloud.rows; j++)
        {
            cv::Vec3f p = xyzCloud.at<cv::Vec3f>(j,i);
            if(p[2] < z_threshold)
            {
                bgrImg.data[3*(j*bgrImg.cols + i)] = 0;
                bgrImg.data[3*(j*bgrImg.cols + i) + 1] = 0;
                bgrImg.data[3*(j*bgrImg.cols + i) + 2] = 0;
            }
            if(p[0] >= minX && p[0] <= maxX && p[1] >= minY && p[1] <= maxY && p[2] >= z_threshold && p[2] < 1.0) 
            {
                counter++;
                meanX += p[0];
                meanY += p[1];
            }
        }
    //std::cout << "ObsDetect.->Color modified" << std::endl;
    cv::imshow("OBSTACLE DETECTOR BY MARCOSOFT", bgrImg);

    //float aheadX = lastPath.poses[pointAheadIdx].pose.position.x;
    //float aheadY = lastPath.poses[pointAheadIdx].pose.position.y;
    //float errorX = aheadX - robotX;
    //float errorY = aheadY - robotY;
    //float errorAngle = atan2(errorY, errorX) - robotTheta;
    //if(errorAngle > M_PI) errorAngle -= 2*M_PI;
    //if(errorAngle <= -M_PI) errorAngle += 2*M_PI;

    //if(fabs(errorAngle) > 0.17)
    //    return false;

    //if(counter > 0)
    //    std::cout << "ObsDetect.->Collision risk detected with kinect: angle-counting: " << errorAngle << "  " << counter << std::endl;
    collisionX = counter > is_obst_counter ? meanX / counter : 0;
    collisionY = counter > is_obst_counter ? meanY / counter : 0;
    if(current_speed_linear < 0.1)
        return false;
    return counter > is_obst_counter;
}

bool detectDoorInFront()
{
    int range=0,range_i=0,range_f=0,range_c=0,cont_laser=0;
    float laser_l=0;
    range=laserScan.ranges.size();
    range_c=range/2;
    range_i=range_c-(range/10);
    range_f=range_c+(range/10);
    //std::cout<<"Range Size: "<< range << "\n ";
    //std::cout<<"Range Central: "<< range_c << "\n ";
    //std::cout<<"Range Initial: "<< range_i << "\n ";
    //std::cout<<"Range Final: "<< range_f << "\n ";

    cont_laser=0;
    laser_l=0;
    for(int i=range_c-(range/10); i < range_c+(range/10); i++)
    {
        if(laserScan.ranges[i] > 0 && laserScan.ranges[i] < 4){ 
            laser_l=laser_l+laserScan.ranges[i]; 
            cont_laser++;
        }
    }
    //std::cout<<"Laser promedio: "<< laser_l/cont_laser << std::endl;    
    if(laser_l/cont_laser > 2.0)
        return true;
    return false;
}

bool detectDoorInFront2()
{
    float theta = laserScan.angle_min;
    int laserCount = 0;
    int totalCount = 0;
    for(int i = 0; i < laserScan.ranges.size(); i++)
    {
        float x, y;
        theta = laserScan.angle_min + i*laserScan.angle_increment;
        x = laserScan.ranges[i] * cos(theta);
        y = laserScan.ranges[i] * sin(theta);
        if(theta >= -0.52 && theta <= 0.52)
        {
            totalCount++;
            if(x >= 0.05 && x <= 2.0 && y >= -0.4 && y <= 0.4)
                laserCount++;
        }
    }
    float media = (float)laserCount / (float)totalCount;
    std::cout << "obs_detect_node.->Media:" << media << std::endl;
    if(media > 0.8)
        return true;
    return false;
}

void callback_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg)
{
    current_speed_linear  = msg->linear.x;
    current_speed_angular = msg->angular.z;
}

int main(int argc, char** argv)
{
    // TODO REMOVE THIS COMMENTED LINES WHEN TEST THE NEW FORM OF PASS THE PARAMS
    /*for(int i=0; i < argc; i++)
    {
        std::string strParam(argv[i]);
        float value;
        if(strParam.compare("--min_x") == 0)
        {
            std::stringstream ss(argv[++i]);
            if(ss >> value)
                minX = value;
        }
        if(strParam.compare("--max_x") == 0)
        {
            std::stringstream ss(argv[++i]);
            if(ss >> value)
                maxX = value;
        }
        if(strParam.compare("--min_y") == 0)
        {
            std::stringstream ss(argv[++i]);
            if(ss >> value)
                minY = value;
        }
        if(strParam.compare("--max_y") == 0)
        {
            std::stringstream ss(argv[++i]);
            if(ss >> value)
                maxY = value;
        }
        if(strParam.compare("--z_threshold") == 0)
        {
            std::stringstream ss(argv[++i]);
            if(ss >> value)
                z_threshold = value;
        }
    }*/

    std::cout << "INITIALIZING OBSTACLE DETECTOR (ONLY LASER) NODE BY MARCOSOFT... " << std::endl;
    ros::init(argc, argv, "obs_detect");
    ros::NodeHandle n;

    if(ros::param::has("~min_x"))
        ros::param::get("~min_x", minX);
    if(ros::param::has("~max_x"))
        ros::param::get("~max_x", maxX);
    if(ros::param::has("~min_y"))
        ros::param::get("~min_y", minY);
    if(ros::param::has("~max_y"))
        ros::param::get("~max_y", maxY);
    if(ros::param::has("~z_threshold"))
        ros::param::get("~z_threshold", z_threshold);
    if(ros::param::has("~is_obst_counter"))
        ros::param::get("~is_obst_counter", is_obst_counter);

    nh = &n;
    ros::Subscriber subLaserScan = n.subscribe("/hardware/scan", 1, callbackLaserScan);
    ros::Subscriber subPath = n.subscribe("/navigation/mvn_pln/last_calc_path", 1, callbackPath);
    ros::Subscriber subEnable = n.subscribe("/navigation/obs_avoid/enable", 1, callbackEnable);
    ros::Subscriber sub_cmd_vel = n.subscribe("/hardware/mobile_base/cmd_vel", 1, callback_cmd_vel);
    ros::Subscriber subEnableDoorDetector = n.subscribe("/navigation/obs_avoid/enable_door_detector", 1, callbackEnableDoorDetector);
    ros::Publisher pubObstacleInFront = n.advertise<std_msgs::Bool>("/navigation/obs_avoid/obs_in_front", 1);
    ros::Publisher pubCollisionRisk = n.advertise<std_msgs::Bool>("/navigation/obs_avoid/collision_risk", 1);
    ros::Publisher pubCollisionPoint = n.advertise<geometry_msgs::PointStamped>("/navigation/obs_avoid/collision_point", 1);
    ros::Publisher pubDetectedDoor = n.advertise<std_msgs::Bool>("/navigation/obs_avoid/detected_door", 1);
    cltRgbdRobotDownsampled = n.serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot_downsampled");
    tf_listener = new tf::TransformListener();
    JustinaTools::setNodeHandle(&n);
    ros::Rate loop(30);

    std::cout << "ObsDetect.->Using parameters: min_x=" << minX << "\tmax_x=" << maxX << "\tmin_y=" << minY << "\tmax_y" << maxY << "\tz_threshold" << z_threshold << std::endl;

    std_msgs::Bool msgObsInFront;
    std_msgs::Bool msgCollisionRisk;
    std_msgs::Bool msgDetectedDoor;
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
    tf_listener->waitForTransform("map", "base_link", ros::Time(0), ros::Duration(10.0));

    while(ros::ok() && cv::waitKey(15) != 27)
    {
        //Getting robot position
        tf_listener->lookupTransform("map", "base_link", ros::Time(0), tf);
        robotX = tf.getOrigin().x();
        robotY = tf.getOrigin().y();
        q = tf.getRotation();
        robotTheta = atan2((float)q.z(), (float)q.w()) * 2;

        //Calculating position 20 path-steps ahead the robot
        int aheadIdx = getLookAheadPathIdx(robotX, robotY);
        //std::cout << "ObstacleDetector.->Next path index: " << getLookAheadPathIdx(robotX, robotY) << std::endl;

        if(enable)
        {
            msgCollisionRisk.data = collisionRiskWithKinect(aheadIdx, robotX, robotY, robotTheta, collisionX, collisionY);
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

        if(enableDoorDetector)
        {
            msgDetectedDoor.data = detectDoorInFront2();
            pubDetectedDoor.publish(msgDetectedDoor);
        }

        ros::spinOnce();
        loop.sleep();
    }
    delete tf_listener;
}
