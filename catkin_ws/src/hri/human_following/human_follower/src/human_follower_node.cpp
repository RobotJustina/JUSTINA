   #include <iostream>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "LowLevelControl.h"

double LegX, LegY;
//double RobotX, RobotY, RobotTheta;
bool StartFollow;
bool LegsPoseUpdate = false;

void callbackLegPose(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    //This topic contains the absolute position of the detected legs. leg_finder node publishes this topic.
    //An array is used due to the possibility of finding more than one pair of legs.
    //Header indicates the frame w.r.t the positions are expressed. Use of 'map' frame is preferred.
    LegX=msg->point.x;
    LegY=msg->point.y;
    LegsPoseUpdate = true;
}

//void callbackCurrentPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
//{
//    //This topic constains the robot pose w.r.t map. It is published by the localization nod (amcl)
//}

void callbackStartFollow(const std_msgs::Bool::ConstPtr& msg)
{
    //Signal for starting to follow human
    StartFollow = msg->data;
}


int main(int argc, char** argv)
{
    std::cout << "INITIALIZING HUMAN FOLLOWER BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "human_follower");
    ros::NodeHandle n;
    ros::Subscriber subLegPose = n.subscribe("/hri/human_following/leg_poses", 1, callbackLegPose);
    //ros::Subscriber subRobotPose = n.subscribe("/navigation/localization/current_pose", 1, callbackCurrentPose);
    ros::Subscriber subStartFollow = n.subscribe("/hri/human_following/start_follow", 1, callbackStartFollow);
    //ros::Subscriber subStopFollow = n.subscribe("/hri/human_following/stop_follow", 1, callbackStartFollow);
    ros::Publisher pubRobotSpeeds = n.advertise<std_msgs::Float32MultiArray>("/hardware/mobile_base/speeds", 1);
    ros::Rate loop(10);
    LowLevelControl lLControl;// = LowLevelControl();
    std_msgs::Float32MultiArray speeds;
    speeds.data.push_back(0);
    speeds.data.push_back(0);
    

    while(ros::ok())
    {
        if (LegsPoseUpdate && StartFollow)// && ((LegX * LegX) + (LegY*LegY)) > 0.49)
        {
            LegsPoseUpdate = false;
            lLControl.CalculateSpeeds(0.0, 0.0 , 0.0 , (float)LegX , (float)LegY , speeds.data[0],  speeds.data[1], false);
        }
        /*
        else if (StartFollow)
        {
            speeds.data[0]=0;
            speeds.data[1]=0;	
        }*/
        pubRobotSpeeds.publish(speeds);
        ros::spinOnce();
        loop.sleep();
    }
}

