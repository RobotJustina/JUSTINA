#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "vision_msgs/TrackedObject.h"

ros::NodeHandle* n;
ros::Subscriber sub_vision_pose;
ros::Subscriber  sub_legs_pose;
ros::Publisher   pub_cmd_vel;
ros::Publisher   pub_head_pose;
bool move_head = false;

geometry_msgs::Twist calculate_speeds(float goal_x, float goal_y)
{
    //Control constants
    float alpha = 1.0;
    //float alpha = 0.9;
    float beta = 0.3;
    float max_angular = 0.7;

    //Error calculation
    float angle_error = atan2(goal_y, goal_x);
    float distance    = sqrt(goal_x*goal_x + goal_y*goal_y);
    distance -= 0.6;
    
    if(distance <   0) 
        distance = 0; //Robot will stop at 0.8 m from walker
    
    if(distance > 0.5) 
        distance = 0.3;
        //distance = 0.5; //Distance is used as speed, so, robot will move at 0.5 max
    
    geometry_msgs::Twist result;
    
    if(distance > 0)
    {
	   result.linear.x  = distance * exp(-(angle_error * angle_error) / alpha);
	   result.linear.y  = 0;
	   result.angular.z = max_angular * (2 / (1 + exp(-angle_error / beta)) - 1);
    }
    else
    {
	   result.linear.x  = 0;
	   result.linear.y  = 0;
	   result.angular.z = 0;
    }
    return result;
}


void callback_vision_pose(const vision_msgs::TrackedObject& msg)
{
    vision_msgs::TrackedObject personPos;
    personPos=msg;
    
    pub_cmd_vel.publish(calculate_speeds(personPos.position.x, personPos.position.y));
    if(move_head)
    {
        std_msgs::Float32MultiArray head_poses;

        if(0.30 >= personPos.position.y && personPos.position.y >= -0.30)
        {
            head_poses.data.push_back(0.0);
            head_poses.data.push_back(-0.2);
        }

        else if(personPos.position.y > 0.30)
        {
            head_poses.data.push_back(0.7);
            head_poses.data.push_back(-0.2);
        }

        else
        {
           head_poses.data.push_back(-0.7);
           head_poses.data.push_back(-0.2); 
        }

        //head_poses.data.push_back(atan2(personPos.position.y, personPos.position.x));
        //head_poses.data.push_back(-0.2);
        pub_head_pose.publish(head_poses);
    }
}

void callback_enable(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data)
    {
	
       std::cout << "VisionFinder.->Enable recevied "<< std::endl;
       sub_vision_pose = n->subscribe("/vision/roi_tracker/tracking_inFront", 1, callback_vision_pose);  
	   pub_cmd_vel   = n->advertise<geometry_msgs::Twist>("/hardware/mobile_base/cmd_vel", 1);
	   pub_head_pose = n->advertise<std_msgs::Float32MultiArray>("/hardware/head/goal_pose", 1);
    }
    else
    {
	   //sub_legs_pose.shutdown();
       std::cout << "VisionFinder.->Disable recevied "<< std::endl; 
       sub_vision_pose.shutdown(); 
	   pub_cmd_vel.shutdown();
	   pub_head_pose.shutdown();
    }
}

int main(int argc, char** argv)
{
    move_head = false;
    for(int i=0; i < argc; i++)
    {
        std::string strParam(argv[i]);
        if(strParam.compare("--move_head") == 0)
            move_head = true;
    }
    
    std::cout << "INITIALIZING HYBRID FOLLOWER..." << std::endl;
    ros::init(argc, argv, "hybrid_follower");
    n = new ros::NodeHandle();
    ros::Subscriber sub_enable = n->subscribe("/hri/hybrid_following/start_follow", 1, callback_enable);
    ros::Rate loop(20);

    while(ros::ok())
    {        
        ros::spinOnce();
        loop.sleep();
    }
    delete n;
}
