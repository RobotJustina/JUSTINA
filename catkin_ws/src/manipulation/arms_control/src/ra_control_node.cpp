#include <iostream>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Bool.h"
#include "manip_msgs/SpeedProfile.h"
#include "ros/ros.h"

#define SAMPLING_FREQ 50
#define SM_WAIT_FOR_NEW_POSE      0
#define SM_SENDING_PROFILES      10
#define SM_WAIT_FOR_GOAL_REACHED 20

std::vector<float> global_goal_angular ;
std::vector<float> goal_cartesian_pose ;
std::vector<float> current_angular_pose;
std::vector<float> current_angular_speed (7); //--------
bool new_global_goal = false;


void callback_ra_goto_angles(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    std::cout << "ra_control.->Received new global goal angles: ";
    for(int i=0; i < msg->data.size(); i++)
        std::cout << msg->data[i] << "  ";
    std::cout << std::endl;

    global_goal_angular = msg->data;
    new_global_goal = true;
}

void callback_ra_current_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    current_angular_pose = msg->data;
}

bool get_speed_profiles(ros::ServiceClient& clt, std::vector<std::vector<float> >& positions,
    std::vector<std::vector<float> >& speeds)
{
    manip_msgs::SpeedProfile srv;
    srv.request.dt = 1.0/SAMPLING_FREQ;
    srv.request.t0 = 0;
    srv.request.tf = 1.3;
    srv.request.p0 = 0;
    srv.request.pf = 1;
    srv.request.w0 = 0;
    srv.request.wf = 0;
    srv.request.a0 = 0;
    srv.request.af = 0;
    positions.resize(7);
    speeds   .resize(7);
    for(int i=0; i< 7; i++)
    {
        srv.request.p0 = current_angular_pose[i];
        srv.request.pf = global_goal_angular [i];
//        srv.request.w0 = current_angular_speed[i];//-------
        if(clt.call(srv))
        {
            positions[i] = srv.response.positions.data;
            speeds   [i] = srv.response.speeds   .data;
        }
        else
            return false;
    }
    return true;
}

float get_max_angle_error()
{
    float max = 0;
    for(int i=0; i < 7; i++)
        if(fabs(global_goal_angular[i] - current_angular_pose[i]) > max)
            max = fabs(global_goal_angular[i] - current_angular_pose[i]);
    return max;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING RIGHT ARM LOW LEVEL CONTROL BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "ra_control");
    ros::NodeHandle n;
    ros::Subscriber    sub_go_to_angles  = n.subscribe("/manipulation/manip_pln/ra_goto_angles", 1, callback_ra_goto_angles);
    ros::Subscriber    sub_ra_current    = n.subscribe("/hardware/right_arm/current_pose", 1, callback_ra_current_pose);
    ros::Publisher     pub_ra_goal_pose  = n.advertise<std_msgs::Float32MultiArray>("/hardware/right_arm/goal_pose", 1000);
    ros::Publisher  pub_ra_goal_reached  = n.advertise<std_msgs::Bool>("/manipulation/ra_goal_reached", 1000);    
    ros::ServiceClient clt_speed_profile = n.serviceClient<manip_msgs::SpeedProfile>("/manipulation/get_speed_profile");
    ros::Rate loop(SAMPLING_FREQ);

    int time_k = 0;
    int state  = SM_WAIT_FOR_NEW_POSE;
    std::vector<std::vector<float> > profile_positions  ;
    std::vector<std::vector<float> > profile_speeds     ;
    std_msgs::Float32MultiArray      msg_ra_goal_pose   ;
    std_msgs::Bool                   msg_ra_goal_reached;

    //system("echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer"); 
    //system("echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB1/latency_timer");
    //system("echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB2/latency_timer");

    while(ros::ok())
    {
        switch(state)
        {
        case SM_WAIT_FOR_NEW_POSE:
            if(new_global_goal)
            {
                new_global_goal = false;
                time_k = 0;
                msg_ra_goal_pose.data.resize(14);
                if(get_speed_profiles(clt_speed_profile, profile_positions, profile_speeds))
                    state = SM_SENDING_PROFILES;
                else
                    std::cout << "ra_control.->Error!!! Cannot calculate position and speed profiles :'( " << std::endl;
            }
            break;
        case SM_SENDING_PROFILES:
            if(new_global_goal == false)  //----------
            {                             //---------- 
                for(int i=0; i < 7; i++)
                {
                    msg_ra_goal_pose.data[i  ] = profile_positions[i][time_k];
                    msg_ra_goal_pose.data[i+7] = profile_speeds   [i][time_k];
                    current_angular_speed[i] = profile_speeds[i][time_k];//----------
                }
                pub_ra_goal_pose.publish(msg_ra_goal_pose);
                if(time_k++ >= profile_positions[0].size())
                {
                    msg_ra_goal_pose .data.resize(7);
                    for(int i=0; i < 7; i++)
                        msg_ra_goal_pose.data[i] = profile_positions[i][profile_positions[i].size()-1];
                    pub_ra_goal_pose.publish(msg_ra_goal_pose); 
                    state = SM_WAIT_FOR_GOAL_REACHED;
                }
            }                  //-----------
            else               //----------- 
                state = SM_WAIT_FOR_NEW_POSE;//---------

            break;
        case SM_WAIT_FOR_GOAL_REACHED:
            msg_ra_goal_reached.data = true;
            pub_ra_goal_reached.publish(msg_ra_goal_reached);
            state = SM_WAIT_FOR_NEW_POSE;
            break;
        default:
            std::cout << "ra_control.-> error. " << std::endl;
        }
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}
