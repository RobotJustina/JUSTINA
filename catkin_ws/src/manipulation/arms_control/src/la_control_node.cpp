#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <cmath>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/filesystem/path.hpp>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Bool.h"
#include "manip_msgs/SpeedProfile.h"
#include "ros/ros.h"


#define SAMPLING_FREQ            50
#define SM_WAIT_FOR_NEW_POSE      0
#define SM_SENDING_PROFILES      10
#define SM_WAIT_FOR_GOAL_REACHED 20

std::vector<float> global_goal_angular ;
std::vector<float> goal_cartesian_pose ;
std::vector<float> current_angular_pose;
std::vector<float> current_angular_speed (7); 
std::map<std::string, std::vector<float> > laPredefPoses;
std::map<std::string, std::vector<float> > loadArrayOfFloats(std::string path);

ros::Publisher pub_go_to_angles;
std_msgs::Float32MultiArray msg_go_to_angles;

bool new_global_goal = false;
double trajectory_time;


std::map<std::string, std::vector<float> > loadArrayOfFloats(std::string path)
{
    std::cout << "ManipPln.->Extracting array of floats from file: " << path << std::endl;
    std::vector<std::string> lines;
    std::ifstream file(path.c_str());
    std::string tempStr;
    while(std::getline(file, tempStr))
        lines.push_back(tempStr);

    //Extraction of lines without comments
    for(size_t i=0; i < lines.size(); i++)
    {
        size_t idx = lines[i].find("//");
        if(idx != std::string::npos)
            lines[i] = lines[i].substr(0, idx);
    }

    std::map<std::string, std::vector<float> > data;

    float fValue;
    bool parseSuccess;
    for(size_t i=0; i<lines.size(); i++)
    {
        //std::cout << "ManipPln.->Parsing line: " << lines[i] << std::endl;
        std::vector<std::string> parts;
        boost::split(parts, lines[i], boost::is_any_of(" ,\t"), boost::token_compress_on);
        if(parts.size() < 2)
            continue;
        //First part should be the label and the next ones, the values
        if(!boost::filesystem::portable_posix_name(parts[0]))
            continue;
        parseSuccess = true;
        for(size_t j=1; j<parts.size() && parseSuccess; j++)
        {
            std::stringstream ssValue(parts[j]);
            if(!(ssValue >> fValue)) parseSuccess = false;
            else data[parts[0]].push_back(fValue);
        }
    }
    return data;
}

bool loadPredefinedPosesAndMovements(std::string folder)
{ 
    std::string leftArmPosesFile = folder + "left_arm_poses.txt";
    std::map<std::string, std::vector<float> > data = loadArrayOfFloats(leftArmPosesFile);

    std::cout<<"la_contol_node has received new a msg..."<<std::endl;
    for(std::map<std::string, std::vector<float> >::iterator i = data.begin(); i != data.end(); i++)
    {
        if(i->second.size() != 7)
        {
            std::cout << "ManipPln.->Invalid number of angles in left arm predef position " << i->first << std::endl;
            continue;
        }
        laPredefPoses[i->first] = i->second;

    }
    std::cout << "ManipPln.->Left arm predefined positions: " <<std::endl;
    for(std::map<std::string, std::vector<float> >::iterator i = laPredefPoses.begin(); i != laPredefPoses.end(); i++)
    {
        std::cout << i->first << " ";
        for(int j=0; j < i->second.size(); j++)
            std::cout << i->second[j] << " ";
        std::cout << std::endl;
    }
    return true;
}

void callback_la_go_to_loc(const std_msgs::String::ConstPtr& msg)
{

    if(laPredefPoses.find(msg->data) == laPredefPoses.end())
    {
        std::cout << "ManipPln.->Cannot find left arm predefined position: " << msg->data << std::endl;
        return;
    }
    std::cout << "ManipPln.->Left Arm goal pose: " << msg->data << " = ";
    msg_go_to_angles.data.resize(7);
    for(int i=0; i< laPredefPoses[msg->data].size(); i++)
    { 
        std::cout << laPredefPoses[msg->data][i] << " ";
        msg_go_to_angles.data[i] = laPredefPoses[msg->data][i];
    }
    pub_go_to_angles.publish(msg_go_to_angles);

    std::cout << std::endl;
    std_msgs::Bool msgGoalReached;
    msgGoalReached.data = false;
}

void callback_la_goto_angles(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    std::cout << "la_control.->Received new global goal angles: ";
    for(int i=0; i < msg->data.size(); i++)
        std::cout << msg->data[i] << "  ";
    std::cout << std::endl;

    global_goal_angular = msg->data;
    new_global_goal = true;
}

void callback_la_current_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    current_angular_pose = msg->data;
}

bool get_speed_profiles(ros::ServiceClient& clt, std::vector<std::vector<float> >& positions,
    std::vector<std::vector<float> >& speeds)
{
    manip_msgs::SpeedProfile srv;
    srv.request.dt = 1.0/SAMPLING_FREQ;
    srv.request.t0 = 0;
    srv.request.tf = trajectory_time;
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
    std::cout << "INITIALIZING LEFT ARM LOW LEVEL CONTROL BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "la_control");
    ros::NodeHandle node("~");
                       pub_go_to_angles  = node.advertise<std_msgs::Float32MultiArray>("/manipulation/manip_pln/la_goto_angles", 1000);    
    ros::Subscriber    sub_go_to_angles  = node.subscribe("/manipulation/manip_pln/la_goto_angles", 1, callback_la_goto_angles);
    ros::Subscriber    sub_la_current    = node.subscribe("/hardware/left_arm/current_pose", 1, callback_la_current_pose);
    ros::Subscriber    sub_la_go_to_loc  = node.subscribe("/manipulation/manip_pln/la_goto_loc", 1, callback_la_go_to_loc);    
    ros::Publisher     pub_la_goal_pose  = node.advertise<std_msgs::Float32MultiArray>("/hardware/left_arm/goal_pose", 1000);
    ros::Publisher  pub_la_goal_reached  = node.advertise<std_msgs::Bool>("/manipulation/la_goal_reached", 1000);    
    ros::ServiceClient clt_speed_profile = node.serviceClient<manip_msgs::SpeedProfile>("/manipulation/get_speed_profile");
    ros::Rate loop(SAMPLING_FREQ);

    int time_k = 0;
    int state  = SM_WAIT_FOR_NEW_POSE;
    std::vector<std::vector<float> > profile_positions  ;
    std::vector<std::vector<float> > profile_speeds     ;
    std_msgs::Float32MultiArray      msg_la_goal_pose   ;
    std_msgs::Bool                   msg_la_goal_reached;

    node.setParam("/manipulation/la_control/trajectory_time", 1.5);    
    
    std::string folder = "";
    for(int i=0; i < argc; i++)
    {
        std::string strParam(argv[i]);
        if(strParam.compare("-f") == 0)
            folder = argv[++i];
    }

    loadPredefinedPosesAndMovements(folder);
    
    while(ros::ok())
    {
        switch(state)
        {
        case SM_WAIT_FOR_NEW_POSE:
            if(new_global_goal)
            {
                new_global_goal = false;
                time_k = 0;
                msg_la_goal_pose.data.resize(14);
                node.param("trajectory_time", trajectory_time, 1.5);
                if(get_speed_profiles(clt_speed_profile, profile_positions, profile_speeds))
                    state = SM_SENDING_PROFILES;
                else
                    std::cout << "la_control.->Error!!! Cannot calculate position and speed profiles :'( " << std::endl;
            }
            break;
        case SM_SENDING_PROFILES:
            if(!new_global_goal)  //----------
            {                             //---------- 
                for(int i=0; i < 7; i++)
                {
                    msg_la_goal_pose.data[i  ] = profile_positions[i][time_k];
                    msg_la_goal_pose.data[i+7] = profile_speeds   [i][time_k];
                    current_angular_speed[i] = profile_speeds[i][time_k];//----------
                }
                pub_la_goal_pose.publish(msg_la_goal_pose);
                if(time_k++ >= profile_positions[0].size())
                {
                	msg_la_goal_pose .data.resize(7);
                	for(int i=0; i < 7; i++)
                		msg_la_goal_pose.data[i] = profile_positions[i][profile_positions[i].size()-1];
                	pub_la_goal_pose.publish(msg_la_goal_pose);	
                    state = SM_WAIT_FOR_GOAL_REACHED;
                }
            }                  //-----------
            else               //----------- 
                state = SM_WAIT_FOR_NEW_POSE;//---------

            break;
        case SM_WAIT_FOR_GOAL_REACHED:
            node.setParam("/manipulation/la_control/trajectory_time", 1.5); 
            msg_la_goal_reached.data = true;
            pub_la_goal_reached.publish(msg_la_goal_reached);
            state = SM_WAIT_FOR_NEW_POSE;
            break;
        default:
            std::cout << "la_control.->Sorry. Somebody really stupid programmed this shit. " << std::endl;
        }
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}
