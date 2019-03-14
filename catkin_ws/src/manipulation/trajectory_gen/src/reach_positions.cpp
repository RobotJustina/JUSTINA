#include<ros/ros.h>
#include<std_msgs/Float32MultiArray.h>
#include<vector>
#include "manip_msgs/SpeedProfile.h"
#include <cstdlib>
/*
int main(int argc, char **argv)
{
  ros::init(argc, argv, "SpeedProfile");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<manip_msgs::SpeedProfile>("/manipulation/get_speed_profile");
  manip_msgs::SpeedProfile srv;
  srv.request.dt = 0.02;
  srv.request.t0 = 0;
  srv.request.tf = 1.5;
  srv.request.p0 = 0;
  srv.request.pf = 1;

  if (client.call(srv))
  {
      std::cout<<"Number of points: "<<srv.response.speeds.data.size()<<std::endl;
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}//*/

#define ID_O        0
#define ID_1        1
#define ID_2        2
#define ID_3        3
#define ID_4        4
#define ID_5        5
#define ID_6        6

#define DYNAMIXEL_MAX_VEL 12.259 // 12.259 rad/s  
#define TOTAL_TIME        3.5   // seconds
#define DELTA_T           0.02   // sampling step [s]

using namespace std;
using namespace ros;


int record = 0;
bool connection = false;

float goalPos[7] = {0, 0, 0, 0, 0, 0, 0};

vector<vector<float> >  goalSpeeds;
vector<vector<float> >  goalPositions;

float point_numbers = TOTAL_TIME / DELTA_T;



bool speeds_record(ros::ServiceClient& client);


void posesCallback(const std_msgs::Float32MultiArray::ConstPtr &msg){
        if(!(msg->data.size() == 7 || msg->data.size() == 14))
                cout << "Can not process the goal poses for the left arm" << endl;
        else{
                connection = true;
            
                goalPos[0] =  msg->data[0];
                goalPos[1] =  msg->data[1];
                goalPos[2] =  msg->data[2];
                goalPos[3] =  msg->data[3];
                goalPos[4] =  msg->data[4];
                goalPos[5] =  msg->data[5];
                goalPos[6] =  msg->data[6];                
        }

}

int main(int argc, char **argv){
  	cout<<"Initializing reach_positions node..."<<endl;
  	init(argc, argv, "reach_positions");
  	NodeHandle node;

    goalSpeeds.resize(7);
    goalPositions.resize(7);
    
    Subscriber getGoalPos = node.subscribe("/x", 1000, posesCallback);
    Publisher pubGoalPos = node.advertise<std_msgs::Float32MultiArray>("/hardware/left_arm/goal_pose", 1000);
    ServiceClient client = node.serviceClient<manip_msgs::SpeedProfile>("/manipulation/get_speed_profile");

    Rate loop_rate(50);
    std_msgs::Float32MultiArray msg;


    while(connection == false){
        cout<<"Waiting for goal poses..."<<endl;        
        spinOnce();

        loop_rate.sleep();
    }
    

    if(!speeds_record(client))
    {
        cout<<"Cannot calculate speeds profiles..."<<endl;
        return -1;        
    }

    //Duration(2).sleep();
    std::cout << "Profile speeds calculated" << std::endl;
    msg.data.resize(14);
    while(ok()){
      //msg.data.clear();
        //Setting positions for dynamixels
        for(int i=0; i<7; i++)
	  //msg.data[i] = goalPose[i];
	  msg.data[i] = goalPositions[i][record];
	  //msg.data.push_back(goalPos[i]);          

        //Seting velocities for dynamixels
        for(int i=7; i<14; i++){
	  msg.data[i] = goalSpeeds[i-7][record];
	  /*
            if(i == 7)
                msg.data.push_back(goalSpeeds[ID_O][record]);
            else if(i == 8)
                msg.data.push_back(goalSpeeds[ID_1][record]);
            else if(i == 9)
                msg.data.push_back(goalSpeeds[ID_2][record]);
            else if(i == 10)
                msg.data.push_back(goalSpeeds[ID_3][record]);
            else if(i == 11)
                msg.data.push_back(goalSpeeds[ID_4][record]);
            else if(i == 12)
                msg.data.push_back(goalSpeeds[ID_5][record]);
            else if(i == 13)
                msg.data.push_back(goalSpeeds[ID_6][record]);
            else
                msg.data.push_back(0);*/
        }

        record++;

    	pubGoalPos.publish(msg);

    	spinOnce();
        loop_rate.sleep();

        if(record >= goalSpeeds[0].size())
          return 0;
        //if(record > point_numbers)
        //    return 0;
     }//From while(ok())   
}//From int main()



bool speeds_record(ros::ServiceClient& client)
{
    manip_msgs::SpeedProfile srv;
    srv.request.dt = 0.02;
    srv.request.t0 = 0;
    srv.request.tf = 1.3;
    srv.request.p0 = 0;
    srv.request.pf = 1;
    srv.request.w0 = 0;
    srv.request.wf = 0;
    srv.request.a0 = 0;
    srv.request.af = 0;

    for(int i =0 ; i< 7 ; i++)
    {
        srv.request.pf = goalPos[i];
        goalSpeeds[i].clear();
        if(client.call(srv)){
          goalSpeeds[i] = srv.response.speeds.data;
	  goalPositions[i] = srv.response.positions.data;
        }
        else
          return false;        
    }
    
    return true;

    for (int i=0; i< 7; i++)
      goalSpeeds[i].resize(point_numbers);
    for(int i=0 ; i <= point_numbers ; i++){
        goalSpeeds[ID_O][i] = abs(goalPos[ID_O] * (30*pow(DELTA_T*i, 2)/pow(TOTAL_TIME,3)  -  60*pow(DELTA_T*i,3)/pow(TOTAL_TIME,4) 
                                                         +  30*pow(DELTA_T*i,4)/pow(TOTAL_TIME,5)))/DYNAMIXEL_MAX_VEL;

        goalSpeeds[ID_1][i] = abs(goalPos[ID_1] * (30*pow(DELTA_T*i, 2)/pow(TOTAL_TIME,3)  -  60*pow(DELTA_T*i,3)/pow(TOTAL_TIME,4) 
                                                         +  30*pow(DELTA_T*i,4)/pow(TOTAL_TIME,5)))/DYNAMIXEL_MAX_VEL;
        goalSpeeds[ID_2][i] = abs(goalPos[ID_2] * (30*pow(DELTA_T*i, 2)/pow(TOTAL_TIME,3)  -  60*pow(DELTA_T*i,3)/pow(TOTAL_TIME,4) 
                                                         +  30*pow(DELTA_T*i,4)/pow(TOTAL_TIME,5)))/DYNAMIXEL_MAX_VEL;
        goalSpeeds[ID_3][i] = abs(goalPos[ID_3] * (30*pow(DELTA_T*i, 2)/pow(TOTAL_TIME,3)  -  60*pow(DELTA_T*i,3)/pow(TOTAL_TIME,4) 
                                                         +  30*pow(DELTA_T*i,4)/pow(TOTAL_TIME,5)))/DYNAMIXEL_MAX_VEL;
        goalSpeeds[ID_4][i] = abs(goalPos[ID_4] * (30*pow(DELTA_T*i, 2)/pow(TOTAL_TIME,3)  -  60*pow(DELTA_T*i,3)/pow(TOTAL_TIME,4) 
                                                         +  30*pow(DELTA_T*i,4)/pow(TOTAL_TIME,5)))/DYNAMIXEL_MAX_VEL;
        goalSpeeds[ID_5][i] = abs(goalPos[ID_5] * (30*pow(DELTA_T*i, 2)/pow(TOTAL_TIME,3)  -  60*pow(DELTA_T*i,3)/pow(TOTAL_TIME,4) 
                                                         +  30*pow(DELTA_T*i,4)/pow(TOTAL_TIME,5)))/DYNAMIXEL_MAX_VEL; 
        goalSpeeds[ID_6][i] = abs(goalPos[ID_6] * (30*pow(DELTA_T*i, 2)/pow(TOTAL_TIME,3)  -  60*pow(DELTA_T*i,3)/pow(TOTAL_TIME,4) 
                                                         +  30*pow(DELTA_T*i,4)/pow(TOTAL_TIME,5)))/DYNAMIXEL_MAX_VEL;                                                                   
    }
    return true;
}//*/
