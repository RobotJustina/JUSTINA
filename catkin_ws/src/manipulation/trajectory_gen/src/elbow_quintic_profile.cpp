//Program that calculates the trajectory for dynamixels using a quintic profile
#include<ros/ros.h>
#include<std_msgs/Float32MultiArray.h>
#include<vector>

#define OBJETIVE_ANGLE    1.7    // 1.7 rad
#define CONSTANT_VELOCITY 0.13 // 0.613  rad/s
#define DYNAMIXEL_MAX_VEL 12.259 // 12.259 rad/s  

using namespace std;
using namespace ros;

float current_pos = 0;
float current_time = 0;
int record = 0;

float constant_velocity = CONSTANT_VELOCITY * DYNAMIXEL_MAX_VEL; 
float total_time = abs(OBJETIVE_ANGLE/constant_velocity);
int num_velocities = total_time / 0.02;
float* goalSpeeds = new float[num_velocities];

float speeds_record();


int main(int argc, char **argv){
	cout<<"Initializing elbow_quintic_profile node..."<<endl;
	init(argc, argv, "elbow_quintic_profile");
	NodeHandle node;
  

    Publisher pubGoalPos = node.advertise<std_msgs::Float32MultiArray>("/hardware/left_arm/goal_pose", 1000);


    //Setting all speeds of trajectory
    speeds_record();



    std_msgs::Float32MultiArray msg;
    Rate loop_rate(50);


    while(ok()){
    	
        msg.data.clear();
        //Setting positions for dynamixels
        for(int i=0; i<7; i++){
            if(i==3)
                msg.data.push_back(OBJETIVE_ANGLE);
            else
                msg.data.push_back(0);          
        }
        //Seting velocities for dynamixels
        for(int i=7; i<14; i++){
            if(i==10)
                msg.data.push_back(goalSpeeds[record]);
            else
                msg.data.push_back(0);
        }


    	pubGoalPos.publish(msg);
    	spinOnce();

        cout<<"\tTime: "<<current_time<<"\tgoalSpeeds: "<<goalSpeeds[record]<<"\tcurrent position: "<<current_pos<<endl;
    	current_pos += goalSpeeds[record] * DYNAMIXEL_MAX_VEL * 0.02;
		current_time += 0.02;

        record++;
    	loop_rate.sleep();

		if(record > num_velocities)
			return 0;

    }//From while ok()
}//From int main()




float speeds_record(){
    for(int i=0 ; i <= num_velocities ; i++)
            goalSpeeds[i] = (OBJETIVE_ANGLE * (30*pow(0.02*i, 2)/pow(total_time,3)  -  
                60*pow(0.02*i,3)/pow(total_time,4)  +  30*pow(0.02*i,4)/pow(total_time,5)))/DYNAMIXEL_MAX_VEL;  
}