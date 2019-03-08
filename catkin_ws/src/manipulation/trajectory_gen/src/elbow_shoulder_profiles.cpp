#include<ros/ros.h>
#include<std_msgs/Float32MultiArray.h>
#include<vector>


#define ELBOW_ID 		  3
#define ELBOW_ID_VEL	 10
#define SHOULDER_ID		  0	
#define SHOULDER_ID_VEL   7
#define DYNAMIXEL_MAX_VEL 12.259 // 12.259 rad/s  

#define ELBOW_OBJETIVE_ANGLE     1.7    // 1.7 rad
#define ELBOW_CONSTANT_VELOCITY  0.13   // 0.613  rad/s

#define FLAG_FOR_SHOULDER            0.5    //It indicates when shoulder starts to move
#define SHOULDER_OBJETIVE_ANGLE      -0.6
#define SHOULDER_CONSTANT_VELOCITY   0.008

using namespace std;
using namespace ros;

float elbow_current_pos = 0;
float elbow_goalSpeeds = 0;
float elbow_maximum_velocity = 0;

float elbow_constant_velocity = ELBOW_CONSTANT_VELOCITY * DYNAMIXEL_MAX_VEL; 
float elbow_total_time = ELBOW_OBJETIVE_ANGLE/elbow_constant_velocity;
float flag = ELBOW_OBJETIVE_ANGLE * FLAG_FOR_SHOULDER;


float shoulder_current_pos = 0;
float shoulder_goalSpeeds = 0;
float shoulder_maximum_velocity = 0;

float shoulder_constant_velocity = SHOULDER_CONSTANT_VELOCITY * DYNAMIXEL_MAX_VEL; 
float shoulder_total_time = SHOULDER_OBJETIVE_ANGLE/shoulder_constant_velocity;


float current_time = 0;

int main(int argc, char **argv){

	cout<<"Initializing elbow_shoulder_profiles node..."<<endl;
	init(argc, argv, "elbow_shoulder_profiles");
	NodeHandle node;

    Publisher pubGoalPos = node.advertise<std_msgs::Float32MultiArray>("/hardware/left_arm/goal_pose", 1000);

    Rate loop_rate(50);

    while(ok()){
    	
    	std_msgs::Float32MultiArray msg;
    	msg.data.clear();

    	//Setting positions for dynamixels
    	for(int i=0; i<7; i++){
	    	if(i == ELBOW_ID)
		    	msg.data.push_back(ELBOW_OBJETIVE_ANGLE);
		    if(i == SHOULDER_ID)
		    	msg.data.push_back(SHOULDER_OBJETIVE_ANGLE);
		    else
		    	msg.data.push_back(0);    		
    	}
    	
		//Setting velocities for dynamixels
    	for(int i=7; i<14; i++){
    		if(i == ELBOW_ID_VEL)
    			msg.data.push_back(elbow_goalSpeeds);
    		if(i == SHOULDER_ID_VEL && elbow_current_pos > flag)
    			msg.data.push_back(shoulder_goalSpeeds);
    		else
    			msg.data.push_back(0);
    	}

    	//----------------------------Constant Profile-------------------------
//    	elbow_goalSpeeds = ELBOW_CONSTANT_VELOCITY;
//    	shoulder_goalSpeeds = SHOULDER_CONSTANT_VELOCITY;

		//---------------------------Triangular Profile------------------------		
		elbow_maximum_velocity = 2 * ELBOW_OBJETIVE_ANGLE / elbow_total_time;
		shoulder_maximum_velocity = 2 * SHOULDER_OBJETIVE_ANGLE / shoulder_total_time;

		if(current_time < elbow_total_time/2)
			elbow_goalSpeeds = (2 * elbow_maximum_velocity * current_time / elbow_total_time) / DYNAMIXEL_MAX_VEL;
		else
			elbow_goalSpeeds = (-(2 * elbow_maximum_velocity * current_time) / elbow_total_time + 2 * elbow_maximum_velocity) / DYNAMIXEL_MAX_VEL;//*/

		if(current_time < shoulder_total_time / 2)
			shoulder_goalSpeeds = (2 * shoulder_maximum_velocity * current_time / shoulder_total_time) / DYNAMIXEL_MAX_VEL;
		else
			shoulder_goalSpeeds = (-(2 * shoulder_maximum_velocity * current_time) / shoulder_total_time + 2 * shoulder_maximum_velocity) / DYNAMIXEL_MAX_VEL;//*/


    	elbow_current_pos += elbow_goalSpeeds * DYNAMIXEL_MAX_VEL * 0.02;

	    cout<<"Time: "<<current_time<<"\tgoalSpeeds: "<<elbow_goalSpeeds<<"\tcurrent position: "<<elbow_current_pos<<endl;


    	pubGoalPos.publish(msg);
    	spinOnce();

		current_time += 0.02;
    	loop_rate.sleep();

		if(current_time >= elbow_total_time)
			return 0;

    }//From while ok()
}