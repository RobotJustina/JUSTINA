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
float goalSpeeds = 0;
float maximum_velocity = 0;

float constant_velocity = CONSTANT_VELOCITY * DYNAMIXEL_MAX_VEL; 
float total_time = OBJETIVE_ANGLE/constant_velocity;



int main(int argc, char **argv){
	cout<<"Initializing elbow_profile node..."<<endl;
	init(argc, argv, "elbow_constant_profile");
	NodeHandle node;
  
    Publisher pubGoalPos = node.advertise<std_msgs::Float32MultiArray>("/hardware/left_arm/goal_pose", 1000);

    Rate loop_rate(50);

    while(ok()){
    	
    	std_msgs::Float32MultiArray msg;
    	msg.data.clear();


    	//Setting positions for dynamixels
    	for(int i=0; i<7; i++){
	    	if(i==3)
		    	msg.data.push_back(OBJETIVE_ANGLE);
		    else
		    	msg.data.push_back(0);    		
    	}


    	//----------------------------Constant Profile-------------------------
    	goalSpeeds = CONSTANT_VELOCITY;


		//Seting velocities for dynamixels
    	for(int i=7; i<14; i++){
    		if(i==10)
    			msg.data.push_back(goalSpeeds);
    		else
    			msg.data.push_back(0);
    	}



    	pubGoalPos.publish(msg);
    	spinOnce();

    	current_pos += goalSpeeds * DYNAMIXEL_MAX_VEL * 0.02;
		current_time += 0.02;


	    cout<<"\tTime: "<<current_time<<"\tgoalSpeeds: "<<goalSpeeds<<"\tcurrent position: "<<current_pos<<endl;

    	loop_rate.sleep();
		if(current_time >= total_time)
			return 0;

    }//From while ok()
}//From int main()
