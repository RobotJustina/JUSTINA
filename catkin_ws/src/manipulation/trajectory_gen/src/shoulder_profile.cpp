#include<ros/ros.h>
#include<std_msgs/Float32MultiArray.h>
#include<vector>

#define OBJETIVE_ANGLE    0.6    // 1.7 rad
#define CONSTANT_VELOCITY 0.05 // 0.613  rad/s
#define DYNAMIXEL_MAX_VEL 12.259 // 12.259 rad/s  


using namespace std;
using namespace ros;

float current_pos = 0;
float current_time = 0;
float goalSpeeds = 0;
float maximum_velocity = 0;

float constant_velocity = CONSTANT_VELOCITY * DYNAMIXEL_MAX_VEL; 
float total_time = OBJETIVE_ANGLE/constant_velocity;



int main(int argc, char **argv)
{
	cout<<"Initializing shoulder_profile node..."<<endl;
	init(argc, argv, "shoulder_profile");
	NodeHandle node;
  
    Publisher pubGoalPos = node.advertise<std_msgs::Float32MultiArray>("/hardware/left_arm/goal_pose", 1000);

    Rate loop_rate(50);

    while(ok()){
    	
    	std_msgs::Float32MultiArray msg;
    	msg.data.clear();


    	//Setting positions for dynamixels
    	for(int i=0; i<7; i++){
	    	if(i==0)
		    	msg.data.push_back(OBJETIVE_ANGLE);
		    else
		    	msg.data.push_back(0);    		
    	}


    	//----------------------------Constant Profile-------------------------
//    	goalSpeeds = CONSTANT_VELOCITY;

		//---------------------------Triangular Profile------------------------		
		maximum_velocity = 2 * OBJETIVE_ANGLE / total_time;

		if(current_time < total_time/2)
			goalSpeeds = (2 * maximum_velocity * current_time / total_time) / DYNAMIXEL_MAX_VEL;
		else
			goalSpeeds = (-(2 * maximum_velocity * current_time) / total_time + 2 * maximum_velocity) / DYNAMIXEL_MAX_VEL;//*/



		//Seting velocities for dynamixels
    	for(int i=7; i<14; i++){
    		if(i==7)
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
