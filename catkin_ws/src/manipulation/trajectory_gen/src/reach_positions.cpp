#include<ros/ros.h>
#include<std_msgs/Float32MultiArray.h>
#include<vector>

#define OBJETIVE_POSE    1.7    // 1.7 rad
#define CONSTANT_VELOCITY 0.13   // 13% of maximum velocity 
#define DYNAMIXEL_MAX_VEL 12.259 // 12.259 rad/s  

using namespace std;
using namespace ros;

int record = 0;
float current_time = 0;
float goalPos[7] = {0, 0, 0, 0, 0, 0, 0};


float constant_velocity = CONSTANT_VELOCITY * DYNAMIXEL_MAX_VEL; 
float total_time = abs(OBJETIVE_POSE/constant_velocity);

int num_velocities = total_time / 0.02;
float* goalSpeeds = new float[num_velocities];
float speeds_record();


void posesCallback(const std_msgs::Float32MultiArray::ConstPtr &msg){
        cout<<"Reciving new goal poses..."<<endl;
        //ROS_INFO("I heard: [%f]", msg->data[3]);
        if(!(msg->data.size() == 7 || msg->data.size() == 14))
            cout << "Can not process the goal poses for the left arm" << endl;
        else{
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
  
    Subscriber getGoalPos = node.subscribe("/x", 1000, posesCallback);
    Publisher pubGoalPos = node.advertise<std_msgs::Float32MultiArray>("/hardware/left_arm/goal_pose", 1000);




    std_msgs::Float32MultiArray msg;
    Rate loop_rate(50);

    speeds_record();

    while(ok()){
    	
        msg.data.clear();
        //Setting positions for dynamixels
        for(int i=0; i<7; i++){
            msg.data.push_back(goalPos[i]);          
        }
        //Seting velocities for dynamixels
        for(int i=7; i<14; i++){
            msg.data.push_back(goalSpeeds[record]);
        }
        cout<<"Time: "<<current_time<<"\tPosition: "<<goalPos[3]<<"\tvelocity: "<<goalSpeeds[record]<<endl;

        current_time += 0.02;
        record++;

    	pubGoalPos.publish(msg);
    	spinOnce();
        loop_rate.sleep();
        if(current_time > total_time)
            return 0;
    }//*/
}//From int main()



float speeds_record(){
    for(int i=0 ; i <= num_velocities ; i++)
        goalSpeeds[i] = (OBJETIVE_POSE * (30*pow(0.02*i, 2)/pow(total_time,3)  -  60*pow(0.02*i,3)/pow(total_time,4)  +  30*pow(0.02*i,4)/pow(total_time,5)))/DYNAMIXEL_MAX_VEL;//*/
}