#include<ros/ros.h>
#include<std_msgs/Float32MultiArray.h>
#include<vector>

#define ID_O        0
#define ID_1        1
#define ID_2        2
#define ID_3        3
#define ID_4        4
#define ID_5        5
#define ID_6        6

#define CONSTANT_VELOCITY 0.08   // 13% of maximum velocity 
#define DYNAMIXEL_MAX_VEL 12.259 // 12.259 rad/s  

using namespace std;
using namespace ros;

int record = 0;
bool connection = false;
float goalPos[7] = {0, 0, 0, 0, 0, 0, 0};
float constant_velocity = CONSTANT_VELOCITY * DYNAMIXEL_MAX_VEL; 

float time_for_id_[7]  = {0,0,0,0,0,0,0};


int num_vel_for_id_[7] = {0,0,0,0,0,0,0};


float goalSpeeds[7][10000];




void speeds_record();


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
            
                for(int i=0; i < 7 ; i++){
                    time_for_id_[i] = abs( goalPos[i]  /  constant_velocity);                
                    num_vel_for_id_[i] = time_for_id_[i] / 0.02;
                }
        }

}

int main(int argc, char **argv){
	cout<<"Initializing reach_positions node..."<<endl;
	init(argc, argv, "reach_positions");
	NodeHandle node;
  
    Subscriber getGoalPos = node.subscribe("/x", 1000, posesCallback);
    Publisher pubGoalPos = node.advertise<std_msgs::Float32MultiArray>("/hardware/left_arm/goal_pose", 1000);


    Rate loop_rate(50);
    std_msgs::Float32MultiArray msg;


    while(connection == false){
        cout<<"Waiting for goal poses..."<<endl;        
        spinOnce();

        loop_rate.sleep();
    }
    

    speeds_record();
    Duration(2).sleep();


    while(ok()){
        msg.data.clear();
        //Setting positions for dynamixels
        for(int i=0; i<7; i++)
            msg.data.push_back(goalPos[i]);          

        //Seting velocities for dynamixels
        for(int i=7; i<14; i++){
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
                msg.data.push_back(0);
        }

        record++;

    	pubGoalPos.publish(msg);

    	spinOnce();
        loop_rate.sleep();
/*        if(record > num_vel_for_id_[ID_O] && record > num_vel_for_id_[ID_1] && record > num_vel_for_id_[ID_2] &&
            record > num_vel_for_id_[ID_3] && record > num_vel_for_id_[ID_4] && record > num_vel_for_id_[ID_5] &&
            record > num_vel_for_id_[ID_6])
            return 0;//*/
    }
}//From int main()



void speeds_record(){
    for(int i=0 ; i <= num_vel_for_id_[ID_O] ; i++){
        goalSpeeds[ID_O][i] = abs(goalPos[ID_O] * (30*pow(0.02*i, 2)/pow(time_for_id_[ID_O],3)  -  60*pow(0.02*i,3)/pow(time_for_id_[ID_O],4) 
                                                         +  30*pow(0.02*i,4)/pow(time_for_id_[ID_O],5)))/DYNAMIXEL_MAX_VEL;//*/
        goalSpeeds[ID_1][i] = abs(goalPos[ID_1] * (30*pow(0.02*i, 2)/pow(time_for_id_[ID_1],3)  -  60*pow(0.02*i,3)/pow(time_for_id_[ID_1],4) 
                                                         +  30*pow(0.02*i,4)/pow(time_for_id_[ID_1],5)))/DYNAMIXEL_MAX_VEL;//*/
        goalSpeeds[ID_2][i] = abs(goalPos[ID_2] * (30*pow(0.02*i, 2)/pow(time_for_id_[ID_2],3)  -  60*pow(0.02*i,3)/pow(time_for_id_[ID_2],4) 
                                                         +  30*pow(0.02*i,4)/pow(time_for_id_[ID_2],5)))/DYNAMIXEL_MAX_VEL;//*/
        goalSpeeds[ID_3][i] = abs(goalPos[ID_3] * (30*pow(0.02*i, 2)/pow(time_for_id_[ID_3],3)  -  60*pow(0.02*i,3)/pow(time_for_id_[ID_3],4) 
                                                         +  30*pow(0.02*i,4)/pow(time_for_id_[ID_3],5)))/DYNAMIXEL_MAX_VEL;//*/
        goalSpeeds[ID_4][i] = abs(goalPos[ID_4] * (30*pow(0.02*i, 2)/pow(time_for_id_[ID_4],3)  -  60*pow(0.02*i,3)/pow(time_for_id_[ID_4],4) 
                                                         +  30*pow(0.02*i,4)/pow(time_for_id_[ID_4],5)))/DYNAMIXEL_MAX_VEL;//*/
        goalSpeeds[ID_5][i] = abs(goalPos[ID_5] * (30*pow(0.02*i, 2)/pow(time_for_id_[ID_5],3)  -  60*pow(0.02*i,3)/pow(time_for_id_[ID_5],4) 
                                                         +  30*pow(0.02*i,4)/pow(time_for_id_[ID_5],5)))/DYNAMIXEL_MAX_VEL;//*/ 
        goalSpeeds[ID_6][i] = abs(goalPos[ID_6] * (30*pow(0.02*i, 2)/pow(time_for_id_[ID_6],3)  -  60*pow(0.02*i,3)/pow(time_for_id_[ID_6],4) 
                                                         +  30*pow(0.02*i,4)/pow(time_for_id_[ID_6],5)))/DYNAMIXEL_MAX_VEL;//*/                                                                   
    }//*/
}