#include<ros/ros.h>
#include<std_msgs/Float32MultiArray.h>
#include<vector>

#define OBJETIVE_ANGLE    1.7    // 1.7 rad
#define CONSTANT_VELOCITY 0.13   // 0.613  rad/s
#define DYNAMIXEL_MAX_VEL 12.259 // 12.259 rad/s  

using namespace std;
using namespace ros;

float goalPos[7] = {0, 0, 0, 4, 0, 0, 0};

int record = 0;

float constant_velocity = CONSTANT_VELOCITY * DYNAMIXEL_MAX_VEL; 
float total_time = abs(OBJETIVE_ANGLE/constant_velocity);


int main(int argc, char **argv){
    cout<<"Initializing reach_positions node..."<<endl;
    init(argc, argv, "send_positions");
    NodeHandle node;
  

    Publisher pubGoalPos = node.advertise<std_msgs::Float32MultiArray>("/x", 1000);




    std_msgs::Float32MultiArray msg;
    msg.data.clear();
    Rate loop_rate(50);


    while(ok()){
        
        msg.data.clear();
        
        //Setting positions for dynamixels            
        for(int i=0; i<7; i++){
                msg.data.push_back(goalPos[i]);          
        }//*/
 

        pubGoalPos.publish(msg);
        spinOnce();

        loop_rate.sleep();

    }//From while ok()
}//From int main()