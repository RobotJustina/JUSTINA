#include<ros/ros.h>
#include<std_msgs/Float32MultiArray.h>
#include<vector>


using namespace std;
using namespace ros;

float goalPos[7] = {1.7, 0.0, 0.0, 1, 0.0, 0.0, 0.0}; //0.5 0.08

int record = 0;


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