#include<ros/ros.h>
#include<hardware_tools/DynamixelManager.hpp>
#include<vector>

using namespace std;
using namespace ros;

string port;
int baudRate;
bool bulkEnable = true, syncWriteEnable = false;
uint16_t goalPos[7] = {1543, 1694, 1742, 2100, 2048, 2066, 1050};
int goalSpeeds[7] = {};
uint16_t curr_position[7] = {};

void parameters();
void showDatas();

int main(int argc, char **argv)
{
    cout<<"Initializing first_trajectory node..."<<endl;
    init(argc, argv, "first_trajectory");
    NodeHandle node;

    parameters();
    showDatas();

    vector<int> ids;
    for(int i = 0; i<7; i++){
        ids.push_back(i);
    }
    for(int i=0; i<7; i++)
        goalSpeeds[i] = 40;

    DynamixelManager dynamixelManager;
    dynamixelManager.enableInfoLevelDebug();
    dynamixelManager.init(port, baudRate, bulkEnable, ids, false);

    for(int i = 0; i < 7; i++){
        dynamixelManager.enableTorque(i);
        dynamixelManager.setPGain(i, 32);
        dynamixelManager.setIGain(i, 0);
        dynamixelManager.setDGain(i, 128);
        /*dynamixelManager.setPGain(i, 32);
          dynamixelManager.setIGain(i, 0);
          dynamixelManager.setDGain(i, 0);*/
        dynamixelManager.setMaxTorque(i, 1023);
        dynamixelManager.setTorqueLimit(i, 768);
        dynamixelManager.setHighestLimitTemperature(i, 80);
        dynamixelManager.setAlarmShutdown(i, 0b00000100);
    }

    for(int i=0; i < 7; i++)
        cout << "Pos" << goalPos[i] << endl;
    cout << "Setting all goal positions" << endl;
    //Setting the zero position of left arm
    dynamixelManager.readBulkData();
    for(int i = 0; i < 9; i++)
        dynamixelManager.getPresentPosition(i, curr_position[i]);
    for (int i = 0; i < 7; i++){
        cout << "index: " << i << endl;
        cout << "Pos" << goalPos[i] << endl;
        dynamixelManager.setMovingSpeed(i, 30);
        cout << "index: " << i << endl;
        cout << "Pos" << goalPos[i] << endl;
        dynamixelManager.setGoalPosition(i, goalPos[i]);		
    }
    std::cout << "Sending write command..." << std::endl;
    if(syncWriteEnable){
        dynamixelManager.writeSyncGoalPosesData();	
        dynamixelManager.writeSyncSpeedsData();
    }

    Duration(1).sleep();
    //Setting the new position of left shoulder
  dynamixelManager.setMovingSpeed(0,30);
    dynamixelManager.setGoalPosition(0, 1771);
    if(syncWriteEnable){
        dynamixelManager.writeSyncGoalPosesData();
        dynamixelManager.writeSyncSpeedsData();
    }//*/
    //Setting the new position of left elbow
/*   	dynamixelManager.setGoalPosition(3, 2328);
    dynamixelManager.setMovingSpeed(3, 20)
        if(syncWriteEnable){
  	        dynamixelManager.writeSyncGoalPosesData();
    	    dynamixelManager.writeSyncSpeedsData();
        }//*/
    return 0;
}

void parameters(){
    NodeHandle node("~");

    if(!node.hasParam("baud"))
        cout<<"missing baudRate"<<endl;    	
    if(!node.getParam("baud",baudRate ))
        cout<<"Invalid baudRate number"<<endl;
    if(!node.hasParam("port"))
        cout<<"missing port"<<endl;    	
    if(!node.getParam("port",port ))
        cout<<"Invalid port"<<endl;
}//From parameters function

void showDatas(){
    cout<<"\n"<<endl;
    cout<<"Baud Rate:  "<<baudRate<<endl;
    cout<<"The port is:  "<<port<<endl;
}//From the function show information
