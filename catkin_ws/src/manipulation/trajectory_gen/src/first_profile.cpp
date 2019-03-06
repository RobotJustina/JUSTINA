#include<ros/ros.h>
#include<hardware_tools/DynamixelManager.hpp>
#include<vector>

#define GRADES_PER_BITS 0.087912088
#define BITS_PER_GRADES 11.375        
#define ZERO_SHOULDER 1543

using namespace std;
using namespace ros;

string port;
int baudRate;
bool bulkEnable = true, syncWriteEnable = false;
uint16_t goalPos[7] = {1543, 1694, 1742, 2100, 2048, 2066, 1050};
int goalSpeeds[7] = {};
uint16_t curr_position[7] = {};

void parameters();
void showDates();

int main(int argc, char **argv)
{
    cout<<"Initializing first_profile node..."<<endl;
    init(argc, argv, "first_profile");
    NodeHandle node;

    parameters();
    showDates();

    vector<int> ids;
    for(int i = 0; i<7; i++){
        ids.push_back(i);
    }

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

    //Reading datas from the dynamixels
    dynamixelManager.readBulkData();
    for(int i = 0; i < 9; i++)
        dynamixelManager.getPresentPosition(i, curr_position[i]);


    Duration(1).sleep();	
    //Setting a constant profile
/*    dynamixelManager.setMovingSpeed(0,20);
    dynamixelManager.setGoalPosition(0, ZERO_SHOULDER - 228);//Moving just 20 grades
    if(syncWriteEnable){
        dynamixelManager.writeSyncGoalPosesData();
        dynamixelManager.writeSyncSpeedsData();
    }//*/
    //Setting triangular profile
    for(int t=0; t <500; t++){    //Part one
        cout<<"Time in: "<<t<<endl;
        dynamixelManager.setMovingSpeed(0,80*t/1000);
        dynamixelManager.setGoalPosition(0, ZERO_SHOULDER + 114);
        if(syncWriteEnable){
            dynamixelManager.writeSyncGoalPosesData();
            dynamixelManager.writeSyncSpeedsData();
        }
        Duration(0.001).sleep();
    }//*/
/*    for(int t=500; t <1000; t++){   //Part two
        dynamixelManager.setMovingSpeed(0,-80*t/1000 + 80);
        dynamixelManager.setGoalPosition(0, ZERO_SHOULDER - 228);
        if(syncWriteEnable){
            dynamixelManager.writeSyncGoalPosesData();
            dynamixelManager.writeSyncSpeedsData();
        }
        Duration(0.001).sleep();
    }//*/        
    dynamixelManager.close();
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

void showDates(){
    cout<<"\n"<<endl;
    cout<<"Baud Rate:  "<<baudRate<<endl;
    cout<<"The port is:  "<<port<<endl;
}//From the function show information
