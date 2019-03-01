#include<ros/ros.h>
#include<hardware_tools/DynamixelManager.hpp>
#include<vector>

using namespace std;
using namespace ros;

string port;
int baudRate;
bool bulkEnable = true, syncWriteEnable = true;
uint16_t goalPos[7] = {1543, 1600, 1800, 2100, 2048, 1800, 1050};
int goalSpeeds[7] = {};
uint16_t curr_position[7] = {};

void parameters();
void showDates();

int main(int argc, char **argv)
{
	cout<<"Initializing first_trajectory node..."<<endl;
	init(argc, argv, "first_trajectory");
	NodeHandle node;

	parameters();
	showDates();

	vector<int> ids;
	for(int i; i<9; i++){
		ids.push_back(i);
	}
	for(int i=0; i<7; i++)
		goalSpeeds[i] = 40;
    
    DynamixelManager dynamixelManager;
    dynamixelManager.init(port, baudRate, bulkEnable, ids, syncWriteEnable);

    for(int i=0; i < 7; i++)
        std::cout << "Pos" << goalPos[i] << std::endl;
    std::cout << "Setting all goal positions" << std::endl;
    //Setting the zero position of left arm
	for (int i = 0; i < 7; i++){
        dynamixelManager.setGoalPosition(i, goalPos[i]);		
        dynamixelManager.setMovingSpeed(i, 30);
        std::cout << "index: " << i << std::endl;
	}
    std::cout << "Sending write command..." << std::endl;
	if(syncWriteEnable){
		dynamixelManager.writeSyncGoalPosesData();	
        dynamixelManager.writeSyncSpeedsData();
	}

	Duration d = Duration(1);
	d.sleep();	
	//Setting the new position of left shoulder
	dynamixelManager.setGoalPosition(0, 1300);
	dynamixelManager.setMovingSpeed(0,30);
	if(syncWriteEnable){
		dynamixelManager.writeSyncGoalPosesData();
		dynamixelManager.writeSyncSpeedsData();
	}
	//Setting the new position of left elbow
/*	dynamixelManager.setGoalPosition(3, 2500);
	dynamixelManager.setMovingSpeed(3, 50)
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

void showDates(){
	cout<<"\n"<<endl;
	cout<<"Baud Rate:  "<<baudRate<<endl;
	cout<<"The port is:  "<<port<<endl;
}//From the function show information
