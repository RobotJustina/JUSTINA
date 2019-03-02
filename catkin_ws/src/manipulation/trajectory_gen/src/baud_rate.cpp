#include<ros/ros.h>
#include<hardware_tools/DynamixelManager.hpp>
#include<vector>

using namespace std;
using namespace ros;

string port;
int baudRate, times;
bool bulkEnable = true, syncWriteEnable = true;
uint16_t goalPos[7] = {1543, 1600, 1800, 2100, 2048, 1800, 1050};
uint16_t curr_position[7] = {};

void parameters();
void showDates();

int main(int argc, char **argv){
	
	cout<<"Initializing baud_rate node..."<<endl;
	init(argc,argv,"baud_rate");

    parameters();    
    showDates();

    vector<int> ids;
    for(int i = 0; i < 9; i++)
        ids.push_back(i);    


    DynamixelManager dynamixelManager;
    dynamixelManager.init(port, baudRate, bulkEnable, ids, syncWriteEnable);
	
	cout << "Getting time of comunication..." <<endl;
	
	for(int j = 0; j < times; j++){
		Time ti = Time::now();
		for (int i = 0; i < 7; i++)
			dynamixelManager.setGoalPosition(i, goalPos[i]);		
		if(syncWriteEnable)
			dynamixelManager.writeSyncGoalPosesData();	

		if(bulkEnable)
			dynamixelManager.readBulkData();
		for(int i=0;i<7;i++)
			dynamixelManager.getPresentPosition(i, curr_position[i]);
		Time tf = Time::now();
		cout<<"Time: "<<ti-tf<<endl;

    }//*/From the number of times to send messages

	return 0;
    
}//From the main function

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
    if(!node.hasParam("times"))
    	cout<<"missing times parameter :("<<endl;    	
    if(!node.getParam("times",times))
    	cout<<"Invalid times parameter :("<<endl;
}//From parameters function

void showDates(){
	cout<<"\n"<<endl;
	cout<<"Baud Rate:  "<<baudRate<<endl;
	cout<<"The port is:  "<<port<<endl;
	cout<<"Number of times required: "<<times<<endl;
	cout<<"\n"<<endl;
	for(int i=0;i<7;i++){
		cout<<"goalPos["<<i<<"]"<<goalPos[i]<<endl;
	}
	cout<<"\n"<<endl;//*/	
    Duration d = Duration(0.00564);
    d.sleep();	
}//From the function show information

//Para que sirven bulk y sync
//Como mandar 1000 datos 
//bulk true es una forma de leer todos los dynamixelManager
