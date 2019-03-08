#include<ros/ros.h>
#include<std_msgs/Float32MultiArray.h>
#include<vector>

#define OBJETIVE_ANGLE		 1.7  // 1.7 rad
#define CONSTANT_OF_VELOCITY 0.613 // 5.3564 rad/s  0.05
#define MAXIMUN_VELOCITY     1.2260  // 1.2260 rad/s 0.10     

using namespace std;
using namespace ros;

int main(int argc, char **argv)
{
	cout<<"Initializing trajectory_pub node..."<<endl;
	init(argc, argv, "trajectory_pub");
	NodeHandle node;
  
    ros::Publisher pubGoalPos = node.advertise<std_msgs::Float32MultiArray>("/hardware/left_arm/goal_pose", 1000);

    ros::Rate loop_rate(50);

    float current_pos,goalSpeeds, t, tf = OBJETIVE_ANGLE/CONSTANT_OF_VELOCITY; 

    while(ok()){
    	
    	std_msgs::Float32MultiArray msg;
    	msg.data.clear();

    	for(int i=0; i<7; i++){//--------------Set the position---------------------------
	    		if(i==3)
		    		msg.data.push_back(OBJETIVE_ANGLE);
		    	else
		    		msg.data.push_back(0);    		
	    
    	}
/*    	for(int i=7; i<14; i++){
    		if(i==10)
    			msg.data.push_back(CONSTANT_OF_VELOCITY);
    		else
    			msg.data.push_back(0);
    	}//*/ //Constant profile

        
        if(t < tf/2){//-----------------------Triangular profile-----------------------------
	    	goalSpeeds = (2 * MAXIMUN_VELOCITY * t / tf) / 12.226	;//Falta convertir
	    	for(int i=7; i<14; i++){
	    		if(i==10)
	    			msg.data.push_back(goalSpeeds);
	    		else
	    			msg.data.push_back(0);
	    	}
	    }
	    else{
	    	goalSpeeds = (-(2 * MAXIMUN_VELOCITY * t / tf) + 2*MAXIMUN_VELOCITY) / 12.226;
	    	for(int i=7; i<14; i++){
	    		if(i==10)
	    			msg.data.push_back(goalSpeeds);
	    		else
	    			msg.data.push_back(0);
	    		}	    	
	    }//*/ //Triangular profile

    	pubGoalPos.publish(msg);
    	spinOnce();
    	current_pos += goalSpeeds * t  * 0.10472 * OBJETIVE_ANGLE;
		t += 0.02;

    	loop_rate.sleep();

	    cout<<"\tTime: "<<t<<"\tgoalSpeeds: "<<goalSpeeds<<"\tcurrent position: "<<current_pos<<endl;

		if(t >= tf)
			return 0;

    }//From while ok()
}//From int main()
