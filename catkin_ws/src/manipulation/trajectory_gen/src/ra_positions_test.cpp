#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

using namespace std;

int main(int argc, char **argv)
{
    cout<<"Starting ra_positions_test node by Luis Nava..."<<endl;
    ros::init(argc, argv, "ra_positions_test");
    ros::NodeHandle n;

    ros::Publisher pubPoses = n.advertise<std_msgs::Float32MultiArray>("/manipulation/manip_pln/ra_goto_angles", 1000);
    std_msgs::Float32MultiArray poses_msg;
    ros::Rate loop(0.5);

    while(ros::ok())
    {
    	poses_msg.data.resize(7);

    	//home 0.0    0.0    0.0    0.0     0.0    0.0  1.5707
    	poses_msg.data[0] = 0.0;
		poses_msg.data[1] = 0.0;
    	poses_msg.data[2] = 0.0;
		poses_msg.data[3] = 0.0;
    	poses_msg.data[4] = 0.0;
		poses_msg.data[5] = 0.0;
    	poses_msg.data[6] = 1.5707;
   		pubPoses.publish(poses_msg);
    	loop.sleep();
    	ros::spinOnce();


        //navigation -1.40    0.0    0.0    2.07    0.0    0.9   0.0
    	poses_msg.data[0] = -1.4;
		poses_msg.data[1] =  0.0;
    	poses_msg.data[2] =  0.0;
		poses_msg.data[3] = 2.07;
    	poses_msg.data[4] =  0.0;
		poses_msg.data[5] =  0.9;
    	poses_msg.data[6] =  0.0;
   		pubPoses.publish(poses_msg);
    	loop.sleep();
    	ros::spinOnce();

        //take 0.3    0.3   -0.4    1.47   0.4     0.3   -0.2
    	poses_msg.data[0] =  0.3;
		poses_msg.data[1] =  0.3;
    	poses_msg.data[2] = -0.4;
		poses_msg.data[3] = 1.47;
    	poses_msg.data[4] =  0.4;
		poses_msg.data[5] =  0.3;
    	poses_msg.data[6] = -0.2;
   		pubPoses.publish(poses_msg);
    	loop.sleep();
    	ros::spinOnce();

    	//navigation -1.40    0.0    0.0    2.07    0.0    0.9   0.0
    	poses_msg.data[0] = -1.4;
		poses_msg.data[1] =  0.0;
    	poses_msg.data[2] =  0.0;
		poses_msg.data[3] = 2.07;
    	poses_msg.data[4] =  0.0;
		poses_msg.data[5] =  0.9;
    	poses_msg.data[6] =  0.0;
   		pubPoses.publish(poses_msg);
    	loop.sleep();
    	ros::spinOnce();     
    }
    
    return 0;
}
