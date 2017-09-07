#include <ros/ros.h>
#include <openpose/OpenPose.hpp>

int main(int argc, char ** argv){
    
    ros::init(argc, argv, "open_pose_node");
    ros::NodeHandle nh;
    ros::Rate rate(30);

    OpenPose op;

    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();
    }

    return 1;

}

