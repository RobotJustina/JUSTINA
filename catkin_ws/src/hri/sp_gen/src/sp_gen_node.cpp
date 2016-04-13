#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sound_play/sound_play.h"

void callbackSay(const std_msgs::String::ConstPtr& msg)
{
    std::cout << "SpGen.->String to say: \"" << msg->data << "\"" << std::endl;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING SP_GEN BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "sp_gen");
    ros::NodeHandle n;
    ros::Subscriber subSay = n.subscribe("sp_gen/say", 1, callbackSay);
    ros::Rate loop(10);
    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }
}
