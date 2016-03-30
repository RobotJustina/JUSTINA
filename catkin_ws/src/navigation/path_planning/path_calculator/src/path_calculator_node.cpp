#include <iostream>
#include "ros/ros.h"
#include "navig_msgs/PathFromMap.h"
#include "tf/tf.h"

bool callbackWaveFront(navig_msgs::PathFromMap::Request &req, navig_msgs::PathFromMap::Response &resp)
{
    //It is assumed that origin pose has an orientation of (0,0,0)
    int width = req.map.info.width;
    int height = req.map.info.height;
    float cell_size = req.map.info.resolution;
    float origin_x = req.map.info.origin.position.x;
    float origin_y = req.map.info.origin.position.y;
    float start_x = req.start_pose.position.x;
    float start_y = req.start_pose.position.y;
    float goal_x = req.goal_pose.position.x;
    float goal_y = req.goal_pose.position.y;
    int start_cell_x = (int)(start_x - origin_x)/cell_size;
    int start_cell_y = (int)(start_y - origin_y)/cell_size;
    int goal_cell_x = (int)(goal_x - origin_x)/cell_size;
    int goal_cell_y = (int)(goal_y - origin_y)/cell_size;
    int start_cell = start_cell_y * width + start_cell_x;
    int goal_cell = goal_cell_y * width + goal_cell_x;
    std::cout << "Calculate path wave front. Start cell: " << start_cell << "  Goal cell: " << goal_cell << std::endl;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING PATH CALCULATOR BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "path_calculator");
    ros::NodeHandle n;
    ros::ServiceServer srvPathWaveFront = n.advertiseService("wave_front", callbackWaveFront);
    ros::Rate loop(10);

    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}

