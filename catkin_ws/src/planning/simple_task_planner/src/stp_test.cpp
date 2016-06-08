#include <iostream>
#include <string>
#include "ros/ros.h"
#include "simple_task_planner/taskadvertiser.h"
#include "simple_task_planner/simpletasks.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_task_planner");
    ros::NodeHandle nodeHandler("simple_task_planner");

    TaskAdvertiser ta(nodeHandler);

    ros::spin();
}
