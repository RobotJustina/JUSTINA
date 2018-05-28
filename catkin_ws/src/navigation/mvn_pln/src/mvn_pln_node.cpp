#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "justina_tools/JustinaNavigation.h"
#include "MvnPln.h"

int main(int argc, char** argv)
{
    std::string locationsFilePath = "";
    bool allow_move_lateral = false;
    bool clean_goal_map = false;
    int value;
    int max_attempts = 7;
    // TODO REMOVE THIS DEPRECTAED PARAMS CONFIG
    /*
    for(int i=0; i < argc; i++)
    {
        std::string strParam(argv[i]);
        if(strParam.compare("-f") == 0)
            locationsFilePath = argv[++i];
        if(strParam.compare("--move_lateral") == 0)
            allow_move_lateral = true;
        if(strParam.compare("--max_attempts") == 0)
        {
            std::stringstream ss(argv[++i]);
            if(ss >> value)
                max_attempts = value;
        }
    }*/

    std::cout << "INITIALIZING MOVING PLANNER BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "mvn_pln");
    ros::NodeHandle n;
    ros::Rate loop(10);

    if(ros::param::has("~move_lateral"))
        ros::param::get("~move_lateral", allow_move_lateral);
    if(ros::param::has("~max_attempts"))
        ros::param::get("~max_attempts", max_attempts);
    if(ros::param::has("~clean_goal_map"))
        ros::param::get("~clean_goal_map", clean_goal_map);

    JustinaNavigation::setNodeHandle(&n);
    JustinaManip::setNodeHandle(&n);
    JustinaKnowledge::setNodeHandle(&n);
    MvnPln mvnPln;
    mvnPln.allow_move_lateral(allow_move_lateral);
    mvnPln.clean_goal_map(clean_goal_map);
    mvnPln.initROSConnection(&n);
    mvnPln.max_attempts = max_attempts;
    mvnPln.spin();

    return 0;
}
