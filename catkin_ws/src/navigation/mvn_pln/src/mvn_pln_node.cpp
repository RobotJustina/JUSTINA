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
    bool clean_unexplored_map = false;
    bool look_at_goal = false;
    bool avoidance_type_obstacle = false;
    int value;
    int max_attempts = 7;
    float kinect_minX = 0.25;
    float kinect_maxX = 0.9;
    float kinect_minY = -0.35;
    float kinect_maxY = 0.35;
    float kinect_minZ = 0.05;
    float kinect_maxZ = 1.0;

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
    if(ros::param::has("~clean_unexplored_map"))
        ros::param::get("~clean_unexplored_map", clean_unexplored_map);
    if(ros::param::has("~look_at_goal"))
        ros::param::get("~look_at_goal", look_at_goal);
    if(ros::param::has("~kinect_minX"))
        ros::param::get("~kinect_minX", kinect_minX);
    if(ros::param::has("~kinect_maxX"))
        ros::param::get("~kinect_maxX", kinect_maxX);
    if(ros::param::has("~kinect_minY"))
        ros::param::get("~kinect_minY", kinect_minY);
    if(ros::param::has("~kinect_maxY"))
        ros::param::get("~kinect_maxY", kinect_maxY);
    if(ros::param::has("~kinect_minZ"))
        ros::param::get("~kinect_minZ", kinect_minZ);
    if(ros::param::has("~kinect_maxZ"))
        ros::param::get("~kinect_maxZ", kinect_maxZ);
    if(ros::param::has("~avoidance_type_obstacle"))
        ros::param::get("~avoidance_type_obstacle", avoidance_type_obstacle);

    JustinaNavigation::setNodeHandle(&n);
    JustinaManip::setNodeHandle(&n);
    JustinaKnowledge::setNodeHandle(&n);
    JustinaVision::setNodeHandle(&n);
    JustinaHRI::setNodeHandle(&n);
    //JustinaIROS::setNodeHandle(&n);
    MvnPln mvnPln;
    mvnPln.allow_move_lateral(allow_move_lateral);
    mvnPln.clean_goal_map(clean_goal_map);
    mvnPln.clean_unexplored_map(clean_unexplored_map);
    mvnPln.look_at_goal(look_at_goal);
    mvnPln.avoidance_type_obstacle(avoidance_type_obstacle);
    mvnPln.initROSConnection(&n);
    mvnPln.max_attempts = max_attempts;
    mvnPln.kinect_minX = kinect_minX;
    mvnPln.kinect_maxX = kinect_maxX;
    mvnPln.kinect_minY = kinect_minY;
    mvnPln.kinect_maxY = kinect_maxY;
    mvnPln.kinect_minZ = kinect_minZ;
    mvnPln.kinect_maxZ = kinect_maxZ;
    mvnPln.spin();

    return 0;
}
