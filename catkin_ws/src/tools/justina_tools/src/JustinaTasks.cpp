#include "justina_tools/JustinaTasks.h"

bool JustinaTasks::is_node_set = false;

bool JustinaTasks::setNodeHandle(ros::NodeHandle* nh)
{
    if(JustinaTasks::is_node_set)
        return true;
    if(nh == 0)
        return false;

    std::cout << "JustinaTasks.->Setting ros node..." << std::endl;
    JustinaTasks::is_node_set = true;
    return true;
}

bool JustinaTasks::alignWithTable()
{
}

bool JustinaTasks::moveToGraspingPosition(float objectX, float objectY, float objectZ, bool withLeftArm)
{
}
