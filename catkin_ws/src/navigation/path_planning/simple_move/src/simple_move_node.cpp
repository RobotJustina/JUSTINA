#include <iostream>
#include "SimpleMoveNode.h"

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING SIMPLE MOVE NODE BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "simple_move");

    SimpleMoveNode node;
    node.initROSConnection();
    node.spin();
}
