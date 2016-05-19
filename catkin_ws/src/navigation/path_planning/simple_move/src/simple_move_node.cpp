#include <iostream>
#include "SimpleMoveNode.h"

int main(int argc, char** argv)
{
    bool moveHead = false;
    for(int i=0; i < argc; i++)
    {
        std::string strParam(argv[i]);
        if(strParam.compare("--move_head") == 0)
            moveHead = true;
    }
    std::cout << "INITIALIZING SIMPLE MOVE NODE BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "simple_move");

    SimpleMoveNode node;
    node.moveHead = moveHead;
    node.initROSConnection();
    node.spin();
}
