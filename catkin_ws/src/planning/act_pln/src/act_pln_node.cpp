#include <iostream>
#include "ros/ros.h"
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaNavigation.h"

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING ACT_PLN BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "act_pln");
    ros::NodeHandle n;
    JustinaHardware::setNodeHandle(&n);
    JustinaNavigation::setNodeHandle(&n);
    ros::Rate loop(10);
    int counter = 0;
    std::vector<std::vector<float> > position;
    std::vector<float> p;
    for(int i=0; i< 8; i++)
    {
        position.push_back(p);
        for(int j=0; j<7; j++)
            position[i].push_back(0);
    }
    
    position[0][0] = -0.2;
    position[0][3] = 0.4;
    
    position[1][0] = 1;
    position[1][3] = 1.5;
    position[1][5] = 0.5;

    position[2][0] = 1;
    position[2][3] = 1.5;
    position[2][5] = 1.2;
    position[2][2] = 0.3;

    position[3][0] = 1;
    position[3][3] = 1.5;
    position[3][5] = 1.2;
    position[3][2] = -0.8;

    position[4][0] = 1;
    position[4][3] = 1.5;
    position[4][5] = 1.2;
    position[4][2] = 0.8;
    position[4][6] = 1;

    position[5][0] = 1;
    position[5][3] = 1.5;
    position[5][5] = 1.2;
    position[5][2] = 0.8;
    position[5][6] = -1;
    

    position[6][0] = 1;
    position[6][3] = 1.5;
    position[6][5] = 1.2;
    position[6][2] = -0.8;
    position[6][6] = 0;

    position[7][0] = 0;
    position[7][3] = 0;
    position[7][5] = 0;
    position[7][2] = 0;
    position[7][6] = 0;
    
    int idx = 0;
    while(ros::ok())
    {
        if(++counter % 20 == 0)
        {
            
            JustinaHardware::setLeftArmGoalPose(position[idx]);
            JustinaHardware::setRightArmGoalPose(position[idx]);
            JustinaHardware::setHeadGoalPose(1,1);
            idx++;
            if(idx == position.size())
                idx = 0;
        }
        ros::spinOnce();
        loop.sleep();
    }
}
