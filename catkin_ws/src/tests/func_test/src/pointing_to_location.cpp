#include <iostream>
#include "justina_tools/JustinaKnowledge.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaManip.h"

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING A TEST JUST FOR TEST BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "act_pln");
    ros::NodeHandle n;
    JustinaKnowledge::setNodeHandle(&n);
    JustinaTools::setNodeHandle(&n);
    JustinaManip::setNodeHandle(&n);
    ros::Rate loop(10);

    int nextState = 1;
    bool fail = false;
    bool success = false;
    float gx_w, gy_w, gz_w, guest_z, host_z = 0.7;    
    float goalx, goaly, goala;
    float pointingArmX, pointingArmY, pointingArmZ;
    float pointingDirX, pointingDirY, pointingDirZ, pointingNormal;
    float distanceArm = 0.4;
    bool usePointArmLeft = false;

    while(ros::ok() && !fail && !success){
        switch(nextState){
        case 1:
            JustinaKnowledge::getKnownLocation("kitchen", goalx, goaly, goala);
            JustinaTools::transformPoint("/map", goalx, goaly , host_z, "/base_link", pointingArmX, pointingArmY, pointingArmZ);
            if(pointingArmY > 0){
                usePointArmLeft = true;
                JustinaTools::transformPoint("/map", goalx, goaly , host_z, "/left_arm_link0", pointingArmX, pointingArmY, pointingArmZ);
            }else{
                usePointArmLeft = false;
                JustinaTools::transformPoint("/map", goalx, goaly , host_z, "/right_arm_link0", pointingArmX, pointingArmY, pointingArmZ);
            }
            pointingNormal = sqrt(pointingArmX * pointingArmX + pointingArmY * pointingArmY + pointingArmZ * pointingArmZ);
            std::cout << pointingNormal << std::endl;
            pointingDirX = pointingArmX / pointingNormal;
            pointingDirY = pointingArmY / pointingNormal;
            pointingDirZ = pointingArmZ / pointingNormal;
            if(usePointArmLeft)
                JustinaManip::laGoToCartesian(distanceArm * pointingDirX, distanceArm * pointingDirY, distanceArm * pointingDirZ, 0, 0, 1.5708, 0, 3000);
            else
                JustinaManip::raGoToCartesian(distanceArm * pointingDirX, distanceArm * pointingDirY, distanceArm * pointingDirZ, 0, 0, 1.5708, 0, 3000);
            nextState = 2;
            break;
        default:
            std::cout << "NavigTest.->Somebody very stupid programmed this shit. " << std::endl;
            fail = true;
            break;
        }
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}
