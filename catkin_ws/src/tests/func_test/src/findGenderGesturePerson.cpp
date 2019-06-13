#include <iostream>
#include "justina_tools/JustinaTasks.h"

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING A TEST JUST FOR TEST BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "act_pln");
    ros::NodeHandle n;
    JustinaTasks::setNodeHandle(&n);
    ros::Rate loop(10);

    int nextState = 1;
    bool fail = false; 
    bool success = false;
    bool findGesture = false;
    int minDelayAfterSay = 0;
    int maxDelayAfterSay = 300;

    Eigen::Vector3d centroidGesture;
    float gx_w, gy_w, gz_w; 
    float goalx, goaly, goala, angleError;
    float robot_y, robot_x, robot_a;
    std::stringstream ss;
    int numberGuest =0;
    bool reachedGoal = false;
    float dist_to_head;

    geometry_msgs::Pose pose;

    while(ros::ok() && !fail && !success){
        switch(nextState){
        case 1:

            std::cout << "Farewell Test...->SM_SEARCH_WAVING" << std::endl;
            JustinaHRI::waitAfterSay("I will search the guests", 3500, minDelayAfterSay);
            findGesture = JustinaTasks::findGenderGesturePerson("waving", 0, -M_PI_4, M_PI_4 / 2.0, -M_PI_4, -0.2, -0.2, -0.2, 0.0, 0.0, 9.0, centroidGesture);
            if(findGesture){
                JustinaVision::stopSkeletonFinding();
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
                ros::spinOnce();
                
                JustinaTools::transformPoint("/base_link", centroidGesture(0, 0), centroidGesture(1, 0) , centroidGesture(2, 0), "/map", gx_w, gy_w, gz_w);
                JustinaManip::hdGoTo(0.0, 0.0, 1000);

                ss.str("");
                ss << "I noticed that somebody are calling me " << std::endl;
                if(centroidGesture(1, 0) > -0.4 && centroidGesture(1, 0) < 0.4)
                    ss << "in front of me";
                else if(centroidGesture(1, 0) > 0.4)
                    ss << "in my left side";
                else if(centroidGesture(1, 0) < -0.4)
                    ss << "in my right side";
                JustinaHRI::waitAfterSay(ss.str(), 5000, minDelayAfterSay);
                JustinaHRI::waitAfterSay("I am going to approach to you for confirmation", 5000, maxDelayAfterSay);
                JustinaHRI::enableSpeechRecognized(true);
                nextState = 2;
            }
            else
                nextState = 1;                            
            
            break;
        case 2:
            std::cout << "Farewell Test...-> SM_CLOSE_TO_GUEST" << std::endl;
            ss.str("");
            ss << "guest_" << numberGuest;
            JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
            JustinaKnowledge::addUpdateKnownLoc(ss.str(), gx_w, gy_w, atan2(gy_w - robot_y, gx_w - robot_x) - robot_a);
            JustinaKnowledge::getKnownLocation(ss.str(), goalx, goaly, goala);
            std::cout << "Farewell Test...->Centroid gesture:" << goalx << "," << goaly << "," << goala << std::endl;
            reachedGoal = JustinaTasks::closeToLoclWithDistanceTHR(ss.str(), 0.9, 120000);
            JustinaTasks::closeToGoalWithDistanceTHR(gx_w, gy_w, 0.9, 120000);
            reachedGoal = true;
            
            JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
            dist_to_head = sqrt( pow( goalx - robot_x, 2) + pow(goaly- robot_y, 2));
            if(reachedGoal)
                JustinaKnowledge::addUpdateKnownLoc(ss.str(), robot_a);
            float torsoSpine, torsoWaist, torsoShoulders;
            JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist, torsoShoulders);
            float angleHead;
            angleHead = atan2(goaly - robot_y, goalx - robot_x) - robot_a;
            if(angleHead < -M_PI)
                angleHead = 2 * M_PI + angleHead;
            if(angleHead > M_PI)
                angleHead = 2 * M_PI - angleHead;
            JustinaManip::startHdGoTo(angleHead, atan2(gz_w - (1.45 + torsoSpine), dist_to_head));
            nextState=3;
            break;
        case 3:
            
            std::cout << "NavigTest.->Somebody very stupid programmed this shit. " << std::endl;
            success = true;
            break;
        }
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}
