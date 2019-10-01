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
    std::vector<std::string> ids;
    ids.push_back("person");
    bool isFound;

    JustinaTasks::POSE poseRecog = JustinaTasks::NONE;
    float torsoSpine, torsoWaist, torsoShoulders;
    std::vector<Eigen::Vector3d> centroids;
    std::vector<std::string> centroids_loc;
    float robot_x, robot_y, robot_a;
    float gx_w, gy_w, gz_w;
    std::stringstream ss_loc; 
    float goalx, goaly, goala;
    float theta = 0, thetaToGoal = 0, angleHead = 0;
    float dist_to_head;

    while(ros::ok() && !fail && !success){
        switch(nextState){
        case 1:
            std::cout << "trying find a person yolo " << std::endl;
            //success = JustinaTasks::findYolo(idsPerson, poseRecog, JustinaTasks::NONE, tokens[1]);
            centroids.clear();
            //isFound = JustinaTasks::turnAndRecognizeGesture("waving", -M_PI_4, M_PI_4 / 2.0, M_PI_4, -0.2, -0.2, -0.2, 0.0, 0.0f, 9.0, centroids, "", true, 0, 0.7);
            isFound = JustinaTasks::turnAndRecognizeYolo(ids, poseRecog, -M_PI_4, M_PI_4 / 2.0, M_PI_4, -0.2, -0.2, -0.3, 0.1, 0.1f, 8.0, centroids, "", 0, 1.0);
            if(isFound){
                JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                for(int i = 0; i < centroids.size(); i++)
                {
                    Eigen::Vector3d centroid = centroids[i];
                    JustinaTools::transformPoint("/base_link", centroid(0, 0), centroid(1, 0) , centroid(2, 0), "/map", gx_w, gy_w, gz_w);
                    ss_loc << "person_" << i;
                    centroids_loc.push_back(ss_loc.str());
                    JustinaKnowledge::addUpdateKnownLoc(ss_loc.str(), gx_w, gy_w, atan2(gy_w - robot_y, gx_w - robot_x) - robot_a);
                }
                nextState = 2;
            }
            else
                nextState = 1;
            break;
        case 2:
            std::cout << "go to the person locations " << std::endl;
            if(centroids_loc.size() > 0){
                JustinaKnowledge::getKnownLocation(centroids_loc[0] ,goalx, goaly, goala);
                JustinaTasks::closeToGoalWithDistanceTHR(goalx, goaly, 1.3, 30000);
                JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                thetaToGoal = atan2(goaly - robot_y, goalx - robot_x);
                if (thetaToGoal < 0.0f)
                    thetaToGoal += 2 * M_PI;
                theta = thetaToGoal - robot_a;
                JustinaNavigation::moveDistAngle(0, theta, 3000);
                dist_to_head = sqrt( pow( goalx - robot_x, 2) + pow(goaly - robot_y, 2));
                JustinaHardware::getTorsoCurrentPose(torsoSpine, torsoWaist, torsoShoulders);
                JustinaNavigation::getRobotPose(robot_x, robot_y, robot_a);
                angleHead = atan2(goaly - robot_y, goalx - robot_x) - robot_a;
                if(angleHead < -M_PI)
                    angleHead = 2 * M_PI + angleHead;
                if(angleHead > M_PI)
                    angleHead = 2 * M_PI - angleHead;
                JustinaManip::startHdGoTo(angleHead, atan2(gz_w - (1.45 + torsoSpine), dist_to_head));
                centroids_loc.erase(centroids_loc.begin());
                nextState = 2;
            }
            else
                nextState = 3;
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
