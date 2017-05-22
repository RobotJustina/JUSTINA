#include <iostream>
#include "nav_msgs/Path.h"
#include "tf/transform_listener.h"

class ObstacleDetector
{
public:
    ObstacleDetector();
    ~ObstacleDetector();

    void initROSConnection(ros::NodeHandle* n); 
    void getRobotPose(float& robotX, float& robotY, float& robotTheta);
    bool checkCollision(nav_msgs::Path& path);
};
