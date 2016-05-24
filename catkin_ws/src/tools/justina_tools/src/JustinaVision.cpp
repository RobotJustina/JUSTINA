#include "justina_tools/JustinaVision.h"

bool JustinaVision::is_node_set = false;
//Members for operating skeleton finder
ros::Publisher JustinaVision::pubSktStartRecog;
ros::Publisher JustinaVision::pubSktStopRecog;
//Members for operating face recognizer
ros::Publisher JustinaVision::pubFacStartRecog;
ros::Publisher JustinaVision::pubFacStopRecog;

bool JustinaVision::setNodeHandle(ros::NodeHandle* nh)
{
    if(JustinaVision::is_node_set)
        return true;
    if(nh == 0)
        return false;

    std::cout << "JustinaVision.->Setting ros node..." << std::endl;
    JustinaVision::pubSktStartRecog = nh->advertise<std_msgs::Empty>("/vision/skeleton_finder/start_recog", 1);
    JustinaVision::pubSktStopRecog = nh->advertise<std_msgs::Empty>("/vision/skeleton_finder/stop_recog", 1);
    //Members for operating face recognizer
    JustinaVision::pubFacStartRecog = nh->advertise<std_msgs::Empty>("/vision/face_recognizer/start_recog", 1);
    JustinaVision::pubFacStopRecog = nh->advertise<std_msgs::Empty>("/vision/face_recognizer/stop_recog", 1);

    JustinaVision::is_node_set = true;
    return true;
}

//Methods for operating skeleton finder
void JustinaVision::startSkeletonFinding()
{
    std_msgs::Empty msg;
    JustinaVision::pubSktStartRecog.publish(msg);
}

void JustinaVision::stopSkeletonFinding()
{
    std_msgs::Empty msg;
    JustinaVision::pubSktStopRecog.publish(msg);
}
//Methods for operating face recognizer
void JustinaVision::startFaceRecognition()
{
    std_msgs::Empty msg;
    JustinaVision::pubFacStartRecog.publish(msg);
}

void JustinaVision::stopFaceRecognition()
{
    std_msgs::Empty msg;
    JustinaVision::pubFacStopRecog.publish(msg);
}
