#include "QtRosNode.h"

QtRosNode::QtRosNode()
{
    this->gui_closed = false;
}

QtRosNode::~QtRosNode()
{
}

void QtRosNode::run()
{    
    ros::Rate loop(10);
    pubEqualizer = n->advertise<std_msgs::Float32MultiArray>("/audio/equalizer", 1);
    while(ros::ok() && !this->gui_closed)
    {
        //std::cout << "Ros node running..." << std::endl;
        emit updateGraphics();
        ros::spinOnce();
        loop.sleep();
    }
    emit onRosNodeFinished();
}

void QtRosNode::setNodeHandle(ros::NodeHandle* nh)
{
    this->n = nh;
    
}

void QtRosNode::sendEqualizer(float b1, float b2, float b3, float b4, float b5, float b6, float b7, float b8, float b9, float b10)
{
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(b1);
    msg.data.push_back(b2);
    msg.data.push_back(b3);
    msg.data.push_back(b4);
    msg.data.push_back(b5);
    msg.data.push_back(b6);
    msg.data.push_back(b7);
    msg.data.push_back(b8);
    msg.data.push_back(b9);
    msg.data.push_back(b10);
    pubEqualizer.publish(msg);
}
