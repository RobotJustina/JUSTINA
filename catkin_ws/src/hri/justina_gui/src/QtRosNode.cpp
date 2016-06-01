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
    JustinaHardware::setNodeHandle(nh);
    JustinaNavigation::setNodeHandle(nh);
    JustinaHRI::setNodeHandle(nh);
    JustinaManip::setNodeHandle(nh);
    JustinaVision::setNodeHandle(nh);
    JustinaTools::setNodeHandle(nh);
}
