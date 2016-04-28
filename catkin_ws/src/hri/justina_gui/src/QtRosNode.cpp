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
    this->pub_Spr_Recognized = this->n->advertise<std_msgs::String>("/hri/sp_rec/recognized", 1);
    this->pub_Spr_Hypothesis = this->n->advertise<hri_msgs::RecognizedSpeech>("/hri/sp_rec/hypothesis", 1);
    
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
}

void QtRosNode::publish_Spr_Recognized(std::string fakeRecoString)
{
    std::cout << "QtRosNode.->Publishing fake recognized command: " << fakeRecoString << std::endl;
    std_msgs::String msg;
    msg.data = fakeRecoString;
    this->pub_Spr_Recognized.publish(msg);
    ros::spinOnce();
}
