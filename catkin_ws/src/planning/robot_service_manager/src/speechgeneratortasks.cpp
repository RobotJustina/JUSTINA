#include "robot_service_manager/speechgeneratortasks.h"
SpeechGeneratorTasks::SpeechGeneratorTasks(std::string syncSpeechServName, 
        std::string asyncSpeechServName) :
        m_syncSpeechServName(syncSpeechServName), m_asyncSpeechServName(
                asyncSpeechServName)
{
}

bool SpeechGeneratorTasks::syncSpeech(std::string textToSpeech, int timeOut)
{
    if(!ros::isInitialized())
    {
        return false;
    }

    ros::NodeHandle nodeHandler;
    ros::ServiceClient client = nodeHandler.serviceClient
        <bbros_bridge::Default_ROS_BB_Bridge>(m_syncSpeechServName);

    bbros_bridge::Default_ROS_BB_Bridge srv;
    srv.request.parameters = textToSpeech;
    srv.request.timeout = timeOut;

    if(client.call(srv))
    {
        return true;
    }
    return false;
}

void SpeechGeneratorTasks::asyncSpeech(std::string textToSpeech)
{
    if(!ros::isInitialized())
    {
        return;
    }
    ros::NodeHandle nodeHandler;
    ros::ServiceClient client = nodeHandler.serviceClient
        <bbros_bridge::Default_ROS_BB_Bridge>(m_asyncSpeechServName);

    bbros_bridge::Default_ROS_BB_Bridge srv;
    srv.request.parameters = textToSpeech;
    srv.request.timeout = 0;

    client.call(srv);
    //if(client.call(srv))
    //{
    //    return;
    //}
    //return ;
}
