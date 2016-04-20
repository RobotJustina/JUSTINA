#include "justina_tools/JustinaHRI.h"

bool JustinaHRI::is_node_set = false;
//Members for operating speech synthesis and recognition. (Assuming that blackboard modules are used)
ros::ServiceClient JustinaHRI::cltSpGenSay;
ros::Publisher JustinaHRI::pubSprRecognized; 
ros::Publisher JustinaHRI::pubSprHypothesis;
//Members for operating human_follower node
ros::Publisher JustinaHRI::pubFollowStart;
ros::Publisher JustinaHRI::pubFollowStop;

//Variables for speech
std::string JustinaHRI::lastRecoSpeech = "";

//
//The startSomething functions return inmediately after starting the requested action
//The others, block until the action is finished
//

bool JustinaHRI::setNodeHandle(ros::NodeHandle* nh)
{
    if(JustinaNavigation::is_node_set)
        return true;
    if(nh == 0)
        return false;

    std::cout << "JustinaHRI.->Setting ros node..." << std::endl;
    //JustinaHRI::cltSpGenSay = nh->serviceClient<bbros_bridge>("
}

//Methos for speech synthesis and recognition
bool JustinaHRI::waitForSpokenSentence(std::string& recognizedSentence, int timeOut_ms){
}
}

void JustinaHRI::fakeSpokenSentence(std::string sentence)
{
}

void JustinaHRI::startSay(std::string strToSay)
{
}

void JustinaHRI::say(std::string strToSay)
{
}

//Methods for human following
void JustinaHRI::startFollowHuman()
{
}

void JustinaHRI::stopFollowHuman()
{
}

