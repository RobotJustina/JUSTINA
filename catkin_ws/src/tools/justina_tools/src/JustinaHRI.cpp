#include "justina_tools/JustinaHRI.h"

bool JustinaHRI::is_node_set = false;
//Members for operating speech synthesis and recognition. (Assuming that blackboard modules are used)
ros::Publisher JustinaHRI::pubFakeSprRecognized; 
ros::Publisher JustinaHRI::pubFakeSprHypothesis;
ros::Subscriber JustinaHRI::subSprRecognized; 
ros::Subscriber JustinaHRI::subSprHypothesis;
ros::ServiceClient JustinaHRI::cltSpgSay;
//Members for operating human_follower node
ros::Publisher JustinaHRI::pubFollowStartStop;
ros::Publisher JustinaHRI::pubLegsEnable;
ros::Subscriber JustinaHRI::subLegsFound;
//Variables for speech
std::string JustinaHRI::_lastRecoSpeech = "";
std::vector<std::string> JustinaHRI::_lastSprHypothesis;
std::vector<float> JustinaHRI::_lastSprConfidences;
bool JustinaHRI::newSprRecognizedReceived = false;
bool JustinaHRI::_legsFound;
//Variabeles for qr reader
ros::Subscriber JustinaHRI::subQRReader;
boost::posix_time::ptime JustinaHRI::timeLastQRReceived = boost::posix_time::second_clock::local_time();
std::string JustinaHRI::lastQRReceived;

//
//The startSomething functions return inmediately after starting the requested action
//The others, block until the action is finished
//

bool JustinaHRI::setNodeHandle(ros::NodeHandle* nh)
{
    if(JustinaHRI::is_node_set)
        return true;
    if(nh == 0)
        return false;

    pubFakeSprHypothesis = nh->advertise<hri_msgs::RecognizedSpeech>("/recognizedSpeech", 1);
    pubFakeSprRecognized = nh->advertise<std_msgs::String>("/hri/sp_rec/recognized", 1);
    subSprHypothesis = nh->subscribe("/recognizedSpeech", 1, &JustinaHRI::callbackSprHypothesis);
    subSprRecognized = nh->subscribe("/hri/sp_rec/recognized", 1, &JustinaHRI::callbackSprRecognized);
    cltSpgSay = nh->serviceClient<bbros_bridge::Default_ROS_BB_Bridge>("/spg_say");

    pubFollowStartStop = nh->advertise<std_msgs::Bool>("/hri/human_following/start_follow", 1);
    pubLegsEnable = nh->advertise<std_msgs::Bool>("/hri/leg_finder/enable", 1);
    subLegsFound = nh->subscribe("/hri/leg_finder/legs_found", 1, &JustinaHRI::callbackLegsFound);
    std::cout << "JustinaHRI.->Setting ros node..." << std::endl;
    //JustinaHRI::cltSpGenSay = nh->serviceClient<bbros_bridge>("
    subQRReader = nh->subscribe("/hri/qr/recognized", 1, &JustinaHRI::callbackQRRecognized);
}

//Methos for speech synthesis and recognition
bool JustinaHRI::waitForSpeechRecognized(std::string& recognizedSentence, int timeOut_ms)
{
    newSprRecognizedReceived = false;
    int attempts = timeOut_ms / 100;
    ros::Rate loop(10);
    while(ros::ok() && !newSprRecognizedReceived && --attempts > 0)
    {
        ros::spinOnce();
        loop.sleep();
    }
    if(newSprRecognizedReceived)
    {
        recognizedSentence = _lastRecoSpeech;
        return true;
    }
    else
    {
        recognizedSentence = "";
        return false;
    }
}

bool JustinaHRI::waitForSpeechHypothesis(std::vector<std::string>& sentences, std::vector<float>& confidences, int timeOut_ms)
{
    newSprRecognizedReceived = false;
    int attempts = timeOut_ms / 100;
    ros::Rate loop(10);
    while(ros::ok() && !newSprRecognizedReceived && --attempts > 0)
    {
        ros::spinOnce();
        loop.sleep();
    }
    if(newSprRecognizedReceived)
    {
        sentences = _lastSprHypothesis;
        confidences = _lastSprConfidences;
        return true;
    }
    else
    {
        sentences.clear();
        confidences.clear();
        return false;
    }
}

bool JustinaHRI::waitForSpecificSentence(std::string expectedSentence, int timeOut_ms)
{
    std::vector<std::string> sentences;
    std::vector<float> confidences;
    if(!waitForSpeechHypothesis(sentences, confidences, timeOut_ms))
        return false;
    for(size_t i=0; i<sentences.size(); i++)
        if(expectedSentence.compare(sentences[i]) == 0)
            return true;
    return false;
}

bool JustinaHRI::waitForSpecificSentence(std::string option1, std::string option2, std::string& recog, int timeOut_ms)
{
    std::vector<std::string> sentences;
    std::vector<float> confidences;
    if(!waitForSpeechHypothesis(sentences, confidences, timeOut_ms))
        return false;
    for(size_t i=0; i<sentences.size(); i++)
        if(option1.compare(sentences[i]) == 0 || option2.compare(sentences[i]) == 0)
        {
            recog = sentences[i];
            return true;
        }
    return false;
}

bool JustinaHRI::waitForSpecificSentence(std::string option1, std::string option2, std::string option3,
                                    std::string& recog, int timeOut_ms)
{
    std::vector<std::string> sentences;
    std::vector<float> confidences;
    if(!waitForSpeechHypothesis(sentences, confidences, timeOut_ms))
        return false;
    for(size_t i=0; i<sentences.size(); i++)
        if(option1.compare(sentences[i]) == 0 || option2.compare(sentences[i]) == 0 || option3.compare(sentences[i]) == 0)
        {
            recog = sentences[i];
            return true;
        }
    return false;
}

bool JustinaHRI::waitForSpecificSentence(std::string option1, std::string option2, std::string option3, std::string option4,
                                    std::string& recog, int timeOut_ms)
{
    std::vector<std::string> sentences;
    std::vector<float> confidences;
    if(!waitForSpeechHypothesis(sentences, confidences, timeOut_ms))
        return false;
    for(size_t i=0; i<sentences.size(); i++)
        if(option1.compare(sentences[i]) == 0 || option2.compare(sentences[i]) == 0 ||
           option3.compare(sentences[i]) == 0 || option4.compare(sentences[i]) == 0)
        {
            recog = sentences[i];
            return true;
        }
    return false;
}

bool JustinaHRI::waitForSpecificSentence(std::vector<std::string>& options, std::string& recognized, int timeOut_ms)
{
    std::vector<std::string> sentences;
    std::vector<float> confidences;
    if(!waitForSpeechHypothesis(sentences, confidences, timeOut_ms))
        return false;
    for(size_t i=0; i<sentences.size(); i++)
        for(size_t j=0; j<options.size(); j++)
            if(options[j].compare(sentences[i]) == 0)
            {
                recognized = sentences[i];
                return true;
            }
    return false;
}

bool JustinaHRI::waitForUserConfirmation(bool& confirmation, int timeOut_ms)
{
    std::vector<std::string> sentences;
    std::vector<float> confidences;
    if(!waitForSpeechHypothesis(sentences, confidences, timeOut_ms))
        return false;
    for(size_t i=0; i<sentences.size(); i++)
    {
        if(sentences[i].compare("robot yes") == 0)
        {
            confirmation = true;
            return true;
        }
        if(sentences[i].compare("robot no") == 0)
        {
            confirmation = false;
            return true;
        }
    }
    return false;
}

std::string JustinaHRI::lastRecogSpeech()
{
    return _lastRecoSpeech;
}

void JustinaHRI::fakeSpeechRecognized(std::string sentence)
{
    std_msgs::String str;
    hri_msgs::RecognizedSpeech spr;
    str.data = sentence;
    spr.hypothesis.push_back(sentence);
    spr.confidences.push_back(0.9);
    pubFakeSprRecognized.publish(str);
    pubFakeSprHypothesis.publish(spr);
}

void JustinaHRI::startSay(std::string strToSay)
{
}

void JustinaHRI::say(std::string strToSay)
{
    std::cout << "JustinaHRI.->Saying: " << strToSay << std::endl;
    bbros_bridge::Default_ROS_BB_Bridge srv;
    srv.request.parameters = strToSay;
    srv.request.timeout = 10000;
    cltSpgSay.call(srv);
}

//Methods for human following
void JustinaHRI::startFollowHuman()
{
    std_msgs::Bool msg;
    msg.data = true;
    JustinaHRI::pubFollowStartStop.publish(msg);
}

void JustinaHRI::stopFollowHuman()
{
    std_msgs::Bool msg;
    msg.data = false;
    JustinaHRI::pubFollowStartStop.publish(msg);
}

void JustinaHRI::enableLegFinder(bool enable)
{
    if(!enable)
    {
        JustinaHRI::_legsFound = false;
        std::cout << "JustinaHRI.->Leg_finder disabled. " << std::endl;
    }
    else
        std::cout << "JustinaHRI.->Leg_finder enabled." << std::endl;
    std_msgs::Bool msg;
    msg.data = enable;
    JustinaHRI::pubLegsEnable.publish(msg);
}

bool JustinaHRI::frontalLegsFound()
{
    return JustinaHRI::_legsFound;
}
                     
void JustinaHRI::callbackSprRecognized(const std_msgs::String::ConstPtr& msg)
{
    _lastRecoSpeech = msg->data;
    newSprRecognizedReceived = true;
    std::cout << "JustinaHRI.->Received recognized speech: " << msg->data << std::endl;
}

void JustinaHRI::callbackSprHypothesis(const hri_msgs::RecognizedSpeech::ConstPtr& msg)
{
    if(msg->hypothesis.size() < 1 || msg->confidences.size() < 1)
    {
        std::cout << "JustinaHRI.->Invalid speech recog hypothesis: msg is empty" << std::endl;
        return;
    }
    _lastRecoSpeech = msg->hypothesis[0];
    _lastSprHypothesis = msg->hypothesis;
    _lastSprConfidences = msg->confidences;
    std::cout << "JustinaHRI.->Last reco speech: " << _lastRecoSpeech << std::endl;
    newSprRecognizedReceived = true;
}

void JustinaHRI::callbackLegsFound(const std_msgs::Empty::ConstPtr& msg)
{
    std::cout << "JustinaHRI.->Legs found signal received!" << std::endl;
    JustinaHRI::_legsFound = true;
}

//Methods for qr reader
 void JustinaHRI::callbackQRRecognized(const std_msgs::String::ConstPtr& msg){
    std::cout << "JustinaHRI.->Qr reader received" << std::endl;
    boost::posix_time::ptime timeCurrQRReceived = boost::posix_time::second_clock::local_time();
    if(lastQRReceived.compare(msg->data) != 0 || (timeCurrQRReceived - timeLastQRReceived).total_milliseconds() > 5000){
        timeLastQRReceived = boost::posix_time::second_clock::local_time();
        lastQRReceived = msg->data;
	std_msgs::String str;
    	hri_msgs::RecognizedSpeech spr;
    	str.data = msg->data;
    	spr.hypothesis.push_back(msg->data);
    	spr.confidences.push_back(0.9);
    	pubFakeSprRecognized.publish(str);
    	pubFakeSprHypothesis.publish(spr);
    }
 }
