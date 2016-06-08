#include "robot_service_manager/speechrecognitionstatus.h"

SpeechRecognitionStatus::SpeechRecognitionStatus(ros::NodeHandle *nh,
        std::string recoSentencesTopic) : 
    m_recoSentencesTopic(recoSentencesTopic)
{
    /**
     * The listen mode is turned off by default
     */
    m_listenActivated = false;
    m_sentenceRecognized = false;

    /**
     * Subscribe to the recognized speech ros topic if the communication with 
     * ROS is initialized.
     */
    m_isInitialized = false; 
    if(ros::isInitialized())
    {
        /**
         * If no node handler provided, then create a new one.
         */
        if(nh == 0)
        {
            nh = new ros::NodeHandle;
        }
        /*
         * Initialize subscribers and verify.
         */
        
        m_subRecoSpeech = nh->subscribe(m_recoSentencesTopic,
                100, &SpeechRecognitionStatus::recoSentenceCallback, this);
        if(m_subRecoSpeech)
        {
            m_isInitialized = true; 
        }
    }
}

void SpeechRecognitionStatus::initRosConnection(ros::NodeHandle *nh)
{
    /**
     * Subscribe to the recognized speech ros topic if the communication with 
     * ROS is initialized.
     */
    if(!m_isInitialized && ros::isInitialized())
    {
        /**
         * If no node handler provided, then create a new one.
         */
        if(nh == 0)
        {
            nh = new ros::NodeHandle;
        }
        /*
         * Initialize subscribers and verify.
         */
        m_subRecoSpeech = nh->subscribe(m_recoSentencesTopic,
                100, &SpeechRecognitionStatus::recoSentenceCallback, this);
        if(m_subRecoSpeech)
        {
            m_isInitialized = true; 
        }

        return;
    }
    /*
    if(m_isInitialized)
    {
    }
    if(ros::isInitialized())
    {
    }
    */
}

void SpeechRecognitionStatus::recoSentenceCallback(const 
        hri_msgs::RecognizedSpeech::ConstPtr &recoSentencesMsg)
{
    /**
     * Clear the last recognized sentences queue in order to enqueue the new
     * recognized sentences
     */
    clearRecoSentencesQueue(m_lastRecoSentencesQueue);

    int hypothesisCount = recoSentencesMsg->hypothesis.size();
    /**
     * Add each recognized sentence to the last recognized sentences queue and, 
     * if the listen mode is activated, in the listen recognized sentences 
     * queue.
     */
    for(int i=0; i<hypothesisCount; i++)
    {
        RecognizedSentence currentHypothesis(recoSentencesMsg->hypothesis[i], 
                recoSentencesMsg->confidences[i]);
        m_lastRecoSentencesQueue.push(currentHypothesis);

        if(m_listenActivated) 
        {
            m_listenRecoSentencesQueue.push(currentHypothesis);
        }
    }
    m_sentenceRecognized = true;
}

void SpeechRecognitionStatus::startListening()
{
    m_sentenceRecognized = false;
    m_listenActivated = true;
}

void SpeechRecognitionStatus::stopListening()
{
    m_listenActivated = false;
}

void SpeechRecognitionStatus::clearRecoSentencesQueue (
        std::queue<RecognizedSentence> &queue)
{
    /**
     * Creates an empty queue
     */
    std::queue<RecognizedSentence> emptyQueue;
    /**
     * Swap the receiven queue with the empty queue
     */
    std::swap( queue, emptyQueue );
}

void SpeechRecognitionStatus::clearListenedSentencesQueue()
{
    if(m_listenActivated) 
    {
        clearRecoSentencesQueue(m_listenRecoSentencesQueue);
    }
}

bool SpeechRecognitionStatus::isSentenceRecognized()
{
    return m_sentenceRecognized;
}

std::string SpeechRecognitionStatus::getLastRecognizedSentence()
{
    return m_lastRecoSentencesQueue.front().hypothesis;
}

