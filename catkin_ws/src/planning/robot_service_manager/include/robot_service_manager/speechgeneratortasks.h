/**
 * @class SpeechGeneratorTasks
 * @brief Perform speech generator tasks.
 *
 * Perform the speech generator tasks using the Microsoft SPGEN module, 
 * running on Windows and connected to BlackBoard, via BBROS Bridge.
 * 
 * @author R. Nonato Lagunas (nonato)
 * @version 0.1
*/
#ifndef _JUSTINA_SPEECHTASKS_H
#define _JUSTINA_SPEECHTASKS_H
#include <string>
#include "ros/ros.h"
#include "bbros_bridge/Default_ROS_BB_Bridge.h"
class SpeechGeneratorTasks
{
    private:
        std::string m_syncSpeechServName; /**< Stores the name of the service 
                                           which will perform the sync text-to
                                           -speech task */
        std::string m_asyncSpeechServName; /**< Stores the name of the service
                                             which will perform the async text-
                                             to-speech task */

    public:
        /**
         * @brief Class constructor
         * 
         * Creates a new SpeechGeneratorTasks object.
         *
         * @param syncSpeechServiceName The name of the service which will 
         * perform the synchronous text-to-speech task. Default "/spg_say"
         * @param asyncSpeechServiceName The name of the service which will 
         * perform the asynchronous text-to-speech task. Default "/spg_say"
         */
        SpeechGeneratorTasks(std::string syncSpeechServName="/spg_say", 
                std::string asyncSpeechServName="/spg_say");
        /**
         * @brief Performs a synchronous text-to-speech task.
         * 
         * Makes a call to a ROS service, advertised by the BBROS-Bridge
         * node, to perform a text-to-speech task using the SPGEN module,
         * running on windows an connected to BlackBoard.
         *
         * @param textToSpeech The message to generate
         * @param timeOut Waiting time (in milliseconds) for the BlackBoard to
         * respond
         * @return true if the task was performed succesfully, false otherwise
         */
        bool syncSpeech(std::string textToSpeech, int timeOut);
        /**
         * @brief Performs an asynchronous text-to-speech task.
         * 
         * Makes a call to a ROS service, advertised by the BBROS-Bridge
         * node, to perform a text-to-speech task using the SPGEN module,
         * running on windows an connected to BlackBoard.
         *
         * @param textToSpeech The message to generate
         * @return void
         */
        void asyncSpeech(std::string textToSpeech);
};
#endif
