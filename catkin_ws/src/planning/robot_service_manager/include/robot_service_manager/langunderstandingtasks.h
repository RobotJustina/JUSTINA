/**
 * @class LangUnderstandingTasks
 * @brief Perform language uderstanding tasks.
 *
 * Perform language understanding tasks by calling a ROS service wich uses the 
 * planning_msgs/parse_sentence_cfr srv format.
 * 
 * @author R. Nonato Lagunas (nonato)
 * @version 0.1
*/
#ifndef _JUSTINA_LANGUNDTASKS_H
#define _JUSTINA_LANGUNDTASKS_H
#include <string>
#include <map>
#include "ros/ros.h"
#include "planning_msgs/parse_sentence_cfr.h"
class LangUnderstandingTasks
{
    private:
        /**
         * @struct CommandFrame
         * @brief A structure to store the Command Frame Representation of a 
         * parsed sentence.
         *
         * @var CommandFrame::command Contains the command corresponding
         * to the sentence.
         * @var CommandFrame::params Contains the pair parameters-values of the
         * corresponding command.
         * @var CommandFrame::CFiterator Iterator to find keys in the params
         * map.
         */
        struct CommandFrame
        {
            std::string command;
            std::map<std::string, std::string> params;

            /**
             * @brief Creates a new CommandFrame struct object.
             *
             * @param newCommand The command to store.
             * @param newParams The parameters to store.
             */
            CommandFrame(std::string newCommand = "", 
                    std::map<std::string, std::string> newParams = 
                    std::map<std::string, std::string>()) :
                command(newCommand), params(newParams) {}
        };

        std::string m_parseSentenceServName; /**< Stores the name of the
                                               service which will perform
                                               the parse sentence task.*/

        /**
         * @brief Performs the parse sentence task.
         * 
         * Makes a call to a ROS service, advertised by the language 
         * understanding node, to perform a sentence parsing task.
         *
         * @param sentenceToParse The sentence to parse
         * @param parseResult Stores the result command and parameters of the
         * parse sentence task.
         * @return true if the task was performed succesfully, false otherwise
         */
        bool parseSentence(std::string sentenceToParse, 
                CommandFrame &parseResult);

    public:
        /**
         * @brief Class constructor
         * 
         * Creates a new LangUnderstandingTasks object.
         *
         * @param parseSentenceServName The name of the service which will 
         * perform the synchronous text-to-speech task. Default "/spg_say"
         */
        LangUnderstandingTasks(std::string parseSentenceServName=
                "/language_understanding/parse_sentence_cfr");

        /**
         * @brief Indicates if a given sentence corresponds to a positive 
         * confirmation (eg. robot yes).
         *
         * @param sentence The sentence to analyse.
         * @return True if the sentence corresponds to a positive  confirmation,
         * False otherwise.
         */
        bool isPositiveUserConfirmation(std::string sentence);

        /**
         * @brief Indicates if a given sentence corresponds to a negative 
         * confirmation (eg. robot no).
         *
         * @param sentence The sentence to analyse.
         * @return True if the sentence corresponds to a negative confirmation,
         * False otherwise.
         */
        bool isNegativeUserConfirmation(std::string sentence);

        /**
         * @brief Indicates if a given sentence corresponds to a follow goal  
         * instruction.
         *
         * @param[in] sentence The sentence to analyse.
         * @param[out] goalToFollow The goal to follow.
         * @return True if the sentence corresponds to a follow goal
         * instruction, False otherwise.
         */
        bool isFollowGoalInstruction(std::string sentence, 
                std::string &goalToFollow);

        /**
         * @brief Indicates if a given sentence corresponds to a start switch 
         * command of the follow goal instruction.
         *
         * @param[in] sentence The sentence to analyse.
         * @param[out] goalToFollow The goal to follow.
         * @return True if the sentence corresponds to a start switch command
         * of the follow goal instruction, False otherwise.
         */
        bool isStartFollowInstruction(std::string sentence, 
                std::string &goalToFollow);

        /**
         * Indicates if a given sentence corresponds to a start switch 
         * command of the guide goal instruction.
         *
         * @param[in] t_sentence The sentence to analyse.
         * @param[out] t_goalToGuide The goal to follow.
         * @return True if the sentence corresponds to a start switch command
         * of the guide goal instruction, False otherwise.
         */
        bool isStartGuideInstruction(std::string t_sentence, 
                std::string &t_goalToGuide);

        /**
         * @brief Indicates if a given sentence corresponds to a stop switch 
         * command of the follow goal instruction.
         *
         * @param[in] sentence The sentence to analyse.
         * @param[out] goalToUnfollow The goal to stop follow.
         * @return True if the sentence corresponds to a stop switch command
         * of the follow goal instruction, False otherwise.
         */
        bool isStopFollowInstruction(std::string sentence, 
                std::string &goalToUnfollow);

        /**
         * Indicates if a given sentence corresponds to a membership
         * instruction. This method can parse sentence like:
         * 'my name is john'
         * 'my tie is black'
         * 'her son is luis'
         *
         * @param[in] t_sentence The sentence to analyse.
         * @param[out] t_theme The membership object. In the sentence 'my name
         * is justina' the object corresponds to the word 'name'.
         * @param[out] t_value The membership object value. In the sentence 'my
         * name is justina' the object value corresponds to the word 'justina'.
         * @return True if the sentence corresponds to a membership 
         * instruction. False otherwise.
         */
        bool isMembershipSentence(std::string t_sentence, std::string &t_theme,
                std::string &t_value);

        /**
         * @brief Indicates if a given sentence corresponds to a valid voice
         * command.
         *
         * @param[in] sentence The sentence to anlyse.
         * @param[out] command The detected command as a result of the parsing.
         * @param[out] params The corresponding params of the detected command.
         * @return True if the given sentence is a valid voice command. False
         * otherwise.
         */
        bool isValidCommand(std::string sentence, std::string &command,
                std::map<std::string, std::string> &params);
};
#endif
