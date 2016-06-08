#include "robot_service_manager/langunderstandingtasks.h"

LangUnderstandingTasks::LangUnderstandingTasks (
        std::string parseSentenceServName) : m_parseSentenceServName(
            parseSentenceServName)
{
}

bool LangUnderstandingTasks::isMembershipSentence(std::string t_sentence,
        std::string &t_theme, std::string &t_value)
{
    CommandFrame parseResult;
    parseSentence(t_sentence, parseResult);

    //Verify if the sentence belongs to a membership command 
    if(parseResult.command.compare("MEMBERSHIP") != 0)
    {
        return false;
    }

    //Store the goal parameter of the start follow instruction
    t_theme = parseResult.params["theme"];
    t_value = parseResult.params["value"];

    return true;
}

bool LangUnderstandingTasks::isStartGuideInstruction(std::string t_sentence, 
                std::string &t_goalToGuide)
{
    CommandFrame parseResult;
    parseSentence(t_sentence, parseResult);

    /**
     * Verify if the sentence belongs to a switch command and if the asociated
     * action and value is guide and start respectively.
     */
    if(parseResult.command.compare("SWITCH") != 0)
    {
        return false;
    }

    if((parseResult.params["action"].compare("guide") != 0 && 
                parseResult.params["action"].compare("guiding") != 0) ||
            parseResult.params["value"].compare("start") != 0)
    {
        return false;
    }
    
    /**
     * Store the goal parameter of the start follow instruction
     */
    t_goalToGuide = parseResult.params["goal"];

    return true;
}

bool LangUnderstandingTasks::isStartFollowInstruction(std::string sentence, 
                std::string &goalToFollow)
{
    CommandFrame parseResult;
    parseSentence(sentence, parseResult);

    /**
     * Verify if the sentence belongs to a switch command and if the asociated
     * action and value is follow and start respectively.
     */
    if(parseResult.command.compare("SWITCH") != 0)
    {
        return false;
    }

    if((parseResult.params["action"].compare("follow") != 0 && 
                parseResult.params["action"].compare("following") != 0) ||
            parseResult.params["value"].compare("start") != 0)
    {
        return false;
    }
    
    /**
     * Store the goal parameter of the start follow instruction
     */
    goalToFollow = parseResult.params["goal"];

    return true;
}

bool LangUnderstandingTasks::isStopFollowInstruction(std::string sentence, 
                std::string &goalToUnfollow)
{
    CommandFrame parseResult;
    parseSentence(sentence, parseResult);

    /**
     * Verify if the sentence belongs to a switch command and if the asociated
     * action and value is follow and start respectively.
     */
    if(parseResult.command.compare("SWITCH") != 0)
    {
        return false;
    }

    if((parseResult.params["action"].compare("follow") != 0 && 
                parseResult.params["action"].compare("following") != 0) ||
            parseResult.params["value"].compare("stop") != 0)
    {
        return false;
    }
    
    /**
     * Store the goal parameter of the start follow instruction
     */
    goalToUnfollow = parseResult.params["goal"];

    return true;

}

bool LangUnderstandingTasks::isFollowGoalInstruction(std::string sentence, 
        std::string &goalToFollow)
{
    CommandFrame parseResult;
    parseSentence(sentence, parseResult);

    /**
     * Verify if the sentence belongs to a motion instruction and if its path
     * is follow
     */
    if(parseResult.command.compare("MOTION") != 0)
    {
        return false;
    }

    if(parseResult.params["path"].compare("follow") != 0 && 
            parseResult.params["path"].compare("following") != 0)
    {
        return false;
    }
    
    /**
     * Store the goal parameter of the follow instruction
     */
    goalToFollow = parseResult.params["goal"];

    return true;
}

bool LangUnderstandingTasks::isPositiveUserConfirmation(std::string sentence)
{
    CommandFrame parseResult;
    parseSentence(sentence, parseResult);

    if(parseResult.command.compare("CONFIRMATION") == 0)
    {
        if(parseResult.params["confirmation"].compare("yes")==0)
        {
            return true;
        }
    }

    return false;
}

bool LangUnderstandingTasks::isNegativeUserConfirmation(std::string sentence)
{
    CommandFrame parseResult;
    parseSentence(sentence, parseResult);

    if(parseResult.command.compare("CONFIRMATION") == 0)
    {
        if(parseResult.params["confirmation"].compare("no")==0)
        {
            return true;
        }
    }

    return false;
}

bool LangUnderstandingTasks::parseSentence(std::string sentenceToParse, 
        CommandFrame &parseResult)
{
    if(!ros::isInitialized())
    {
        return false;
    }

    /**
     * Create a client object to call the ROS service
     */
	ros::NodeHandle nodeHandler;
	ros::ServiceClient client = nodeHandler.serviceClient
        <planning_msgs::parse_sentence_cfr>(m_parseSentenceServName);

    /**
     * Create a srv object to send the request to the ROS service
     */
	planning_msgs::parse_sentence_cfr srv;
	srv.request.sentence = sentenceToParse;
	
    /**
     * Call the ROS service
     */
	if(client.call(srv))
	{
		/**
         * Verify if the sentence was succesfully parsed
         */
		if(srv.response.cfr.command.compare("NO_INTERPRETATION") == 0)
        {
			return false;
        }

        /**
         * To store the command an parameters resulting from the parsing.
         */
        std::string resultCommand;
        std::map<std::string, std::string> resultParams;

        /**
         * Storing the parameters resulting from the parsing.
         */
        resultCommand = srv.response.cfr.command;
        for(int i=0; i<srv.response.cfr.params.size(); i++)
        {
            resultParams.insert(std::pair<std::string, std::string> (
                        srv.response.cfr.params[i].frame_id, 
                        srv.response.cfr.params[i].frame_value
                        ));
        }
        parseResult.command = resultCommand;
        parseResult.params = resultParams;

        return true;
	}

	return false;
}

bool LangUnderstandingTasks::isValidCommand(std::string sentence, 
        std::string &command,
        std::map<std::string, std::string> &params)
{
    command = "";

    //Verify if the sentence belongs to a valid voice command
    CommandFrame parseResult;
    if(!parseSentence(sentence, parseResult))
    {
        return false;
    }

    command = parseResult.command;
    params = parseResult.params;
    
    return true;
}
