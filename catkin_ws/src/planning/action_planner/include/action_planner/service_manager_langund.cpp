#include "action_planner/service_manager.h"
#include "language_understanding/parse_sentence.h"

/*
*	Implements a synchronous call to the language understanding module to parse a string sentence
*	Receives:
*		stringToProcess : the strin sentence to process
*		conceptualDependency: the result of the natural language processing (command format)
*/
bool ServiceManager::langundProcess(std::string stringToProcess, std::string &conceptualDependency)
{
	std::string serviceName("/language_understanding/parse_sentence");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<language_understanding::parse_sentence>(serviceName);

	language_understanding::parse_sentence srv;
	srv.request.sentence = stringToProcess;
	
	if(client.call(srv))
	{
		ROS_DEBUG_STREAM_NAMED("action_planner", serviceName << " service called successfully with parameters: " << srv.request);
		conceptualDependency = srv.response.conceptual_dependency;
		/*verify if the sentence was succesfully parsed*/
		if(srv.response.conceptual_dependency.compare("not_parsed") != 0)
			return true;
		ROS_DEBUG_STREAM_NAMED("action_planner",  " sentence not parsed correctly.");
	}

	ROS_ERROR_STREAM_NAMED("action_planner", "an error acurred when trying to call the " << serviceName << " service with parameters: " << srv.request);
	return false;
}
