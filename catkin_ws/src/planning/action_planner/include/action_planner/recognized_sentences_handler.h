#ifndef ACT_PLN_SP_REC_HAND
#define ACT_PLN_SP_REC_HAND

#include "ros/ros.h"
#include "bbros_bridge/RecognizedSpeech.h"
#include <deque>
#include <string>

namespace RecognizedSentencesHandler
{
	/*
	* Recognized sentences data structure
	*/
	struct recoSentenceTuple
	{
		recoSentenceTuple() : hypothesis(""), confidences(0.0) {}
		std::string hypothesis;
		double confidences;
	};
	
	/*
	*	Robot speech recognition queuqueue
	*/
	std::deque<recoSentenceTuple> recognizedSentences;

	/*
	* Variables to enable/disable the listen mode
	*/
	bool listenSpeechReco = false;
	bool speechRecoUpdated = false;
	
	/*
	*	Listen to the recognizedSentence topic to update the current recognizedSentence data structure
	*/
	void updateRecognizedSentences(const bbros_bridge::RecognizedSpeech::ConstPtr& recognizedSpeech)
	{
		int recoSpeechSize = recognizedSpeech->hypothesis.size();
		if(listenSpeechReco)
		{
			for(int i=0; i<recoSpeechSize; i++)
			{
				recoSentenceTuple recoSentence;
				recoSentence.hypothesis = recognizedSpeech->hypothesis[i];
				recoSentence.confidences = recognizedSpeech->confidences[i];
				recognizedSentences.push_back(recoSentence);
			}
			//only notify if was required (inside listen mode)
			speechRecoUpdated = true;
		}
		else
		{
			//TODO:Send log message "received but no enqueued"
		}
	}

	/*
	* Enters to listen mode and waits until the robot heard a sentence or timeout
	*	Receives:
	*		confidentSentence: the most confident sentence heared (reference)
	*		timeout: the duration of the listen 
	* 	Returns:	
	*		true: if the robot heard something
	*		false: otherwise
	*/
	bool listen(std::string &confidentSentence, int timeout)
	{
		//to flush the topic queue
		ros::spinOnce();
		//set up listen status
		recognizedSentences.clear();
		speechRecoUpdated = false;
		
		//start listening
		listenSpeechReco = true;
		
		//wait until listen some instruction or timeout
		ros::Rate loop_rate(1000);
		int currentTimeout = 0;
		while(!speechRecoUpdated && (currentTimeout++)<timeout)
		{
			ros::spinOnce();
			loop_rate.sleep();
		}

		//stop listening
		listenSpeechReco = false;

		//verify if something was listened
		if(currentTimeout >= timeout)
		{
			//nothing listened
			confidentSentence = "";
			return false;
		}

		confidentSentence = recognizedSentences[0].hypothesis;
		return true;
	}

	/*
	* Enters to listen mode and waits until the robot heard a sentence or timeout
	*	Receives:
	*		timeout: the duration of the listen 
	* 	Returns:	
	*		a copy of the recognized sentences queue (a queue empty if the robot does not heard nothing)
	*/
	std::deque<recoSentenceTuple> listen(int timeout)
	{
		//to flush the topic queue
		ros::spinOnce();

		//set up listen status
		recognizedSentences.clear();
		speechRecoUpdated = false;
		
		//start listening
		listenSpeechReco = true;
		
		//wait until listen some instruction or timeout
		ros::Rate loop_rate(1000);
		int currentTimeout = 0;
		while(!speechRecoUpdated && (currentTimeout++)<timeout)
		{
			ros::spinOnce();
			loop_rate.sleep();
		}

		//stop listening
		listenSpeechReco = false;

		//verify if something was listened
		if(currentTimeout >= timeout)
			//nothing listened
			return std::deque<recoSentenceTuple>();

		//return a copy of recognizedSentences
		return recognizedSentences;
	}

}

#endif
