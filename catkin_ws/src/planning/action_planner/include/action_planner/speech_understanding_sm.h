#include "action_planner/primitives_tasks.h"
#include "action_planner/service_manager.h"
#include "action_planner/states_machines.h"
#include "ros/ros.h"
#include <iostream>
#include <vector>

class SpeechUnderstandingSM
{
private:
	/*
	*	ADD THE STATES ID YOU NEED FOR EACH STATE FUNCTION (IF YOU WANT)
	*/
	enum States
	{
		InitialState,
		WaitForInitCommand,
		ProcessAudioFiles,
		ListenAndParse,
		ReportResult,
		FinalState
	};

	//for the SM api
	StatesMachines SM;
	//service manager
	static ServiceManager srv_man;
	//stpln
	static PrimitivesTasks m_tasks;
	//state func members
	static int listenAttempt;
	static std::string cfrRepresentation;
	static std::string recoSentence;
	static bool audioProcesed;

	/********************************************************************/
	/*
	*	ADD THE STATE FUNCTIONS YOU NEED
	*/
	static int initialState();
	static int waitForInitCommand();
	static int processAudioFiles();
	static int listenAndParse();
	static int reportResult();
	static int finalState();
	/**********************************************************************/
	
public:
	SpeechUnderstandingSM(PrimitivesTasks tasks);
	bool execute();
};

PrimitivesTasks SpeechUnderstandingSM::m_tasks;
ServiceManager SpeechUnderstandingSM::srv_man;
int SpeechUnderstandingSM::listenAttempt;
std::string SpeechUnderstandingSM::cfrRepresentation;
std::string SpeechUnderstandingSM::recoSentence;
bool SpeechUnderstandingSM::audioProcesed;


/*
* A particular constructor for your state machine
* Initialize your state machine here (add states, define the final state, define the execution method, etc)
*/
SpeechUnderstandingSM::SpeechUnderstandingSM(PrimitivesTasks tasks)
{
	m_tasks = tasks;
	//int (SpeechUnderstandingSM::*
	//add states to the state machine
	SM.addState((int)InitialState, &initialState);
	SM.addState((int)WaitForInitCommand, &waitForInitCommand);
	SM.addState((int)ProcessAudioFiles, &processAudioFiles);
	SM.addState((int)ListenAndParse, &listenAndParse);
	SM.addState((int)ReportResult, &reportResult);;
	SM.addState((int)FinalState, &finalState, true);
}
bool SpeechUnderstandingSM::execute()
{
	while(SM.runNextStep());
	return true;
};

/*
*	ADD THE STATE FUNCTIONS YOU NEED
*/
int SpeechUnderstandingSM::initialState()
{
	std::cout << "executing initial state " << std::endl;

	/*Initialize member variables*/
	listenAttempt=0;

	return (int)WaitForInitCommand;
}

int SpeechUnderstandingSM::waitForInitCommand()
{
	std::cout << "waiting for init command....." << std::endl;
	std::getchar();

	if(!audioProcesed)
		return (int)ProcessAudioFiles;
	else
		return (int)ListenAndParse;
}

int SpeechUnderstandingSM::processAudioFiles()
{
	std::cout << "processing audio files... " << std::endl;
	std::cout << "send instruction to BB module " << std::endl;
	audioProcesed = true;
	return (int)ListenAndParse;
}

int SpeechUnderstandingSM::listenAndParse()
{
	std::cout << "waiting for speech command... " << std::endl;
	recoSentence = "BAD_RECOGNITION";
	cfrRepresentation = "NO_INTERPRETATION";
	if(m_tasks.listen(recoSentence, 10000))
	{
		std::cout << "parsing command... " << std::endl;
		if(!srv_man.langundProcess(recoSentence, cfrRepresentation))
			cfrRepresentation = "NO_INTERPRETATION";
	}
	listenAttempt++;
	//else
	//{
	//	//nothing heared, try again
	//	srv_man.spgenSay("Human I cannot hear you. Please get closer to my microphone", 5000);
	//	if(listenAttempts<maxListenAttempts)
	//	{
	//		listenAttempts++;
	//		return (int)ListenAndParse;
	//	}
	//	return (int)ReportResult;
	//}
	
	//ros::Duration(0.5).sleep();
	return (int)ReportResult;
}

int SpeechUnderstandingSM::reportResult()
{
	std::cout << "send the information to the bb module" << std::endl;
	std::cout << "fb_mic_phase_speech_audio_" << listenAttempt << ".wav|" << recoSentence << "|" << cfrRepresentation << std::endl;
	//return (int)FinalState;
	return (int)WaitForInitCommand;
}

int SpeechUnderstandingSM::finalState()
{
	std::cout << "finalState reached" << std::endl;
	return (int)FinalState;
}

/**********************************************************************/
