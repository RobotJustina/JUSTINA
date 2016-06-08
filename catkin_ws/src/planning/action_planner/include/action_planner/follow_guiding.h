#include "action_planner/primitives_tasks.h"
#include "action_planner/states_machines.h"
#include "ros/ros.h"
#include <iostream>

class FollowGuidingSM
{
public:
	/*
	*	ADD THE STATES ID YOU NEED FOR EACH STATE FUNCTION (IF YOU WANT)
	*/
	enum States
	{
		InitialState,
		WaitForInitCommand,
		LookForObjects,
		ReportResult,
		FinalState
	};

	//for the SM api
	StatesMachines SM;
	//stpln
	PrimitivesTasks m_tasks;
	/********************************************************************/
	
	/*
	*	ADD THE STATE FUNCTIONS YOU NEED
	*/
	static int initialState()
	{
		std::cout << "executing initial state" << std::endl;
		return (int)WaitForInitCommand;
	}
	
	static int waitForInitCommand()
	{
		std::cout << "waiting for init command....." << std::endl;
		return (int)LookForObjects;
	}
	
	static int lookForObjects()
	{
		std::cout << "looking for objects" << std::endl;
		return (int)ReportResult;
	}

	static int reportResult()
	{
		std::cout << "Repoorting results" << std::endl;
		return (int)FinalState;
	}

	static int finalState()
	{
		std::cout << "finalState reached" << std::endl;
		return (int)FinalState;
	}
	
	/**********************************************************************/
	
	/*
	* A particular constructor for your state machine
	* Initialize your state machine here (add states, define the final state, define the execution method, etc)
	*/
	FollowGuidingSM(PrimitivesTasks tasks)
	{
		m_tasks = tasks;
		//add states to the state machine
		SM.addState((int)InitialState, &initialState);
		SM.addState((int)WaitForInitCommand, &waitForInitCommand);
		SM.addState((int)LookForObjects, &lookForObjects);
		SM.addState((int)ReportResult, &reportResult);;
		SM.addState((int)FinalState, &finalState, true);
	
		//execute the state machine from the initial state until the final state
		//while(SM.runNextStep());
	
		//return true;
	}
	bool execute()
	{
		while(SM.runNextStep());
		return true;
	}
};
