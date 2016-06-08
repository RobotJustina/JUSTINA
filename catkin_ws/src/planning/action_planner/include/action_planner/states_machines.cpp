#include "action_planner/states_machines.h"

/*
* Constructor
*/
StatesMachines::StatesMachines()
{
	//currentSMStatus = SM_STATUS::Initializing;
	//statesDictionary[0] = defaultState;
	stateToExecute = -1;
	finalStateExecuted = false;
}

/*
* Default state defined when the programmer does not define
*/
int StatesMachines::defaultState()
{
	std::cout << "INITIAL STATE NOT DEFINED YET" << std::endl;
}

/*
* Add a state to the States list
* Receives:
*	stateID: the int ID of the state to add
*	stateFunction: a pointer of the function to execute for the stateID state
*	isFinalState: an optional boolean that indicates if the state is a final state(true) or not (false)
*/
void StatesMachines::addState(int stateID, fncPtr stateFunction, bool isFinalState)
{
	stateToExecute = (stateID==0) ? 0:stateToExecute;
	//add the specified state to the states dictionary
	statesDictionary[stateID] = stateFunction;

	//update the final state id 
	finalStateID = (isFinalState) ? stateID : finalStateID;
}

/*
* Clears the list of states added to the SM
*/
void StatesMachines::resetStates()
{
	//currentSMStatus = SM_STATUS::Initializing;
	finalStateExecuted = false;
	statesDictionary.clear();
	stateToExecute = -1;
}

/*
* Executes scheduled state of the state machine and schedules the next state to execute.
* Schedules the initial state (id 0) at the beginning and when the final state was executed.
*Returns:
*	true if the final state wasn't n'texecuted, else otherwise
*/
bool StatesMachines::runNextStep()
{
	if(stateToExecute == -1)
	{
		std::cout << "NO STATES ADDED YET" << std::endl;
		return false;
	}	
	if(stateToExecute==finalStateID)
		finalStateExecuted = true;

	stateToExecute = statesDictionary[stateToExecute]();
	
	if(stateToExecute==finalStateID && finalStateExecuted)
	{
		stateToExecute = 0;
		return false;
	}

	return true;
}
