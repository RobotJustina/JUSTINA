#ifndef ACT_PLN_SMS
#define ACT_PLN_SMS

#include <map>
#include <iostream>
class StatesMachines
{
public:
	/*
	* Constructor
	*/
	StatesMachines();

	typedef int (*fncPtr)();
	
	void resetStates();
	void addState(int, fncPtr, bool isFinalState=false);
	bool runNextStep();
private:
	//dictionary to store the states of the sm to execute
	std::map<int, fncPtr> statesDictionary;
	//variable to store the final state id
	int finalStateID;
	//variable to store the next state to execute
	int stateToExecute;
	//variable to know if the final state was executed
	bool finalStateExecuted;

	int defaultState();
};
#endif
