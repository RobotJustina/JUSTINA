#include <map>
#include <sstream>
#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaAudio.h"

//*******************************************************************
//
// Name of designated location for the test
//
//*******************************************************************
#define SPOT "livingroom"


enum State{
	SM_INIT,
	SM_WAIT_DOOR,
	SM_NAVIGATE_SPOT,
	SM_NAVIGATE_SPOT_WAIT,
	SM_QUESTION_P1,
	SM_QUESTION_P2,
	SM_QUESTION_P2R,
	SM_TURN,
	SM_TURN_WAIT,
	SM_FINAL_STATE = -1
};
typedef std::map <std::string, std::string> QMap;
QMap questions;
std::vector<std::string> questionList;

/**
* Loads all the questions
*/
void fillQuestions();

/**
* Checks if the detected speech is stored as a question. If such is the case
* returns the answer. Otherwise, returns a default sring stating it could not
* understand the question.
* @param lastRecoSpeech The string detected by the speech recognition
*/
bool getAnswer(const std::string& lastRecoSpeech, std::string& answer) {
	// Check if the question is on the list.
	if( questions.count(lastRecoSpeech) ){
		// If the question is found, return it
		answer = questions[lastRecoSpeech];
		return true;
	}
	// Otherwise, provide a default one
	answer = std::string("I did not understand the question");
	return false;
}

bool listenAndAnswer(const int& timeout){
	std::string answer;
	std::string lastRecoSpeech;
	

	if(!JustinaHRI::waitForSpecificSentence(questionList, lastRecoSpeech, timeout))
		return false;
	if(! getAnswer(lastRecoSpeech, answer) )
		return false;
	JustinaHRI::say(answer);
	return true;
}

bool listenTurnAndAnswer(const int& timeout, ros::Rate& loop){
	float audioSourceAngle = 0;
	std::string answer;
	std::string lastRecoSpeech;
	
	loop.sleep();

	std::cout << "Starting audio source detection" << std::endl;
	JustinaAudio::startSimpleAudioSource();
	ros::spinOnce();

	bool understood = JustinaHRI::waitForSpecificSentence(questionList, lastRecoSpeech, timeout);
	audioSourceAngle = JustinaAudio::getAudioSource();
	std::cout << "Audio source at" << (180 * audioSourceAngle / 3.141592) << "degrees" << std::endl;
	JustinaHRI::say("Wait while I turn and look at you.");
	JustinaNavigation::moveDistAngle(0, (double) audioSourceAngle, 5000);
	if(!understood || !getAnswer(lastRecoSpeech, answer) )
		return false;
	JustinaHRI::say(answer);
	return true;
}

int main(int argc, char** argv)
{
	std::cout << "Initializing Speech Recognition & Audio Test..." << std::endl;
	ros::init(argc, argv, "act_pln");
	ros::NodeHandle n;
	JustinaHardware::setNodeHandle(&n);
	JustinaHRI::setNodeHandle(&n);
	JustinaManip::setNodeHandle(&n);
	JustinaNavigation::setNodeHandle(&n);
	JustinaTools::setNodeHandle(&n);
	JustinaAudio::setNodeHandle(&n);
	// JustinaVision::setNodeHandle(&n);
	ros::Rate loop(10);

	int sleepAudioCaptureDelay = 4; 
	double turnAngle = 0;
	int numQuestion;
	State nextState = SM_INIT;
	bool fail = false;
	bool success = false;
	std::string answer;
	std::stringstream ss; // String buffer for spoken mssages.
	fillQuestions();

	while(ros::ok() && !fail && !success)
	{
		ros::Rate loop(sleepAudioCaptureDelay);

		switch(nextState)
		{
			// Initial state:
			//			  It enters navigation position and states robot is ready
			// Next State:  Wait for door to be open
			case SM_INIT:
				//JustinaManip::laGoTo("navigation", 10000);
				//JustinaManip::raGoTo("navigation", 10000);
				JustinaHRI::say("I'm ready for the speech-recognition and audio-detection test");
				nextState = SM_WAIT_DOOR;
				break;

			// State:	   SM_WAIT_DOOR
			//			  It waits for the door to be opened
			// Next state:  SM_WAIT_DOOR | SM_NAVIGATE_SPOT
			case SM_WAIT_DOOR:
				// if(JustinaX::isDoorOpen())
				if(true)
				{
					nextState = SM_NAVIGATE_SPOT_WAIT;
					numQuestion = 1; 
				}
				break;

			// State:	   SM_NAVIGATE_SPOT | Navigate to designated location
			//			  Command the robot to start going to designated location
			// Next state:  SM_NAVIGATE_SPOT_WAIT
			case SM_NAVIGATE_SPOT:
				//JustinaNavigation::startGetClose(SPOT);
				//JustinaNavigation::getClose(SPOT, 120000);
				break;

			// State:	   SM_NAVIGATE_SPOT_WAIT
			//			  Loops in this state until the designated location is reached
			//			  and sets up next state.
			// Next state:  SM_NAVIGATE_SPOT_WAIT | SM_QUESTION_P1
			case SM_NAVIGATE_SPOT_WAIT:
				//if(JustinaNavigation::isGoalReached()){
					nextState = SM_QUESTION_P1;
					numQuestion = 1;
					JustinaHRI::say("I'm ready for question one.");
				//}
				break;

			// State:	   SM_QUESTION_P1
			//			  Listen to 5 question and answers them
			//				  1. Waits 20 secs for a question to arrive
			//				  2. Answers a question when detected, or informs it has been skipped.
			//				  3. When 5 questions have reached, jumps to Phase 2
			// Next state:  SM_QUESTION_P1 | SM_QUESTION_P2
			case SM_QUESTION_P1:
				ss.str(std::string()); // Clear the buffer
				if( !listenAndAnswer(10000) )
					ss << "I did not understood the question. ";
				if(++numQuestion < 6){
					ss << "Lets proceed with question " << numQuestion;
					nextState = SM_QUESTION_P1;
				}
				else{
					ss << "Lets proceed with the test";
					numQuestion = 1;
					nextState = SM_QUESTION_P2;
				}
				ss << ".";
				JustinaHRI::say(ss.str());
				break;

			// State:	   SM_QUESTION_P2
			//			  Listen to 5 question and answers them
			//				  1. Waits 15 secs for a question to arrive
			//				  2. Answers a question when detected.
			//				  3. When time is out, jumps to SM_QUESTION_P2R
			//				  4. When 5 questions have reached, jumps to FINAL_STATE
			// Next state:  SM_QUESTION_P2 | SM_QUESTION_P2R | SM_FINAL_STATE
			case SM_QUESTION_P2:
				ss.str(std::string()); // Clear the buffer
				if( listenTurnAndAnswer(8000, loop) )
				{
					if(++numQuestion < 6){
						ss << "Lets proceed with question " << numQuestion;
						nextState = SM_QUESTION_P2;
					}
					else{
						ss << "I will answer no more questions. Thank you!";
						nextState = SM_FINAL_STATE;
					}
				}
				else{
					ss << "I did not hear you. Please repeat question ";
					ss << numQuestion;
					nextState = SM_QUESTION_P2R;
				}
				ss << ".";
				JustinaHRI::say(ss.str());
				sleepAudioCaptureDelay = 4;
				break;

			// State:	   SM_QUESTION_P2R
			//			  Ask a question to be repeated, then waits for the question and answers it
			//				  1. Waits 15 secs for a question to arrive
			//				  2. Answers a question when detected.
			//				  3. When time is out, apologize and jumps to SM_QUESTION_P2 or...
			//				  4. if 5 questions have reached, jumps to FINAL_STATE
			// Next state:  SM_QUESTION_P2 | SM_QUESTION_P2R | SM_FINAL_STATE
			case SM_QUESTION_P2R:
				ss.str(std::string()); // Clear the buffer
				if( !listenAndAnswer(8000) )
					ss << "I did not understood the question. ";
				if(++numQuestion < 6){
					ss << "Lets proceed with question " << numQuestion;
					nextState = SM_QUESTION_P2;
				}
				else{
					ss << "I have finished the test";
					nextState = SM_FINAL_STATE;
				}
				ss << ".";
				JustinaHRI::say(ss.str());
				break;

			// case SM_TURN:
			//	 JustinaNavigation::startMoveDistAngle(0, turnAngle);
			//	 nextState = SM_TURN_WAIT;
			//	 break;

			// case SM_TURN_WAIT:
			//	 if(!JustinaNavigation::isGoalReached()){
			//		 turnAngle = 0;
			//		 nextState = SM_QUESTION_P2;
			//	 }
			//	 break;

			case SM_FINAL_STATE:
				success = false;
				break;
		}
		ros::spinOnce();
		loop.sleep();
	}

	return 0;
}



void fillQuestions()
{
	// Reads from the file 'predefined_questions.csv' each line, splits at the
	// comma, stores the left part as question and the right part as answer.
	// Commented since I don't know if such file can be read.
	/*
	std::ifstream pqfile("predefined_questions.csv");
	std::string line;

	while (std::getline(pqfile, line))
	{
		std::istringstream ss(line);
		std::string question;
		std::string answer;
		std::getline(ss, question, ',');
		std::getline(ss, answer);
		questionList.push_back(question);
		questions[question] = answer;
	}
	*/

	questionList.push_back("Who are the inventors of the C programming language?");
	questions["Who are the inventors of the C programming language?"] = "Ken Thompson and Dennis Ritchie";

	questionList.push_back("Who is the inventor of the Python programming language?");
	questions["Who is the inventor of the Python programming language?"] = "Guido fan Rho sum";

	questionList.push_back("Which robot was the star in the movie Wall-E?");
	questions["Which robot was the star in the movie Wall-E?"] = "I would like to say mop, but it was Wall-E";

	questionList.push_back("Where does the term computer bug come from?");
	questions["Where does the term computer bug come from?"] = "From a moth trapped in a relay";

	questionList.push_back("What is the name of the round robot in the new Star Wars movie?");
	questions["What is the name of the round robot in the new Star Wars movie?"] = "Bee bee eight";

	questionList.push_back("How many curry sausages are eaten in Germany each year?");
	questions["How many curry sausages are eaten in Germany each year?"] = "About 800 million currywurst every year";

	questionList.push_back("Who is president of the galaxy in The Hitchhiker's Guide to the Galaxy?");
	questions["Who is president of the galaxy in The Hitchhiker's Guide to the Galaxy?"] = "Zaphod Beeblebrox";

	questionList.push_back("Which robot is the love interest in Wall-E?");
	questions["Which robot is the love interest in Wall-E?"] = "That robot is EVE";

	questionList.push_back("Which company makes ASIMO?");
	questions["Which company makes ASIMO?"] = "ASIMO is made by Honda";

	questionList.push_back("What company makes Big Dog?");
	questions["What company makes Big Dog?"] = "Big Dog was created by Boston Dynamics";

	questionList.push_back("What is the funny clumsy character of the Star Wars prequals?");
	questions["What is the funny clumsy character of the Star Wars prequals?"] = "Jar-Jar Binks. By the way, I hate him.";

	questionList.push_back("How many people live in the Germany?");
	questions["How many people live in the Germany?"] = "A little over 80 million";

	questionList.push_back("What are the colours of the German flag?");
	questions["What are the colours of the German flag?"] = "Black red and yellow";

	questionList.push_back("What city is the capital of the Germany?");
	questions["What city is the capital of the Germany?"] = "The capital of germany is Berlin";

	questionList.push_back("How many arms do you have?");
	questions["How many arms do you have?"] = "I have two arms";

	questionList.push_back("What is the heaviest element?");
	questions["What is the heaviest element?"] = "Plutonium when measured by the mass of the element but Osmium is densest";

	questionList.push_back("What did Alan Turing create?");
	questions["What did Alan Turing create?"] = "Many things like Turing machines and the Turing test";

	questionList.push_back("Who is the helicopter pilot in the A-Team?");
	questions["Who is the helicopter pilot in the A-Team?"] = "Captain Howling Mad Murdock";

	questionList.push_back("What Apollo was the last to land on the moon?");
	questions["What Apollo was the last to land on the moon?"] = "The last Apollo is Apollo 17";

	questionList.push_back("Who was the last man to step on the moon?");
	questions["Who was the last man to step on the moon?"] = "The mas is Gene Cernan";

	questionList.push_back("In which county is the play of Hamlet set?");
	questions["In which county is the play of Hamlet set?"] = "The Hamlet set is played in Denmark";

	questionList.push_back("What are names of Donald Duck's nephews?");
	questions["What are names of Donald Duck's nephews?"] = "Donald Duck's nephews are Huey Dewey and Louie Duck";

	questionList.push_back("How many metres are in a mile?");
	questions["How many metres are in a mile?"] = "About 1609 metres";

	questionList.push_back("Name a dragon in The Lord of the Rings?");
	questions["Name a dragon in The Lord of the Rings?"] = "There are no dragons in The Lord of the Rings. In The Hobbit, there is Smaug";

	questionList.push_back("Who is the Chancellor of Germany?");
	questions["Who is the Chancellor of Germany?"] = "Angela Merkel";

	questionList.push_back("Who developed the first industrial robot?");
	questions["Who developed the first industrial robot?"] = "The American physicist Joseph Engelberg. He is also considered the father of robotics.";

	questionList.push_back("What's the difference between a cyborg and an android?");
	questions["What's the difference between a cyborg and an android?"] = "Cyborgs are biological being with electromechanical enhancements. Androids are human-shaped robots.";

	questionList.push_back("Do you know any cyborg?");
	questions["Do you know any cyborg?"] = "Professor Kevin Warwick. He implanted a chip in in his left arm to remotely operate doors an artificial hand and an electronic wheelchair.";

	questionList.push_back("In which city is this year's RoboCup hosted?");
	questions["In which city is this year's RoboCup hosted?"] = "In Leipzig, Germany.";

	questionList.push_back("Which city hosted last year's RoboCup?");
	questions["Which city hosted last year's RoboCup?"] = "In Hefei, China.";

	questionList.push_back("In which city will next year's RoboCup be hosted?");
	questions["In which city will next year's RoboCup be hosted?"] = "It hasn't been announced yet.";

	questionList.push_back("Name the main rivers surrounding Leipzig");
	questions["Name the main rivers surrounding Leipzig"] = "The Parthe Pleisse and the White Elster.";

	questionList.push_back("What is the Cospudener See?");
	questions["What is the Cospudener See?"] = "The Cospudener See is a lake situated south of Leipzig on the site of a former open cast mine.";

	questionList.push_back("Where started the peaceful revolution of 1989?");
	questions["Where started the peaceful revolution of 1989?"] = "The peaceful revolution started in September 4 1989 in Leipzig at the St. Nicholas Church.";

	questionList.push_back("Where is the world's oldest trade fair hosted?");
	questions["Where is the world's oldest trade fair hosted?"] = "The world's oldest trade fair is in Leipzig.";

	questionList.push_back("Where is one of the world's largest dark music festivals hosted?");
	questions["Where is one of the world's largest dark music festivals hosted?"] = "Leipzig hosts one of the world's largest dark music festivals.";

	questionList.push_back("Where is Europe's oldest continuous coffee shop hosted?");
	questions["Where is Europe's oldest continuous coffee shop hosted?"] = "Europe's oldest continuous coffee shop is in Leipzig.";

	questionList.push_back("Name one of the greatest German composers");
	questions["Name one of the greatest German composers"] = "Johann Sebastian Bach.";

	questionList.push_back("Where is Johann Sebastian Bach buried?");
	questions["Where is Johann Sebastian Bach buried?"] = "Johann Sebastian Bach is buried in St. Thomas' Church here in Leipzig.";

	questionList.push_back("Do you have dreams?");
	questions["Do you have dreams?"] = "I dream of Electric Sheeps.";

	questionList.push_back("Hey what's up?");
	questions["Hey what's up?"] = "I don't know since I've never been there.";

	questionList.push_back("There are seven days in a week. True or false?");
	questions["There are seven days in a week. True or false?"] = "True. There are seven days in a week.";

	questionList.push_back("There are eleven days in a week. True or false?");
	questions["There are eleven days in a week. True or false?"] = "False. There are seven days in a week not eleven.";

	questionList.push_back("January has 31 days. True or false?");
	questions["January has 31 days. True or false?"] = "True. January has 31 days.";

	questionList.push_back("January has 28 days. True or false?");
	questions["January has 28 days. True or false?"] = "False. January has 31 days not 28.";

	questionList.push_back("February has 28 days. True or false?");
	questions["February has 28 days. True or false?"] = "True, but in leap-years has 29.";

	questionList.push_back("February has 31 days. True or false?");
	questions["February has 31 days. True or false?"] = "False. February has either 28 or 29 days. Depend on the year.";

	questionList.push_back("Do you have dreams?");
	questions["Do you have dreams?"] = "I dream of Electric Sheep.";

	questionList.push_back("Who used first the word Robot?");
	questions["Who used first the word Robot?"] = "The word robot was first used by Czech writer Karel Capek.";

	questionList.push_back("What origin has the word Robot?");
	questions["What origin has the word Robot?"] = "The Czech word robota that means forced work or labour";
}