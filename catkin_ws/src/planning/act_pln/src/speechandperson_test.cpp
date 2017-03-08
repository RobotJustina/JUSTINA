#include <iostream>
#include <stdlib.h>
#include "ros/ros.h"
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"
#include "std_msgs/Bool.h"
#include "string"

#define SM_InitialState 0
#define	SM_WaitingandTurn 10
#define SM_StatingtheCrowd 20
#define SM_RequestingOperator 30
#define	SM_RiddleGame 40
#define	SM_FinalState 50


std::string personName = "operator";


void fillQuestions();

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

std::vector<vision_msgs::VisionFaceObject> recognizeAllFaces(float timeOut, bool &recognized)
{
		JustinaVision::startFaceRecognition();
		recognized = false;
		boost::posix_time::ptime curr;
		boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
		boost::posix_time::time_duration diff;
		std::vector<vision_msgs::VisionFaceObject> lastRecognizedFaces;

		do{
			boost::this_thread::sleep(boost::posix_time::milliseconds(100));
			JustinaVision::facRecognize();
			JustinaVision::getLastRecognizedFaces(lastRecognizedFaces);
			curr = boost::posix_time::second_clock::local_time();
			ros::spinOnce();
		}while(ros::ok() && (curr - prev).total_milliseconds()< timeOut);



		if(lastRecognizedFaces.size()>0)
			recognized = true;
		else
			recognized = false;

		std::cout << "recognized:" << recognized << std::endl;
		return lastRecognizedFaces;
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
    JustinaVision::setNodeHandle(&n);
    ros::Rate loop(10);

    
    int nextState = 0;
    bool recog=false;
    int numQuestion = 1;
    std::string answer;
	std::stringstream ss;
    
	
	std::stringstream contW;
	std::stringstream contM;
	std::stringstream contU;
	std::stringstream profPlace;
	std::stringstream genderOperator;
	std::stringstream contC;
	int mIndex=0;
	int women=0;
	int men=0;
	int unknown=0;
	int genero=10;
	int contCrowd=0;
	
	//vector para almacenar los rostros encontrados
	std::vector<vision_msgs::VisionFaceObject> dFaces;

	fillQuestions();
    

    while(ros::ok() && !fail && !success)
    {
        switch(nextState)
        {

        case SM_InitialState:
        	std::cout << "start the speech and person recognition test" << std::endl;
        	JustinaHardware::setHeadGoalPose(0.0, 0.0);
        	JustinaHRI::say("I'm ready for the speech and person recognition and test");
        	ros::Duration(2.0).sleep();
        	JustinaHRI::say("I want to play a riddle game");
        	ros::Duration(12.0).sleep();
            nextState = SM_FindCrowd;

        break;

        case SM_WaitingandTurn:
        	std::cout << "finding the crowd" << std::endl;
        	JustinaHRI::say("I'm turnning around to find the crowd");
        	JustinaHardware::setHeadGoalPose(0.0, -0.2)
        	JustinaNavigation::moveDistAngle(0.0, 3.141592, 80000);
        	ros::Duration(1.0).sleep();
        	nextState = SM_StatingtheCrowd;
        break;

        case SM_StatingtheCrowd:
        	std::cout << "requesting operator" << std::endl;
        	JustinaHRI::say("Please do not move, I'm going to state the size of the crowd");
        	while(!recog)
			{
				dFaces = recognizeAllFaces(10000,recog);
				JustinaVision::stopFaceRecognition();
			}

			std::cout <<"tamaño de arreglo " << dFaces.size() <<std::endl;

			for(int i=0; i<dFaces.size(); i++)
			{
				if(dFaces[i].gender==0)
					women++;
				if(dFaces[i].gender==1)
					men++;
				if(dFaces[i].gender==2)
					unknown++;
	
				std::cout<<"hombres: "<< men << std::endl;
			}
			std::cout <<"Reporting results" << std::endl;
		
		
			
			contCrowd=women+men+unknown;
			contC << "the size of the crowd is " <<contCrowd << std::endl;

	
			contW << "There are " << women << " women";
			contM << "There are " << men << " men";
			contU << "There are " << unknown << " people with unknown genre";

			JustinaHRI::say("I am going to describe the crowd ");
			JustinaHRI::say(contC.str());
			JustinaHRI::say(contW.str());
			JustinaHRI::say(contM.str());
			JustinaHRI::say(contU.str());
	
			ros::Duration(2.0).sleep();
			nextState = SM_RequestingOperator;
        break;

        case SM_RequestingOperator:
			std::cout <<"Requesting Operator" << std::endl;    
			JustinaHRI::say("Who want to play riddles with me?");
			ros::Duration(2.0).sleep();
			JustinaHRI::say("Please, put in front of me and tell me your questions");
			ros::Duration(2.0).sleep();
            nextState = SM_RiddleGame;


        break;

        case SM_RiddleGame:
			ss.str(std::string()); // Clear the buffer
			if( !listenAndAnswer(10000) )
				ss << "I did not understand the question. ";
			if(++numQuestion < 6){
				ss << "Lets proceed with question " << numQuestion;
				nextState = SM_RiddleGame;
			}
			else{
				ss << "Lets proceed with the test";
				//numQuestion = 1;
				nextState = SM_FinalState;
			}
			ss << ".";
			JustinaHRI::say(ss.str());
		break;

		case SM_FinalState:
			std::cout <<"finalState reached" << std::endl;
			JustinaHRI::say("I have finished the speech and person recognition test...");
			success=true;
		break;

    	}
    	ros::spinOnce();
        loop.sleep();
    }
    return 0;
}

void fillQuestions()
{

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