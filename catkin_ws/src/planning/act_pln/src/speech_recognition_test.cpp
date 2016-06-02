#include <map>
#include <sstream>
#include <iostream>
#include "ros/ros.h"
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaVision.h"

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


void fillQuestions(std::vector<std::string>& questionList){
    // questions["what is the capital of this state?"] = "The capital of this state is Victoria city";
    // questions["what is the color of the sky?"] = "The color of the sky is blue";
    // questions["who discovered America?"] = "America was discovered by Christopher Colombus";
    // questions["how are you doing today?"] = "Fine, thank you";
    // questions["when did you have your last maintenance?"] = "My last maintenance was this morning";
    // questions["how many rooms does this arena have?"] = "This arena has four rooms";
    // questions["did you fancy the other robot?"] = "I am not ready for a commitment";
    // questions["who is the best soccer player ever?"] = "The best soccer player is Pele";
    // questions["how do you like your coffee?"] = "I do not drink coffee";
    // questions["are you feeling warm in this room?"] = "yes, very much";
    // questions["what is your favorite movie?"] = "My favorite movie is blade runner";
    // questions["how tall are you?"] = "My height is one point forty five meters";
    // questions["What is your favorite science fiction novel?"] = "My favorite science fiction novel is do androids dreaming of electric-sheep";
    // questions["where do you come from?"] = "I come from Puebla";
    // questions["would you like to drive your own car?"] = "I don't have a drivers license";
    // 
    // for( QMap::iterator it = questions.begin(); it != questions.end(); ++it ) {
    //     questionList.push_back( it->first );
    // }

    questionList.push_back("what is the capital of this state?");
    questionList.push_back("what is the color of the sky?");
    questionList.push_back("who discovered America?");
    questionList.push_back("how are you doing today?");
    questionList.push_back("when did you have your last maintanance?");
    questionList.push_back("how many rooms does this arena have?");
    questionList.push_back("did you fancy the other robot");
    questionList.push_back("who is the best soccer player ever?");
    questionList.push_back("how do you like your coffee");
    questionList.push_back("are you feeling warm in this room?");
    questionList.push_back("what is your favarite movie");
    questionList.push_back("how tall are you?");
    questionList.push_back("what is your favorite science fiction novel");
    questionList.push_back("where do you come from?");
    questionList.push_back("would you like to drive your own car?");

    questions["capital"] = "The capital of this state is Victoria city";
    questions["sky"] = "The color of the sky is blue";
    questions["discovered"] = "America was discovered by Christopher Colombus";
    questions["today"] = "Fine, thank you";
    questions["last"] = "My last maintenance was this morning";
    questions["arena"] = "This arena has four rooms";
    questions["fancy"] = "I am not ready for a commitment";
    questions["soccer"] = "The best soccer player is Pele";
    questions["coffee"] = "I do not drink coffee";
    questions["warm"] = "yes, very much";
    questions["movie"] = "My favorite movie is blade runner";
    questions["tall"] = "My height is one point forty five meters";
    questions["novel"] = "My favorite science fiction novel is do androids dreaming of electric-sheep";
    questions["come"] = "I come from Puebla";
    questions["car"] = "I don't have a drivers license";
}

std::string getAnswer(const std::string& lastRecoSpeech){
    // return questions[lastRecoSpeech];

    for( QMap::iterator it = questions.begin(); it != questions.end(); ++it ) {
        std::size_t found = lastRecoSpeech.find(it->first);
        if (found!=std::string::npos)
            return questions[it->first];
    }
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
    // JustinaVision::setNodeHandle(&n);
    ros::Rate loop(10);

    double turnAngle = 0;
    int numQuestion;
    State nextState = SM_INIT;
    bool fail = false;
    bool success = false;
    std::string lastRecoSpeech;
    std::vector<std::string> questionList;
    std::stringstream ss; // String buffer for spoken mssages.
    fillQuestions(questionList);

    while(ros::ok() && !fail && !success)
    {
        switch(nextState)
        {
            // Initial state:
            //              It enters navigation position and states robot is ready
            // Next State:  Wait for door to be open
            case SM_INIT:
                //JustinaManip::laGoTo("navigation", 10000);
                //JustinaManip::raGoTo("navigation", 10000);
                JustinaHRI::say("I'm ready for the speech-recognition and audio-detection test");
                nextState = SM_WAIT_DOOR;
                break;

            // State:       SM_WAIT_DOOR
            //              It waits for the door to be opened
            // Next state:  SM_WAIT_DOOR | SM_NAVIGATE_SPOT
            case SM_WAIT_DOOR:
                // if(JustinaX::isDoorOpen())
                if(true)
                {
                    nextState = SM_QUESTION_P1;
                    numQuestion = 1; 
                }
                break;

            // State:       SM_NAVIGATE_SPOT | Navigate to designated location
            //              Command the robot to start going to designated location
            // Next state:  SM_NAVIGATE_SPOT_WAIT
            case SM_NAVIGATE_SPOT:
                //JustinaNavigation::startGetClose(SPOT);
                //JustinaNavigation::getClose(SPOT, 120000);
                break;

            // State:       SM_NAVIGATE_SPOT_WAIT
            //              Loops in this state until the designated location is reached
            //              and sets up next state.
            // Next state:  SM_NAVIGATE_SPOT_WAIT | SM_QUESTION_P1
            case SM_NAVIGATE_SPOT_WAIT:
                //if(JustinaNavigation::isGoalReached()){
                    nextState = SM_QUESTION_P1;
                    numQuestion = 1;
                    JustinaHRI::say("I'm ready for question one.");
                //}
                break;

            // State:       SM_QUESTION_P1
            //              Listen to 5 question and answers them
            //                  1. Waits 20 secs for a question to arrive
            //                  2. Answers a question when detected, or informs it has been skipped.
            //                  3. When 5 questions have reached, jumps to Phase 2
            // Next state:  SM_QUESTION_P1 | SM_QUESTION_P2
            case SM_QUESTION_P1:
                ss.str(std::string()); // Clear the buffer
                if(JustinaHRI::waitForSpecificSentence(questionList, lastRecoSpeech, 20000)){
                    JustinaHRI::say(getAnswer(lastRecoSpeech));
                }
                else
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

            // State:       SM_QUESTION_P2
            //              Listen to 5 question and answers them
            //                  1. Waits 15 secs for a question to arrive
            //                  2. Answers a question when detected.
            //                  3. When time is out, jumps to SM_QUESTION_P2R
            //                  4. When 5 questions have reached, jumps to FINAL_STATE
            // Next state:  SM_QUESTION_P2 | SM_QUESTION_P2R | SM_FINAL_STATE
            case SM_QUESTION_P2:
                ss.str(std::string()); // Clear the buffer
                if(JustinaHRI::waitForSpecificSentence(questionList, lastRecoSpeech, 15000)){
                    JustinaHRI::say(getAnswer(lastRecoSpeech));
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
                break;

            // State:       SM_QUESTION_P2R
            //              Ask a question to be repeated, then waits for the question and answers it
            //                  1. Waits 15 secs for a question to arrive
            //                  2. Answers a question when detected.
            //                  3. When time is out, apologize and jumps to SM_QUESTION_P2 or...
            //                  4. if 5 questions have reached, jumps to FINAL_STATE
            // Next state:  SM_QUESTION_P2 | SM_QUESTION_P2R | SM_FINAL_STATE
            case SM_QUESTION_P2R:
                ss.str(std::string()); // Clear the buffer
                if(JustinaHRI::waitForSpecificSentence(questionList, lastRecoSpeech, 15000)){
                    JustinaHRI::say(getAnswer(lastRecoSpeech));
                }
                else
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
            //     JustinaNavigation::startMoveDistAngle(0, turnAngle);
            //     nextState = SM_TURN_WAIT;
            //     break;

            // case SM_TURN_WAIT:
            //     if(!JustinaNavigation::isGoalReached()){
            //         turnAngle = 0;
            //         nextState = SM_QUESTION_P2;
            //     }
            //     break;

            case SM_FINAL_STATE:
                success = false;
                break;
        }
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}
