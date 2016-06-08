#include "simple_task_planner/simpletasks.h"

SimpleTasks::SimpleTasks()
{
    //initialize the arms modules
    m_rightArmTasks = new RobotArmTasks(); 
    m_leftArmTasks = new RobotArmTasks(0,
            "/manipulation/manip_pln/la_goto_angles",
            "/manipulation/manip_pln/la_pose_wrt_arm",
            "/manipulation/manip_pln/la_pose_wrt_robot",
            "/manipulation/manip_pln/la_goto_loc",
            "/manipulation/manip_pln/la_move",
            "/manipulation/la_goal_reached"
            );
    m_rightArmStatus = new RobotArmStatus();
    m_leftArmStatus = new RobotArmStatus(
            0, 7, "/hardware/left_arm/current_pose",
            "/hardware/left_arm/current_gripper",
            "/hardware/left_arm/goal_gripper",
            "/hardware/left_arm/goal_pose",
            "/hardware/left_arm/torque_gripper",
            "/hardware/left_arm/goal_torque",
            "/hardware/robot_state/left_arm_battery"
            );
}

bool SimpleTasks::askForName(std::string &t_personName, int t_attemptTimeout, 
        int t_repeatTimeout, int t_maxTaskAttempts)
{
    using namespace boost::chrono;

    bool nameStored=false;
    t_personName = "";

    //Loop until the person provide it's name.
    int currentTaskAttempt=0;
    milliseconds millisElapsed;
    steady_clock::time_point taskStartTime = steady_clock::now();

    m_sprec.startListening();
    while(ros::ok() && millisElapsed.count() < t_attemptTimeout && !nameStored
            && currentTaskAttempt < t_maxTaskAttempts)
    {
        //verify the heared sentence
        if(m_sprec.isSentenceRecognized())
        {
            bool nameProvided=false;
            //if(nameProvided = m_langTasks.isNameProvide())
            //name provided... try to understand it
            nameStored = askAndWaitForConfirm("I hear that your name is " + 
                    t_personName + ", is that correct?", 20000, 10000);
            if(nameProvided && !nameStored)
            {
                //the name was provided but not understand... try again
                ++currentTaskAttempt;
                taskStartTime = steady_clock::now();
                millisElapsed = duration_cast<milliseconds>(
                        taskStartTime-taskStartTime);
                m_spgenTasks.asyncSpeech("sorry!");
                //m_spgenTasks.asyncSpeech("please again, tell me your name");
            }
            m_sprec.stopListening();
            m_sprec.startListening();
        }
        //Repeat the the question to the user again every t_repeatTimeOut time.
        if(millisElapsed.count()%t_repeatTimeout == 0)
        {
            m_spgenTasks.asyncSpeech("please tell me your name");
        }
        millisElapsed = duration_cast<milliseconds>(
                steady_clock::now() - taskStartTime
                );

        ros::spinOnce();
    }

    m_sprec.stopListening();

    if(!ros::ok() || millisElapsed.count()>=t_attemptTimeout)
    {
        m_spgenTasks.asyncSpeech("Sorry, I cannot hear you.");
        return false;
    }
    if(!nameStored)
    {
        m_spgenTasks.asyncSpeech("Sorry, I cannot understand you.");
        return false;
    }

    m_spgenTasks.asyncSpeech("Nice to meet you " + t_personName);
    return true;
}

bool SimpleTasks::askAndWaitForConfirm(std::string t_questionToAsk, 
        int t_timeout, int t_repeatTimeout)
{
    using namespace boost::chrono;

    //start to listen voic commands
    m_sprec.startListening();

    //Loop to wait for the user's response until timeout
    milliseconds millisElapsed;
    steady_clock::time_point taskStartTime = steady_clock::now();

    while(ros::ok() && millisElapsed.count() < t_timeout
            && !m_sprec.isSentenceRecognized())
    {
        //Repeat the the question to the user again every t_repeatTimeOut time.
        if(millisElapsed.count()%t_repeatTimeout == 0)
        {
    		m_spgenTasks.asyncSpeech(t_questionToAsk);
        }
        millisElapsed = duration_cast<milliseconds>(
                steady_clock::now() - taskStartTime
                );

        ros::spinOnce();
    }
    //disable the speech recognition listen mode
    m_sprec.stopListening();

    //verify if the time out was reached
    if(millisElapsed.count()>=t_timeout || !ros::ok())
    {
        return false;
    }
    //parse the recognized sentence
    return m_langundTasks.isPositiveUserConfirmation(
            m_sprec.getLastRecognizedSentence());
}

bool SimpleTasks::waitForStartFollowCommand(std::string t_sentenceToRepeat, 
        std::string &t_goalToFollow, int t_timeout, int t_repeatTimeout)
{
    using namespace boost::chrono;
    m_sprec.startListening();
    
    //Loop to wait for the user's start command
    milliseconds millisElapsed;
    steady_clock::time_point taskStartTime = steady_clock::now();
    while(ros::ok() && millisElapsed.count() < t_timeout)
    {
        //sentence heard
        if(m_sprec.isSentenceRecognized())
        {
            if(m_langundTasks.isStartFollowInstruction(
                        m_sprec.getLastRecognizedSentence(), t_goalToFollow))
            {
                m_sprec.stopListening();
                return true;
            }
            m_sprec.stopListening();
            m_sprec.startListening();
        }
        
        //Repeat the the question to the user again every t_repeatTimeOut time.
        if(millisElapsed.count()%t_repeatTimeout == 0)
        {
    		m_spgenTasks.asyncSpeech(t_sentenceToRepeat);
        }
        millisElapsed = duration_cast<milliseconds>(
                steady_clock::now() - taskStartTime
                );

        ros::spinOnce();
    }

    m_sprec.stopListening();
    t_goalToFollow = "";

    return false;
}

bool SimpleTasks::waitForStartGuideCommand(std::string t_sentenceToRepeat, 
        std::string &t_goalToGuide, int t_timeout, int t_repeatTimeout)
{
    using namespace boost::chrono;
    m_sprec.startListening();
    
    //Loop to wait for the user's start command
    milliseconds millisElapsed;
    steady_clock::time_point taskStartTime = steady_clock::now();
    while(ros::ok() && millisElapsed.count() < t_timeout)
    {
        //sentence heard
        if(m_sprec.isSentenceRecognized())
        {
            if(m_langundTasks.isStartGuideInstruction(
                        m_sprec.getLastRecognizedSentence(), t_goalToGuide))
            {
                m_sprec.stopListening();
                return true;
            }
            m_sprec.stopListening();
            m_sprec.startListening();
        }
        
        //Repeat the the question to the user again every t_repeatTimeOut time.
        if(millisElapsed.count()%t_repeatTimeout == 0)
        {
    		m_spgenTasks.asyncSpeech(t_sentenceToRepeat);
        }
        millisElapsed = duration_cast<milliseconds>(
                steady_clock::now() - taskStartTime
                );

        ros::spinOnce();
    }

    m_sprec.stopListening();
    t_goalToGuide = "";

    return false;
}

bool SimpleTasks::waitForCommand(
        std::string &t_command,
        std::map<std::string, std::string> &t_params, 
        int t_timeout
        )
{
    using namespace boost::chrono;
    m_sprec.startListening();
    
    //Loop to wait for the user's start command
    milliseconds millisElapsed;
    steady_clock::time_point taskStartTime = steady_clock::now();
    while(ros::ok() && millisElapsed.count() < t_timeout)
    {
        //sentence heard
        if(m_sprec.isSentenceRecognized())
        {
            if(m_langundTasks.isValidCommand(
                        m_sprec.getLastRecognizedSentence(),
                        t_command,
                        t_params))
            {
                m_sprec.stopListening();
                return true;
            }
            m_sprec.stopListening();
            m_sprec.startListening();
        }
        
        millisElapsed = duration_cast<milliseconds>(
                steady_clock::now() - taskStartTime
                );

        ros::spinOnce();
    }

    m_sprec.stopListening();
    t_command = "";

    return false;
}
