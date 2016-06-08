#include "simple_task_planner/simpletasks.h"

bool SimpleTasks::faceSearchAndRemeber(std::string t_findInstructions, 
        std::string t_faceID, 
        std::vector<std::pair<float, float> > t_headMovements)
{
    using namespace std;

    //setting default parameters
    if(t_headMovements.size()==0)
    {
        initializeHeadVector(t_headMovements);
    }
    t_findInstructions = (t_findInstructions=="default") ? 
        "Please look straight to my kinect camera" : t_findInstructions;

    //Instruct to the human
    m_spgenTasks.syncSpeech(t_findInstructions, 10000);

    //search for the human face by moving the head to different positions
    bool faceTrained=false;
    ros::Duration headWaitDuration(3);
    for(int currentHeadPos=0; 
            currentHeadPos<t_headMovements.size() && !faceTrained;
            ++currentHeadPos)
    {
        //move the head to the current position of the list
        m_headStatus.setHeadPose(
                t_headMovements[currentHeadPos].first,
                t_headMovements[currentHeadPos].second
                );
        
        //wait for the head to reach the position (just a timer not a topic 
        //update)
        headWaitDuration.sleep();

        //Try to train the face on the current head position
        if((faceTrained = m_recoFaces.trainFace(t_faceID, 10000, 10)))
        {
            m_spgenTasks.syncSpeech("human, smile", 5000);
            m_recoFaces.trainFace(t_faceID, 3000);
            m_spgenTasks.syncSpeech("now, a serious face", 5000);
            m_recoFaces.trainFace(t_faceID, 3000);
            m_spgenTasks.syncSpeech("I have remembered your face", 5000);
        }
        ros::spinOnce();
    }
    
    return faceTrained;
}

bool SimpleTasks::findPersonInLocation(std::string &t_personFaceID, 
        std::string &t_searchLocation, 
        FaceRecognitionTasks::FaceObject &t_faceConfiguration)
{
    int maxAttemptPerTask = 3;
    int navTimeout = 120000;

    //move the robot to the location where the person will be searched
    bool navSuccess = false;
    int navAttempt = 0;
    while(!navSuccess && navAttempt < maxAttemptPerTask)
    {
       navAttempt = (!(navSuccess = 
                   m_navTasks.syncGetClose(t_searchLocation, navTimeout))) ? 
               navAttempt : navAttempt+1;
    }
    //if the navigation stage was not successfull, then return a fail state
    if(!navSuccess)
    {
        return false;
    }

    //search the person in the current location
    std::vector<std::pair<float, float> > headMovements;
    initializeHeadVector(headMovements);
    //search for the human face by moving the head to different positions
    bool faceFound=false;
    int faceIndex;
    ros::Duration headWaitDuration(3);
    for(int currentHeadPos=0; 
            currentHeadPos<headMovements.size() && !faceFound;
            ++currentHeadPos)
    {
        //move the head to the current position of the list
        m_headStatus.setHeadPose(
                headMovements[currentHeadPos].first,
                headMovements[currentHeadPos].second
                );
        
        //wait for the head to reach the position (just a timer not a topic 
        //update)
        headWaitDuration.sleep();

        //Try to recognize the face on the current head position
        std::vector<FaceRecognitionTasks::FaceObject> detectedFaces;
        if(m_recoFaces.recognizeFaces(detectedFaces, 10000))
        {
            faceIndex = linearSearchFaceID(detectedFaces, t_personFaceID);
            faceFound = (faceIndex > 0) ? true : false;
            t_faceConfiguration = (faceIndex > 0) ? detectedFaces[faceIndex] : 
                t_faceConfiguration;
        }
        ros::spinOnce();
    }

    return faceFound;
}
int SimpleTasks::linearSearchFaceID(
        std::vector<FaceRecognitionTasks::FaceObject> &t_facesList, 
        std::string t_faceID)
{
    //verify if one of the detected faces belongs to the human to find
    for(int currentFace = 0; currentFace < t_facesList.size();
            currentFace++)
    {
        if(t_facesList[currentFace].faceID == t_faceID)
        {
            return currentFace;
        }
    }
    return -1;
}

void SimpleTasks::initializeHeadVector(
        std::vector<std::pair<float, float> > &t_vector)
{
    using namespace std;

    t_vector.clear();
    t_vector.push_back(pair<float, float>(0.0, 0.0));
    t_vector.push_back(pair<float, float>(-0.4, 0.0));
    t_vector.push_back(pair<float, float>(0.4, 0.0));
    t_vector.push_back(pair<float, float>(0.0, -0.4));
    t_vector.push_back(pair<float, float>(-0.4, -0.4));
    t_vector.push_back(pair<float, float>(0.4, -0.4));
    t_vector.push_back(pair<float, float>(0.0, 0.4));
    t_vector.push_back(pair<float, float>(-0.4, 0.4));
    t_vector.push_back(pair<float, float>(0.4, 0.4));
    t_vector.push_back(pair<float, float>(0.0, 0.0));
}
