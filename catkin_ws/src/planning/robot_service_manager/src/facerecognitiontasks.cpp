#include "robot_service_manager/facerecognitiontasks.h"

FaceRecognitionTasks::FaceRecognitionTasks(ros::NodeHandle *t_nh, 
        std::string t_trainFacesTopicName,
        std::string t_trainFacesResultsTopicName,
        std::string t_startRecoFacesTopicName,
        std::string t_recoFacesResultTopicName,
        std::string t_clearFacesTopicName,
        std::string t_clearSpecificFaceTopicName
        ) : 
    m_nh(t_nh),
    m_trainFacesTopicName(t_trainFacesTopicName),
    m_trainFacesResultsTopicName(t_trainFacesResultsTopicName),
    m_startRecoFacesTopicName(t_startRecoFacesTopicName),
    m_recoFacesResultTopicName(t_recoFacesResultTopicName),
    m_clearFacesTopicName(t_clearFacesTopicName),
    m_clearSpecificFaceTopicName(t_clearSpecificFaceTopicName)
{
    m_framesTrained=0;
    m_faceTrainedFinished = false;
    m_faceRecognitionFinished = false;
    m_connectionInitialized = false;

    initRosConnection(m_nh);
}

void FaceRecognitionTasks::initRosConnection(ros::NodeHandle *t_nh)
{
    if(m_connectionInitialized)
    {
        /**
         * TODO: Print warning saying that the class is currently initialized.
         */
        return;
    }

    m_nh = t_nh;
    /**
     * Subscribe to the arm ros topics if the communication with 
     * ROS is initialized.
     */
    if(ros::isInitialized())
    {
        /**
         * If no node handler provided, then create a new one.
         */
        if(m_nh == 0)
        {
            m_nh = new ros::NodeHandle;
        }
        /*
         * Initialize subscribers, publishers and verification.
         */
        prepareRosConnection();
    }
    else
    {
        /*
         * TODO: Print error message if ros is not initialized.
         */
    }

}

void FaceRecognitionTasks::prepareRosConnection()
{
    /*
     * TODO: Print error messages if one of the object could not be 
     * initialized.
     */
    if(!m_trainFacesPub)
    {
        if((m_trainFacesPub = m_nh->advertise<
                    vision_msgs::VisionFaceTrainObject>(m_trainFacesTopicName,
                        100)))
        {
            m_connectionInitialized = true;
        } else {}
    }
    if(!m_trainFacesResultsSub)
    {
        if((m_trainFacesResultsSub = m_nh->subscribe(
                        m_trainFacesResultsTopicName, 100, 
                        &FaceRecognitionTasks::trainFacesResultsCallback, 
                        this)))
        {
            m_connectionInitialized = true; 
        } else {}
    }
    if(!m_startRecoFacesPub)
    {
        if((m_startRecoFacesPub = m_nh->advertise<
                std_msgs::Empty>(
                        m_startRecoFacesTopicName, 100)))
        {
            m_connectionInitialized = true;
        } else {}
    }
    if(!m_recoFacesResultSub)
    {
        if((m_recoFacesResultSub = m_nh->subscribe(m_recoFacesResultTopicName, 
                        100, &FaceRecognitionTasks::recoFacesResultsCallback,
                        this)))
        {
            m_connectionInitialized = true; 
        } else {}
    }
    if(!m_clearFacesPub)
    {
        if((m_clearFacesPub = m_nh->advertise<std_msgs::Empty>(
                        m_clearFacesTopicName, 100)))
        {
            m_connectionInitialized = true;
        } else {}
    }
    if(!m_clearSpecificFacePub)
    {
        if((m_clearSpecificFacePub = m_nh->advertise<std_msgs::String>(
                        m_clearSpecificFaceTopicName, 100)))
        {
            m_connectionInitialized = true;
        } else {}
    }
}

void FaceRecognitionTasks::trainFacesResultsCallback(
        const std_msgs::Int32::ConstPtr &t_savedFrames)
{
    m_framesTrained = t_savedFrames->data;
    m_faceTrainedFinished = true;
}

void FaceRecognitionTasks::recoFacesResultsCallback(
        const vision_msgs::VisionFaceObjects::ConstPtr &t_recoFacesMsg)
{
    m_lastDetectedFaces.clear();
    for(int i=0; i<t_recoFacesMsg->recog_faces.size(); i++)
    {
        std::vector<geometry_msgs::Point> faceBoundingBox;

        faceBoundingBox.push_back(
                t_recoFacesMsg->recog_faces[i].bounding_box[0]
                );
        faceBoundingBox.push_back(
                t_recoFacesMsg->recog_faces[i].bounding_box[1]
                );

        FaceObject currentFace(
                t_recoFacesMsg->recog_faces[i].id, 
                t_recoFacesMsg->recog_faces[i].confidence,
                t_recoFacesMsg->recog_faces[i].face_centroid,
                faceBoundingBox,
                t_recoFacesMsg->recog_faces[i].smile,
                t_recoFacesMsg->recog_faces[i].gender);
        
        m_lastDetectedFaces.push_back(currentFace);
    }
    m_faceRecognitionFinished = true;
}

void FaceRecognitionTasks::startFaceTraining(std::string t_faceID, int t_frames)
{
    if(!m_trainFacesPub)
    {
        //Try to connect.
        m_trainFacesPub = m_nh->advertise<vision_msgs::VisionFaceTrainObject>(
                m_trainFacesTopicName, 100);
    }
    if(m_trainFacesPub)
    {
        m_framesTrained=0;
        m_faceTrainedFinished = false;

        //publish the message
        vision_msgs::VisionFaceTrainObject trainFramesMsg;
        trainFramesMsg.id = t_faceID;
        trainFramesMsg.frames = t_frames;

        m_trainFacesPub.publish(trainFramesMsg);
        ros::spinOnce();
    }
    else
    {
        //TODO: Print error message if the publisher is not valid.
    }

}
bool FaceRecognitionTasks::faceTrainingFinished()
{
    return m_faceTrainedFinished;
}
bool FaceRecognitionTasks::trainFace(std::string t_faceID, int t_timeout, 
        int t_frames)
{  
    using namespace boost;

    startFaceTraining(t_faceID, t_frames);

    //loop until reach timeout or until found and train a face
    bool faceTrained = false;
    chrono::milliseconds elapsedTime;
    chrono::steady_clock::time_point startTime = chrono::steady_clock::now();
    while(ros::ok() && elapsedTime.count()<t_timeout && !faceTrained)
    {
        if(faceTrainingFinished() && m_framesTrained <= (int)(t_frames/2))
        {
            startFaceTraining(t_faceID, t_frames);
        }
        if(faceTrainingFinished() && m_framesTrained > (int)(t_frames/2))
        {
            faceTrained = true;
        }
        elapsedTime = chrono::duration_cast<chrono::milliseconds>(
                chrono::steady_clock::now() - startTime
                );
        ros::spinOnce();
    }

    if(elapsedTime.count() >= t_timeout || !ros::ok())
    {
        return false;
    }
    
    return faceTrained;
}

void FaceRecognitionTasks::startFaceRecognition()
{
    if(!m_startRecoFacesPub)
    {
        //Try to connect.
        m_startRecoFacesPub = m_nh->advertise<std_msgs::Empty>(
                m_startRecoFacesTopicName, 100);
    }
    if(m_startRecoFacesPub)
    {
        m_faceRecognitionFinished = false;

        //publish the message
        std_msgs::Empty emptyMsg;

        m_startRecoFacesPub.publish(emptyMsg);
    }
    else
    {
        //TODO: Print error message if the publisher is not valid.
    }
}

bool FaceRecognitionTasks::faceRecognitionFinished()
{
    return m_faceRecognitionFinished;
}

bool FaceRecognitionTasks::recognizeFaces(std::vector<FaceObject> 
        &t_detectedFaces, int t_timeout)
{
    using namespace boost;

    startFaceRecognition();

    bool facesDetected = false;
    chrono::milliseconds elapsedTime;
    chrono::steady_clock::time_point startTime = chrono::steady_clock::now();
    while(ros::ok() && !facesDetected && 
            elapsedTime.count()<t_timeout)
    {
        if(faceRecognitionFinished() && m_lastDetectedFaces.size() == 0)
        {
            startFaceRecognition();
        }
        if(faceRecognitionFinished() && m_lastDetectedFaces.size() > 0)
        {
            facesDetected = true;
        }
        elapsedTime = chrono::duration_cast<chrono::milliseconds>(
                chrono::steady_clock::now() - startTime
                );
        ros::spinOnce();
    }

    if(elapsedTime.count() >= t_timeout || !ros::ok() || 
            m_lastDetectedFaces.size() == 0)
    {
        t_detectedFaces = std::vector<FaceObject>();
        return false;
    }
    
    //face recognition task executed successfully
    t_detectedFaces = m_lastDetectedFaces;
    return true;
}

void FaceRecognitionTasks::clearFacesDataBase()
{
    if(!m_clearFacesPub)
    {
        //Try to connect.
        m_clearFacesPub = m_nh->advertise<std_msgs::Empty>(
                m_clearFacesTopicName, 100);
    }
    if(m_clearFacesPub)
    {
        //publish the message
        std_msgs::Empty emptyMsg;
        m_clearFacesPub.publish(emptyMsg);
    }
    else
    {
        //TODO: Print error message if the publisher is not valid.
    }
}

void FaceRecognitionTasks::clearSpecificFace(std::string t_faceID)
{
    if(!m_clearSpecificFacePub)
    {
        //Try to connect.
        m_clearSpecificFacePub = m_nh->advertise<std_msgs::String>(
                m_clearSpecificFaceTopicName, 100);
    }
    if(m_clearSpecificFacePub)
    {
        //publish the message
        std_msgs::String faceIDMsg;
        faceIDMsg.data = t_faceID;
        m_clearSpecificFacePub.publish(faceIDMsg);
    }
    else
    {
        //TODO: Print error message if the publisher is not valid.
    }

}
