/**
 * @class FaceRecognitionTasks
 * @brief Containt methods to perform tasks related with face recognition.
 *
 * @author R. Nonato Lagunas (nonato)
 * @version 0.1
*/
#ifndef _JUSTINA_FACERECOGNITIONTASKS_H_
#define _JUSTINA_FACERECOGNITIONTASKS_H_
#include <string>
#include <vector>
#include "std_msgs/Int32.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "vision_msgs/VisionFaceObjects.h"
#include "vision_msgs/VisionFaceTrainObject.h"
#include "ros/ros.h"
#include <boost/chrono.hpp>
class FaceRecognitionTasks
{
    public:
        //Forward declaration
        struct FaceObject;
    private:

        ros::NodeHandle *m_nh; /**< ros node handle onject **/

        bool m_connectionInitialized; /**< Indicates if the object is 
                                        conncected with ROS **/

        bool m_faceTrainedFinished; /**< Indicates when a face training task
                                      is finished. **/

        bool m_faceRecognitionFinished; /**< Indicates when the face 
                                          recognition task is ended. **/

        int m_framesTrained; /**< stores the number of frames stored for the 
                               trained face. **/

        /**< Stores the detected faces by the last recognition task. **/
        std::vector<FaceRecognitionTasks::FaceObject> m_lastDetectedFaces; 

        //Members to store the name of the recognition faces topics
        std::string m_trainFacesTopicName;
        std::string m_trainFacesResultsTopicName;
        std::string m_startRecoFacesTopicName;
        std::string m_recoFacesResultTopicName;
        std::string m_clearFacesTopicName;
        std::string m_clearSpecificFaceTopicName;

        //Ros publishers and subscriber for the reco faces topics.
        ros::Publisher m_trainFacesPub;
        ros::Subscriber m_trainFacesResultsSub;
        ros::Publisher m_startRecoFacesPub;
        ros::Subscriber m_recoFacesResultSub;
        ros::Publisher m_clearFacesPub;
        ros::Publisher m_clearSpecificFacePub;

        /**
         * @brief Method to subscribe to the topics.
         */
        void prepareRosConnection();
        
        /**
         * @brief Callback for the face recognition results.
         */
        void recoFacesResultsCallback(
                const vision_msgs::VisionFaceObjects::ConstPtr &t_recoFacesMsg);

        /**
         * @brief Callback for the face training results.
         */
        void trainFacesResultsCallback(
                const std_msgs::Int32::ConstPtr &t_savedFrames);
    public:
        
        /**
         * @struct FaceObject
         * @brief A structure to store a recognized face with the right format.
         *
         * @var faceID Store the ID of the recognized face.
         * @var confidence Stores the confidence of the recognized face.
         * @var smilingFace Indicates if a smile was detected on the recognized
         * face.
         * @var faceGender Indicates the gender of the recognize face.
         */
        struct FaceObject
        {
            std::string faceID; 
            float confidence;
            geometry_msgs::Point faceCentroid;
            std::vector<geometry_msgs::Point> boundingBox;
            bool smilingFace; 
            int faceGender;

            FaceObject() {}
            FaceObject(std::string t_faceID, float t_confidence, 
                    geometry_msgs::Point t_faceCentroid,
                    std::vector<geometry_msgs::Point> t_boundingBox,
                    bool t_smilingFace, int t_faceGender) : 
                faceID(t_faceID), confidence(t_confidence), 
                boundingBox(t_boundingBox), faceCentroid(t_faceCentroid),
                smilingFace(t_smilingFace), faceGender(t_faceGender) {}
        };
        /**
         * @brief Class constructor
         * 
         * Creates a new FaceRecognitionTasks object 
         *
         * @param nh The ROS Node Handler of the calling node. If 
         * no node handler is provided then a new one will be created.
         * @param t_trainFacesTopicName The name of the topic to indicate to 
         * the face recognition module that the face training process must be 
         * started.
         * @param t_trainFacesResultsTopicName The name of the topic where the 
         * face recognition module will publish the result of the training 
         * process.
         * @param t_startRecoFacesTopicName The name of the topic to indicate 
         * to the face recognition module that the face recognition process 
         * must be started.
         * @param t_recoFacesResultTopicName The name of the topic where the 
         * face recognition module will publish the result of the recognition 
         * process.
         * @param t_clearFacesTopicName The name of the topic to indicate to 
         * the face recognition module that all the remembered faces must be 
         * deleted.
         * @param t_clearSpecificFaceTopicName The name of the topic to
         * indicate to the face recognition module that a specific face name 
         * must be deleted.
         */
        FaceRecognitionTasks(
                ros::NodeHandle *t_nh = 0,
                std::string t_trainFacesTopicName = 
                "/vision/face_recognizer/run_face_trainer_frames",
                std::string t_trainFacesResultsTopicName = 
                "/vision/face_recognizer/trainer_result",
                std::string t_startRecoFacesTopicName =
                "/vision/face_recognizer/run_face_recognizer",
                std::string t_recoFacesResultTopicName = 
                "/vision/face_recognizer/faces",
                std::string t_clearFacesTopicName = 
                "/vision/face_recognizer/clearfacesdb",
                std::string t_clearSpecificFaceTopicName = 
                "/vision/face_recognizer/clearfacesdbbyid"
                );

        /**
         * @brief Initialize the communication of the object with ROS.
         *
         * @param t_nh[in] The ROS node handler of the calling node. If no node 
         * handler provided, the the node will create one later.
         */
        void initRosConnection(ros::NodeHandle *t_nh=0);

        /**
         * @brief Starts a face training task.
         *
         * Performs the task using an specified number of photo frames and 
         * assigns an specified ID to the trained face.
         * Be careful! This method does not wait for the trainer to finish.
         *
         * @param t_faceName The id to assign to the trained face.
         * @param t_frames The number of frames to store for the trained face.
         */
        void startFaceTraining(std::string t_faceID, int t_frames=10);

        /**
         * @brief Indicates if the current training task is finished.
         *
         * @return True if the current training task is finished. False 
         * otherwise.
         */
        bool faceTrainingFinished();

        /**
         * @brief Performs a train face task.
         *
         * Performs the task using an specified number of photo frames and 
         * assigns to the trained face an specified ID. This method blocks the 
         * calling thread till the face training finished or until complete a 
         * time limit.
         *
         * @param t_faceID The ID to assign to the trained face.
         * @param t_timeout The limit time to interrupt the task.
         * @param t_frames The number of frames to store for the trained face.
         * @return  True if the face was trained succesfully. False otherwise.
         */
        bool trainFace(std::string t_faceID, int t_timeout, int t_frames=10);

        /**
         * @brief Starts a face recognition task.
         *
         * Be careful! This method does not wait for the recognizer to finish.
         */
        void startFaceRecognition();

        /**
         * @brief Indicates if the current recognition task is finished.
         *
         * @return True if the current recognition task is finished. False 
         * otherwise.
         */
        bool faceRecognitionFinished();

        /**
         * @brief Performs a face recognition task.
         *
         * Performs the task and return the resulting configuration of the 
         * recognized faces. This method blocks the calling thread until the
         * face recognition finish of until reach a time limit.
         *
         * @param t_detectedFaces The set of detected faces.
         * @param t_timeout The limit time to interrupt the task.
         * @return  True if a face (or faces) where recognized in the scene. 
         * False otherwise.
         */
        bool recognizeFaces(std::vector<FaceObject> &t_detectedFaces, 
                int t_timeout);

        /**
         * @brief Clears all the database of the face recognition node.
         */
        void clearFacesDataBase();

        /**
         * @brief Clear an specified face from the database of the face 
         * recognition node.
         *
         * @param t_faceID The ID of the face's data base to delete.
         */
        void clearSpecificFace(std::string t_faceID);
};
#endif
