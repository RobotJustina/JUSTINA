/**
 * @class ObjectRecoTasks
 * @brief Containt methods to perform tasks related with object recognition.
 *
 * @author R. Nonato Lagunas (nonato)
 * @version 0.1
*/
#ifndef _JUSTINA_OBJECTRECOTASKS_H_
#define _JUSTINA_OBJECTRECOTASKS_H_
#include <string>
#include <vector>
#include "vision_msgs/VisionObject.h"
#include "vision_msgs/DetectObjects.h"
#include "ros/ros.h"

class ObjectRecoTasks
{
    private:

        ros::NodeHandle *m_nh; /**< ros node handle onject **/

        bool m_connectionInitialized; /**< Indicates if the object is 
                                        conncected with ROS **/

        //Members to store the name of the recognition object services
        std::string m_detectObjectsSrvName;
        std::string m_recoObjectsSrvName;

        //Ros objects for the reco obj services.
        ros::ServiceClient m_detectObjectsSrv;
        ros::ServiceClient m_recoObjectsSrv;

        /**
         * @brief Method to advertise the object recognizer services.
         */
        void prepareRosConnection();
        
    public:
        
        /**
         * @brief Class constructor
         * 
         * Creates a new ObjectRecoTasks object 
         *
         * @param nh The ROS Node Handler of the calling node. If 
         * no node handler is provided then a new one will be created.
         * @param t_detectObjecstSrvName The name of the service to detect 
         * objects on a plane.
         * @param t_recoObjectsSrvName The name of the service to recognize 
         * objects on a table.
         */
        ObjectRecoTasks(
                ros::NodeHandle *t_nh = 0,
                std::string t_detectObjectsSrvName = 
                "/vision/det_objs",
                std::string t_recoObjectsSrvName = 
                "/vision/recog_objects"
                );

        /**
         * @brief Initialize the communication of the object with ROS.
         *
         * @param t_nh[in] The ROS node handler of the calling node. If no node 
         * handler provided, the the node will create one later.
         */
        void initRosConnection(ros::NodeHandle *t_nh=0);

        /**
         * Blocks the calling thread while tries to recognize objects using the
         * obejct recognition node.
         *
         * @param[out] t_recognizedObjects The list of recognized objects.
         * @return True if the object recognition was executed succesfully
         * (doesn't matter tath no objects were recognized). False otherwise.
         */
        bool detectObjects(
                std::vector<vision_msgs::VisionObject> &t_recognizedObjects);
};
#endif
