/**
 * @class RobotStatus
 * @brief Reads/modifies the global status of a mobile robot by subscribing to 
 * the corresponding status topics.
 *
 * @author R. Nonato Lagunas (nonato)
 * @author Marco Negrete
 * @version 0.1
*/
#ifndef _JUSTINA_GLOBALSTATUS_H_
#define _JUSTINA_GLOBALSTATUS_H_
#include <string>
#include "std_msgs/Empty.h"
#include "ros/ros.h"

class RobotStatus
{
    private:

        ros::NodeHandle *m_nh; /**< Stores the nodehandler used for the 
                                 subscriptions to topics/services. */

        bool m_isInitialized; /**< Indicates if the object is conncected with
                               ROS */

        std::string m_robotStopTopic; /**< Stores the name of the topic that 
                                        indicates to all the robot's hardware 
                                        to stop. */

        ros::Publisher m_pubRobotStopTopic; /**< Stores the publisher for the 
                                              robot stop indication. */

        /**
         * @brief Method to subscribe to the topics.
         */
        void prepareRosConnection();

    public:
        /**
         * @brief Class constructor
         * 
         * Creates a new RobotStatus object.
         *
         * @param nh[in] The ROS Node Handler of the calling node. If no node 
         * handler is provided then a new one will be created.
         * @param robotStopTopic The name of the robot's stop indication topic.
         */
        RobotStatus( 
                ros::NodeHandle *nh = 0,
                std::string robotStopTopic = "/hardware/robot_state/stop"
                );

        /**
         * @brief Initialize the communication of the object with ROS.
         *
         * @param nh[in] The ROS node handler of the calling node. If no node 
         * handler provided, the the node will create one later.
         */
        void initRosConnection(ros::NodeHandle *nh=0);
        
        /**
         * @brief Sends a stop signal to the stop indication topic.
         *
         * @return void.
         */
        void sendStopHardwareIndication();
};
#endif
