/**
 * @class HeadStatus
 * @brief Reads and modifies the current status of the robot's head.
 *
 * Reads the robot's head status from ROS, from the corresponding topics/servi-
 * ces published and/or advertised by the hardware head modules. Also it writes
 * the news values for the head status.
 * 
 * @author R. Nonato Lagunas (nonato)
 * @version 0.1
*/
#ifndef _JUSTINA_HEADSTATUS_H
#define _JUSTINA_HEADSTATUS_H
#include <string>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"

class HeadStatus
{
    private:
        ros::NodeHandle *m_nh; /**< Stores the nodehandler used for the 
                                 subscriptions to topics/services. */

        bool m_isInitialized; /**< Indicates if the object is conncected with
                               ROS */

        ros::Subscriber m_subHeadCurrentPose; /**< ROS subscriber to the head
                                                pose topic */

        ros::Subscriber m_subHeadCurrentBattery; /**< ROS subscriber to the 
                                                   head battery topic. */

        ros::Publisher m_headPosePublisher; /**< ROS publisher for the head
                                              pose topic*/

        std::string m_headCurrentPoseTopic; /**< Stores the name of the topic 
                                              from where the head status will 
                                              be obtained*/

        std::string m_headGoalPoseTopic; /**< Stores the name of the topic from 
                                       where the head status will be obtained*/

        std::string m_headBatteryTopic; /**< Stores the name of the topic from 
                                          where the head battery status will be
                                          obtained*/

        float m_headPan; /**< Stores the current robot's head pan */

        float m_headTilt; /**< Stores the current robot's head tilt*/

        float m_headBattery; /**< Stores the robot's battery level. */

        /**
         * @brief Head current battery callback
         * 
         * Updates the robot's head battery when the corresponding topic is
         * updated.
         *
         * @param batteryMsg The new value of the topic when it's updated. 
         */
        void headBatteryCallback(
                const std_msgs::Float32::ConstPtr& batteryMsg);

        /**
         * @brief Head current pose callback
         * 
         * Updates the robot's head pose when the corresponding topic is
         * updated.
         *
         * @param poseMsg The new value of the topic when it's updated. 
         */
        void headPoseCallback(
                const std_msgs::Float32MultiArray::ConstPtr& poseMsg);

        /**
         * @brief Method to subscribe to the topics.
         */
        void prepareRosConnection();
    public:
        /**
         * @brief Class constructor
         * 
         * Creates a new HeadStatus object.
         *
         * @param nh The ROS Node ÇHandler of the simple task planner node.
         * @param headCurrentPoseTopic The name of the topic which will be 
         * updated when the robot's head pose information change.
         * @param headGoalPoseTopic The name of the topic to update the robot's
         * head pose (i.e. move the head).
         * @param headBatteryTopic The name of the topic which will be updated
         * when the robot's head battery information change.
         */
        HeadStatus(ros::NodeHandle *nh = 0, 
                std::string headCurrentPoseTopic = 
                "/hardware/head/current_pose", 
                std::string headGoalPoseTopic = 
                "/hardware/head/goal_pose", 
                std::string headBatteryTopic = 
                "/hardware/robot_state/head_battery"
                );
        
        /**
         * @brief Initialize the communication of the object with ROS.
         *
         * @param nh The ROS node handler of the calling node. If no node 
         * handler provided, the the node will create one later.
         */
        void initRosConnection(ros::NodeHandle *nh = 0);

        /**
         * @brief Returns the value of the current head battery level.
         * 
         * @return float.
         */
        float getHeadBattery();

        /**
         * @brief Returns the value of the current head pan.
         * 
         * @return The value of the current head pan.
         */
        float getHeadPan();

        /**
         * @brief Returns the value of the current head tilt.
         * 
         * @return The value of the current head tilt.
         */
        float getHeadTilt();

        /**
         * @brief Returns the value of the current head pan and tilt by 
         * reference.
         * 
         * @param headPan Stores the value of the current head pan.
         * @param headPan Stores the value of the current head tilt.
         * @return void
         */
        void getHeadPose(float &headPan, float &headTilt);

        /**
         * @brief Set the goal values for the head pose.
         *
         * Receives two float values that represent the goal head tilt and pan,
         * and writes it to the corresponding ROS topic.
         * 
         * @param headPan The goal head pan value.
         * @param headPan The goal head tilt value.
         * @return void
         */
        void setHeadPose(float headPan, float headTilt);
};
#endif
