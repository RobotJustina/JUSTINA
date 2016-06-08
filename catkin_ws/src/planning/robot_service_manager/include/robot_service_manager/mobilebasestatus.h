/**
 * @class MobileBaseStatus
 * @brief Reads/modifies the status of a robot mobile base by subscribing to 
 * the corresponding status topics.
 *
 * @author R. Nonato Lagunas (nonato)
 * @author Marco Negrete
 * @version 0.1
*/
#ifndef _JUSTINA_MOBILEBASESTATUS_H_
#define _JUSTINA_MOBILEBASESTATUS_H_
#include <string>
#include <vector>
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

class MobileBaseStatus
{
    private:

        ros::NodeHandle *m_nh; /**< Stores the nodehandler used for the 
                                 subscriptions to topics/services. */

        bool m_isInitialized; /**< Indicates if the object is conncected with
                               ROS */

        std::string m_mbSpeedsTopic; /**< Stores the name of the topic which 
                                       manage the information of the base 
                                       speeds. */

        std::string m_mbCmdVelTopic; /**< Stores the name of the topic which 
                                       manage the information of the base 
                                       command velocity topic. */

        std::string m_mbBatteryTopic; /**< Stores the name of the topic which 
                                        manage the information of the base 
                                        battery. */

        ros::Subscriber m_subMBBattery; /**< Stores the subscriber for the 
                                          mobile base battery topic. */

        ros::Publisher m_pubMBSpeeds; /**< Stores the publisher for the mobile
                                        base speeds topic. */

        ros::Publisher m_pubMBCmdVel; /**< Stores the publisher for the mobile
                                        base command velocity topic. */

        int m_mbMotors; /**< Stores the number of motors on the mobile base. */

        float m_mbBattery; /**< Stores the battery level of the mobile base. */

        /**
         * @brief Method to subscribe to the topics.
         */
        void prepareRosConnection();

        /**
         * @brief Mobile base current battery level callback.
         * 
         * Updates the mobile base battery level when the corresponding topic
         * is updated.
         *
         * @param batteryMsg The new value of the topic when it's updated. 
         */
        void mbBatteryCallback(
                const std_msgs::Float32::ConstPtr& batteryMsg);

    public:
        /**
         * @brief Class constructor
         * 
         * Creates a new MobileBaseStatus object.
         *
         * @param nh[in] The ROS Node Handler of the calling node. If no node 
         * handler is provided then a new one will be created.
         * @param mbMotors The number of motors on the mobile base.
         * @param mbSpeedsTopic The name of the mobile base motor speeds topic.
         * @param mbCmdVelTopic The name of the mobile base command velocity 
         * topic.
         */
        MobileBaseStatus( 
                ros::NodeHandle *nh = 0,
                int mbMotors = 7,
                std::string mbSpeedsTopic = 
                "/hardware/mobile_base/speeds",
                std::string mbCmdVelTopic = 
                "/hardware/mobile_base/cmd_vel",
                std::string mbBatteryTopic = 
                "/hardware/robot_state/base_battery"
                );

        /**
         * @brief Initialize the communication of the object with ROS.
         *
         * @param nh[in] The ROS node handler of the calling node. If no node 
         * handler provided, the the node will create one later.
         */
        void initRosConnection(ros::NodeHandle *nh=0);
        
        /**
         * @brief Change the status of the robot mobile base speed by updating 
         * it corresponding ROS topic.
         *
         * @param speedsVector[in] A vector that contains the new speeds for the 
         * mobile base motors. It size depends on the number of motor in the 
         * mobile base and the arrangement pf the vector is {leftMotorSpeed, 
         * rightMotorSpeed, upperMotorSpeed, lowerMotorSpeed}.
         * @return void.
         */
        void setMobileBaseSpeeds(std::vector<float> &speedsVector);

        /**
         * @brief Send a mobile base command velocity to move the robot's 
         * mobile base.
         *
         * @param linearX The value of the linear x velocity.
         * @param linearY The value of the linear y velocity.
         * @param angular The value of the angular velocity.
         * @return void.
         */
        void setMobileBaseCmdVel(float linearX, float linearY, float angular);

        /**
         * @brief Returns the current mobile base battery level.
         *
         * @return The battery level.
         */
        float getMobileBaseBattery();
};
#endif
