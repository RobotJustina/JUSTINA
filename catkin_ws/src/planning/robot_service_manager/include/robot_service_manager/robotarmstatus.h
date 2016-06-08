/**
 * @class RobotArmStatus
 * @brief Reads/modifies the status of a robot arm by subscribing to the 
 * corresponding status topics.
 *
 * @author R. Nonato Lagunas (nonato)
 * @author Marco Negrete 
 * @version 0.1
*/
#ifndef _JUSTINA_ROBOTARMSTATUS_H_
#define _JUSTINA_ROBOTARMSTATUS_H_
#include <string>
#include <vector>
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "ros/ros.h"

class RobotArmStatus
{
    private:

        ros::NodeHandle *m_nh; /**< Stores the nodehandler used for the 
                                 subscriptions to topics/services. */

        bool m_isInitialized; /**< Indicates if the object is conncected with
                               ROS */

        std::string m_armCurrentPoseTopic; /**< Stores the name of the topic 
                                             which manage the information of 
                                             the arm pose. */

        std::string m_armCurrentGripperTopic; /**< Stores the name of the topic
                                                which manage the information of
                                                the arm gripper. */

        std::string m_armGoalGripperTopic; /**< Stores the name of the topic
                                             which manage the information of 
                                             the gripper goal. */

        std::string m_armGoalPoseTopic; /**< Stores the name of the topic which
                                          manage the information of the pose 
                                          goal (the position of the entire arm)
                                          */

        std::string m_armGripperTorqueTopic; /**< Stores the name of the topic
                                               which manage the information of 
                                               the gripper torque. */

        std::string m_armGoalTorqueTopic; /**< Stores the name of the topic 
                                            which manage the information of the
                                            pose torque. */

        std::string m_armCurrentBatteryTopic; /**< Stores the name of the topic 
                                                which manage the information of
                                                the arm battery level. */

        ros::Subscriber m_subArmCurrentGripper; /**< Stores the subscriber for
                                                  the current gripper topic. */

        ros::Subscriber m_subArmCurrentPose; /**< Stores the subscriber for the
                                               current pose topic. */

        ros::Subscriber m_subArmCurrentBattery; /**< Stores the subscriber for 
                                                  the current arm battery 
                                                  topic.*/

        ros::Publisher m_pubArmGoalGripper; /**< Stores the publisher for the 
                                              arm gripper goal topic. */

        ros::Publisher m_pubArmGoalPose; /**< Stores the publisher for the arm
                                           goal pose topic. */

        ros::Publisher m_pubArmTorqueGrip; /**< Stores the publisher for
                                                 the arm gripper torque 
                                                 topic. */

        ros::Publisher m_pubArmGoalTorque; /**< Stores the publisher for the 
                                             arm goal torque topic. */

        float m_armCurrentGripper; /**< Stores the information of the current 
                                     value of the arm gripper. */

        std::vector<float> m_armCurrentPose; /**< Stores the information of the
                                               current arm pose. */

        float m_armBatteryLevel; /**< Stores the information of the current 
                                   battery level of the robot arm. */

        int m_armBatteryPerc; /**< Stores the information of the current 
                                  battery level (percentage) of the robot arm. 
                                  */

        int m_armDOF; /**< Stores the number of degree of freedom that the arm 
                        has. */
        /**
         * @brief Method to subscribe to the topics.
         */
        void prepareRosConnection();

        /**
         * @brief Arm current gripper value callback.
         * 
         * Updates the robot's arm gripper value when the corresponding topic 
         * is updated.
         *
         * @param floatMsg The new value of the topic when it's updated. 
         */
        void armCurrentGripperCallback(
                const std_msgs::Float32::ConstPtr& floatMsg);

        /**
         * @brief Arm current pose value callback.
         * 
         * Updates the robot's arm pose value when the corresponding topic 
         * is updated.
         *
         * @param floatArrayMsg The new value of the topic when it's updated. 
         */
        void armCurrentPoseCallback(
                const std_msgs::Float32MultiArray::ConstPtr& floatArrayMsg);

        /**
         * @brief Arm current battery value callback.
         * 
         * Updates the robot's arm battery value when the corresponding topic 
         * is updated.
         *
         * @param floatMsg The new value of the topic when it's updated. 
         */
        void armCurrentBatteryCallback(
                const std_msgs::Float32::ConstPtr& floatMsg);
    public:
        /**
         * @brief Class constructor
         * 
         * Creates a new RobotArmStatus object.
         *
         * @param nh[in] The ROS Node Handler of the calling node. If no node 
         * handler is provided then a new one will be created.
         * @param armDOF The degree of freedoms of the robot arm.
         * @param armCurrentPoseTopic The name of the arm current pose topic.
         * @param armCurrentGripperTopic The name of the arm current gripper 
         * topic.
         * @param armGoalGripperTopic The name of the arm goal gripper topic.
         * @param armGoalPoseTopic The name of the arm goal pose topic.
         * @param armGripperTorqueTopic The name of the arm gripper torque 
         * topic.
         * @param armGoalTorqueTopic The name of the arm goal torque topic.
         */
        RobotArmStatus( 
                ros::NodeHandle *nh = 0,
                int armDOF = 7,
                std::string armCurrentPoseTopic = 
                "/hardware/right_arm/current_pose",
                std::string armCurrentGripperTopic = 
                "/hardware/right_arm/current_gripper",
                std::string armGoalGripperTopic = 
                "/hardware/right_arm/goal_gripper",
                std::string armGoalPoseTopic = 
                "/hardware/right_arm/goal_pose",
                std::string armGripperTorqueTopic = 
                "/hardware/right_arm/torque_gripper",
                std::string armGoalTorqueTopic = 
                "/hardware/right_arm/goal_torque",
                std::string armCurrentBatteryTopic =
                "/hardware/robot_state/right_arm_battery"
                );

        /**
         * @brief Initialize the communication of the object with ROS.
         *
         * @param nh[in] The ROS node handler of the calling node. If no node 
         * handler provided, the the node will create one later.
         */
        void initRosConnection(ros::NodeHandle *nh=0);
        
        /**
         * @brief Change the status of the robot arm gripper by updating it
         * corresponding ROS topic.
         *
         * @param goalGripper The goal gripper value (the new value of the
         * topic).
         * @return void.
         */
        void setGoalGripper(float goalGripper);

        /**
         * @brief Change the status of the robot arm goal pose by updating its 
         * corresponding ROS topic.
         *
         * The calling of this function causes that the arm moves to a desired
         * pose indicating by an angles vector.
         *
         * @param goalAngles[in] The vector of goal angles to move the arm to.
         * @return void.
         */
        void setGoalPose(std::vector<float> &goalAngles);

        /**
         * @brief Change the status of the robot arm torque gripper by 
         * updating it corresponding ROS topic.
         *
         * @param goalTorqueGrip The new torque gripper value (the new value
         * of the topic).
         * @return void.
         */
        void setTorqueGrip(float torqueGrip);

        /**
         * @brief Change the status of the robot arm goal torques by updating
         * its corresponding ROS topic.
         *
         * @param goalTorques[in] The value of the torques to update (the new 
         * value of the topic).
         * @return void.
         */
        void setArmGoalTorque(std::vector<float> &goalTorques);

        /**
         * @Brief returns the arm current gripper.
         *
         * @return float
         */
        float getArmCurrentGripper();
            
        /**
         * @Brief returns the arm current pose vector.
         *
         * @return vector<float>
         */
        std::vector<float> getArmCurrentPose();
            
        /**
         * @Brief returns the arm battery level.
         *
         * @return float
         */
        float getArmBatteryLevel();

        /**
         * @Brief returns the arm battery percentage.
         *
         * @return int
         */
        int getArmBatteryPerc();
};
#endif
