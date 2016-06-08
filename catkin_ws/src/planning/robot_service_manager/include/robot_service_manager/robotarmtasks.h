/**
 * @class RobotArmTasks
 * @brief Perform robot arm tasks.
 *
 * @author R. Nonato Lagunas (nonato)
 * @version 0.1
*/
#ifndef _JUSTINA_ROBOTARMTASKS_H_
#define _JUSTINA_ROBOTARMTASKS_H_
#include <string>
#include <boost/chrono.hpp>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "ros/ros.h"
class RobotArmTasks
{
    private:
        ros::NodeHandle *m_nh; /**< The ROS node handle object */

        bool m_connectionInitialized; /**< Indicates if the object is 
                                        conncected with ROS **/

        bool m_armGoalReached; /**< Indicates if the arm reachs the goal 
                                 position after a movement. */

        /** Members to store the topic names passed by parameter to the class 
         * contructor.
         */
        std::string m_armGoToAnglesTopicName; 
        std::string m_armGotoPoseWRTATopicName;
        std::string m_armGoToPoseWRTRTopicName;
        std::string m_armGoToLocationTopicName;
        std::string m_armMoveTopicName;
        std::string m_armGoalReachedTopicName;

        /** Members to publish/read to/from the different arm topics. */
        ros::Publisher m_armGoToAnglesPub; 
        ros::Publisher m_armGotoPoseWRTAPub;
        ros::Publisher m_armGoToPoseWRTRPub;
        ros::Publisher m_armGoToLocationPub;
        ros::Publisher m_armMovePub;
        ros::Subscriber m_armGoalReachedSub;


        /**
         * Method to subscribe to the topics.
         */
        void prepareRosConnection();

        /**
         * Callback for the face training results.
         */
        void armGoalReachedCallback(
                const std_msgs::Bool::ConstPtr &t_goalReached);
        
        /**
         * Publish a message to the arm goto location topic with the specified
         * location.
         *
         * @param t_location The location to publish.
         */
        void startArmGoToLocation(std::string t_location);

    public:
        /**
         * @brief Class constructor
         * 
         * Creates a new RobotArmTasks object.
         *
         * @param nh The ROS Node Handler of the calling node. If no node 
         * handler is provided then a new one will be created.
         * @param t_armGoToAnglesTopicName The topic to indicate that the arm
         * must be moved by a set of specified angles.
         * @param t_armGotoPoseWRTATopicName The name of the topic to move the
         * arm to a pose with respect to the arm.
         * @param t_armGoToPoseWRTRTopicName The name of the topic to move the
         * arm to a pose with respect to the robot.
         * @param t_armGoToLocationTopicName The name of the topic to move the arm to a 
         * predefined string location.
         * @param t_armMoveTopicName The name of the topic to move the arm.
         * @param t_armGoalReachedTopicName The name of the topic that indicates tath
         * the arm had reached the specified goal position.
         */
        RobotArmTasks(
                ros::NodeHandle *t_nh = 0,
                std::string t_armGoToAnglesTopicName = 
                "/manipulation/manip_pln/ra_goto_angles",
                std::string t_armGotoPoseWRTATopicName = 
                "/manipulation/manip_pln/ra_pose_wrt_arm",
                std::string t_armGoToPoseWRTRTopicName = 
                "/manipulation/manip_pln/ra_pose_wrt_robot",
                std::string t_armGoToLocationTopicName = 
                "/manipulation/manip_pln/ra_goto_loc",
                std::string t_armMoveTopicName = 
                "/manipulation/manip_pln/ra_move",
                std::string t_armGoalReachedTopicName = 
                "/manipulation/ra_goal_reached"
                );

        /**
         * @brief Initialize the communication of the object with ROS.
         *
         * @param t_nh[in] The ROS node handler of the calling node. If no node 
         * handler provided, the the node will create one later.
         */
        void initRosConnection(ros::NodeHandle *t_nh=0);

        /**
         * Indicates if the arm has reached its goal position.
         *
         * @return True if the goal was reached. False otherwise.
         */
        bool isGoalReached();

        /**
         * @brief Performs an asynchronous movement of the arm to an specified 
         * goal position defined by an string.
         *
         * This method does not block the calling thread.
         *
         * @param t_goalposition The goal position to reach. 
         */
        void asyncArmGoTo(std::string t_goalPosition);

        /**
         * @brief Performs a synchronous movement of the arm to an specified 
         * goal position defined by an string.
         *
         * This method blocks the calling thread until the arm reach the 
         * specified position or until it reachs a timeout.
         *
         * @param t_goalposition The goal position to reach. 
         * @param t_timeout The limit time to accomplish the task.
         * @return True if the arm reach the specified location. Flase 
         * otherwise.
         */
        bool syncArmGoTo(std::string t_goalPosition, int t_timeout);
};
#endif
