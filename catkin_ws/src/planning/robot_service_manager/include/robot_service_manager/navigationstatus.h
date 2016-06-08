/**
 * @class NavigationStatus
 * @brief Reads/modifies the status of the robot navigation system by 
 * subscribing to the corresponding status topics.
 *
 * @author R. Nonato Lagunas (nonato)
 * @author Marco Negrete
 * @version 0.1
*/
#ifndef _JUSTINA_NAVIGATIONSTATUS_H_
#define _JUSTINA_NAVIGATIONSTATUS_H_
#include <string>
#include <cmath>
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Path.h"
#include "tf/transform_listener.h"
#include "ros/ros.h"

class NavigationStatus
{
    private:

        ros::NodeHandle *m_nh; /**< Stores the nodehandler used for the 
                                 subscriptions to topics/services. */

        float m_currentPositionX; /**< Stores the current robot x position. */

        float m_currentPositionY; /**< Stores the current robot y position. */

        float m_currentPositionAngle; /**< Stores the current robot theta 
                                        position. */

        bool m_isGoalReached; /**< Updates its value with the value of the 
                                goal reached topic. */

        bool m_connectionInitialized; /**< Indicates if the object is 
                                        conncected with ROS */

        std::string m_goalReachedTopicName; /**< Stores the name of the topic 
                                              for goal reached indication. */

        std::string m_moveGoalDistTopicName; /**< Stores the name of the topic
                                               that stores the distance goal of
                                               a move action. */

        std::string m_moveGoalDistAngleTopicName; /**< Stores the name of the 
                                                    topic that stores the 
                                                    distance goal and angle of 
                                                    a move action. */

        std::string m_moveGoalPathTopicName; /**< Stores the name of the topic 
                                               that stores the path goal of a 
                                               move action. */

        std::string m_moveGoalPoseTopicName; /**< Stores the name of the topic 
                                               that stores the pose goal of a 
                                               move action. */

        std::string m_moveGoalRelPoseTopicName; /**< Stores the name of the 
                                                  topic that stores the 
                                                  relative pose goal of a move 
                                                  action. */

        std::string m_getCloseLocTopicName; /**< Stores the name of the topic 
                                               that stores the location goal of
                                               a get close action. */

        std::string m_getCloseXYATopicName; /**< Stores the name of the topic 
                                               that stores the XYA goal of
                                               a get close action. */

        std::string m_currentRobotPoseTopicName; /**< Stores the name of the 
                                                   topic that provides 
                                                   information of the current 
                                                   robot's pose. */

        ros::Subscriber m_goalReachedSub; /**< Stores the subscriber for the 
                                            goal reached topic. */

        ros::Subscriber m_currentRobotPoseSub; /**< Stores the subscriber for 
                                                 the current robot's pose 
                                                 topic. */

        ros::Publisher m_moveGoalDistPub; /**< Stores the publisher for the 
                                            goal dist simple-move topic. */

        ros::Publisher m_moveGoalDistAnglePub; /**< Stores the publisher for the
                                                 goal distance and angle topic.
                                                 */

        ros::Publisher m_moveGoalPathPub; /**< Stores the publisher for the 
                                            goal path topic. */

        ros::Publisher m_moveGoalPosePub; /**< Stores the publisher for the 
                                            goal pose topic. */

        ros::Publisher m_moveGoalRelPosePub; /**< Stores the publisher for the 
                                               goal relative pose topic. */

        ros::Publisher m_getCloseLocPub; /**< Stores the publisher for the 
                                           get close to location topic. */

        ros::Publisher m_getCloseXYAPub; /**< Stores the publisher for the 
                                           get close to XYA position topic. */

        tf::TransformListener* m_tfListener; /**< Stores the TF listener to 
                                                perform transformations. */

        /**
         * @brief Method to subscribe to the topics.
         */
        void prepareRosConnection();

        /**
         * @brief Callback to listen the updates of the goal reched topic.
         *
         * @param t_recMsg The received value fom the updated topic.
         * @return void
         */
        void goalReachedCallback(const std_msgs::Bool::ConstPtr &t_recMsg);

        /**
         * @brief Callback to listen the updates of the current robot pose 
         * topic.
         *
         * @param t_recMsg The received value fom the updated topic.
         * @return void
         */
        void currentRobotPoseCallback(
                const geometry_msgs::PoseWithCovarianceStamped::ConstPtr 
                &t_recMsg);

    public:
        /**
         * @brief Class constructor
         * 
         * Creates a new NavigationStatus object.
         *
         * @param t_nh[in] The ROS Node Handler of the calling node. If no node 
         * handler is provided then a new one will be created.
         * @param t_goalReachedTopicName The name of the goal reached topic.
         * @param t_moveGoalDistTopicName The name of the simple distance move
         * topic.
         * @param t_moveGoalDistAngleTopicName The name of the simple distance 
         * and angle move topic.
         * @param t_moveGoalPathTopicName The name of the simple following 
         * path move topic.
         * @param t_moveGoalPoseTopicName The name of the simple move to a pose
         * topic.
         * @param t_moveGoalRelPoseTopicName The name of the simple move to a 
         * realtive pose topic.
         * @param t_getCloseLocTopicName The name of the get close to a 
         * location by name topic.
         * @param t_getCloseXYATopicName The name of the get close to a (x, y, 
         * angle) position topic.
         * @param t_currentRobotPoseTopicName The name of the topic that manage 
         * information about the robot pose.
         */
        NavigationStatus( 
                ros::NodeHandle *t_nh = 0,
                std::string t_goalReachedTopicName = 
                "/navigation/goal_reached",
                std::string t_moveGoalDistTopicName = 
                "/navigation/path_planning/simple_move/goal_dist",
                std::string t_moveGoalDistAngleTopicName = 
                "/navigation/path_planning/simple_move/goal_dist_angle",
                std::string t_moveGoalPathTopicName = 
                "/navigation/path_planning/simple_move/goal_path",
                std::string t_moveGoalPoseTopicName = 
                "/navigation/path_planning/simple_move/goal_pose",
                std::string t_moveGoalRelPoseTopicName = 
                "/navigation/path_planning/simple_move/goal_rel_pose",
                std::string t_getCloseLocTopicName = 
                "/navigation/mvn_pln/get_close_loc",
                std::string t_getCloseXYATopicName = 
                "/navigation/mvn_pln/get_close_xya",
                std::string t_currentRobotPoseTopicName =
                "/navigation/localization/current_pose");

        /**
         * @brief Initialize the communication of the object with ROS.
         *
         * @param nh[in] The ROS node handler of the calling node. If no node 
         * handler provided, the the node will create one later.
         */
        void initRosConnection(ros::NodeHandle *t_nh=0);
        
        /**
         * @brief Returns the current robot's pose.
         *
         * @param[out] t_x The current robot x position.
         * @param[out] t_y The current robot y position.
         * @param[out] t_angle The current robot angle.
         * @return void.
         */
        void getCurrentPose(float &t_x, float &t_y, float &t_angle);

        /**
         * @brief Indicates if the robot has reached a goal (previously set).
         *
         * @return True if the robot has reached the goal. False otherwise.
         */
        bool isGoalReached();
        
        /**
         * @brief Change the goal distance status in order to start a move 
         * distance action.
         *
         * @param t_distance The distance to move.
         * @return void
         */
        void setGoalDist(float t_distance);

        /**
         * @brief Change the goal angle-distance status in order to start a 
         * move angle-distance action.
         *
         * @param t_distance The distance to move.
         * @param t_angle The angle to turn.
         * @return void
         */
        void setGoalDistAngle(float t_distance, float t_angle);

        /**
         * @brief Change the goal path status in order to start a move path
         * action.
         *
         * @param t_path The path to move.
         * @return void
         */
        void setGoalPath(nav_msgs::Path &t_path);

        /**
         * @brief Change the goal pose status in order to start a move pose
         * action.
         *
         * @param t_x The x goal coordinate.
         * @param t_y The y goal coordinate.
         * @param t_angle The angle goal coordinate.
         * @return void
         */
        void setGoalPose(float t_x, float t_y, float t_angle);

        /**
         * @brief Change the goal relative pose status in order to start a 
         * move to relative pose action.
         *
         * @param t_relX The relative x goal coordinate.
         * @param t_relY The relative y goal coordinate.
         * @param t_relAngle The relative angle goal coordinate.
         * @return void
         */
        void setGoalRelPose(float t_relX, float t_relY, float t_relAngle);

        /**
         * @brief Change the goal (x, y) status in order to start a get
         * close action.
         *
         * @param t_x The x goal coorditate to get close.
         * @param t_y The y goal coorditate to get close.
         * @return void
         */
        void setGetCloseGoal(float t_x, float t_y);

        /**
         * @brief Change the goal (x, y, angle) status in order to start a get
         * close action.
         *
         * @param t_x The x goal coorditate to get close.
         * @param t_y The y goal coorditate to get close.
         * @param t_angle The angle goal coorditate to get close.
         * @return void
         */
        void setGetCloseGoal(float t_x, float t_y, float t_angle);

        /**
         * @brief Change the goal location name status in order to start a get
         * close action.
         *
         * @param t_goalName The goal name to get close.
         * @return void
         */
        void setGetCloseGoal(std::string t_goalName);
};
#endif
