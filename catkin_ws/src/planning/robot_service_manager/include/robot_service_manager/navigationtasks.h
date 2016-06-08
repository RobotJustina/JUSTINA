/**
 * @class NavigationTasks
 * @brief Contains several methods to perform navigation synchronous and 
 * asynchronous tasks.
 * Tasks that can be performed with this class includded (but not limited to):
 *  - Synchronous get close.
 *  - Asynchronous get close.
 *  - Sync/Asyn simple move.
 *  etc.
 *
 * @author R. Nonato Lagunas (nonato)
 * @author Marco Negrete
 * @version 0.1
*/
#ifndef _JUSTINA_NAVIGATIONTASKS_H_
#define _JUSTINA_NAVIGATIONTASKS_H_
#include <string>
#include "robot_service_manager/navigationstatus.h"
#include "robot_service_manager/sensorstasks.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/GetMap.h"
#include "navig_msgs/PlanPath.h"
#include "navig_msgs/PathFromMap.h"
#include "sensor_msgs/PointCloud2.h"
#include "ros/ros.h"
#include <boost/chrono.hpp>

class NavigationTasks
{
    private:

        ros::NodeHandle *m_nh; /**< Stores the nodehandler used for the 
                                 subscriptions to topics/services. */

        bool m_connectionInitialized; /**< Indicates if the object is 
                                        conncected with ROS */

        NavigationStatus m_navStatus;
        SensorsTasks m_sensorsTasks;

        std::string m_calcAStarPathFromMapSrvName; /**< Stores the name of the 
                                                     service used to calculate 
                                                     a path from the robot to 
                                                     the goal using the grid
                                                     map an the A* algorithm. 
                                                     */

        std::string m_calcWaveFrontPathFromMapSrvName; /**< Stores the name of 
                                                         the service used to 
                                                         calculate a path from 
                                                         the robot to goal 
                                                         using the grid map and
                                                         the wave fornt 
                                                         algorithm. */

        std::string m_planPathSrvName; /**< Stores the name of the
                                         service used to calculate 
                                         a path from the robot to 
                                         goal. */

        std::string m_getOccupancyGridSrvName; /**< Stores the name of the 
                                                 service used to get the 
                                                 occupancy grid map. */

        ros::ServiceClient m_getOccupancyGridSrv; /**< Service client ros 
                                                    object to call to the get 
                                                    occupancy grid service. */


        ros::ServiceClient m_calcAStarPathFromMapSrv; /**< Service client ros 
                                                        object to call to the 
                                                        calculate grid A* path 
                                                        service. */

        ros::ServiceClient m_calcWaveFrontPathFromMapSrv; /**< Service client
                                                            ros object to call
                                                            to the calculate
                                                            grid wave front 
                                                            path service. */

        ros::ServiceClient m_planPathSrv; /**< Service client ros 
                                            object to call to the 
                                            calculate grid-sensors 
                                            A* path service. */

        /**
         * @brief Method to subscribe to the topics.
         */
        void prepareRosConnection();

    public:
        /**
         * @brief Class constructor
         * 
         * Creates a new NavigationTasks object.
         *
         * @param t_nh[in] The ROS Node Handler of the calling node. If no node 
         * handler is provided then a new one will be created.
         */
        NavigationTasks(ros::NodeHandle *t_nh = 0,
                std::string t_getOccupancyGridServName = 
                "/navigation/localization/static_map",
                std::string t_calcAStarPathFromMapSrvName = 
                "/navigation/path_planning/path_calculator/a_star_from_map",
                std::string t_calcWaveFrontPathFromMapSrvName = 
                "/navigation/path_planning/path_calculator/wave_front_from_map",
                std::string t_planPathSrvName = 
                "/navigation/mvn_pln/plan_path"
                );

        /**
         * @brief Initialize the communication of the object with ROS.
         *
         * @param nh[in] The ROS node handler of the calling node. If no node 
         * handler provided, the the node will create one later.
         */
        void initRosConnection(ros::NodeHandle *t_nh=0);
        
        /**
         * @brief Blocks the calling thread until the robot reaches a 
         * previously defined goal or until timeout.
         *
         * @param t_timeout The limit time (milliseconds) limit to wait for the
         * robot to reaches the goal.
         * @return True if the robot reaches the goal in the constrained time.
         * False otherwise.
         */
        bool waitForGoalReached(int t_timeout=120000);

        /**
         * @brief Blocks the calling thread until the robot reaches the 
         * specified distance or until the execution time reaches a limit.
         *
         * @param t_distance The distance to move.
         * @param t_timeout The limit time for task's execition.
         */
        bool syncMove(float t_distance, int t_timeout);

        /**
         * @brief Blocks the calling thread until the robot reaches the 
         * specified distance and turn the specified angle or until the 
         * execution time reaches a limit.
         *
         * @param t_distance The distance to move.
         * @param t_angle The angle to turn.
         * @param t_timeout The limit time for task's execition.
         */
        bool syncMove(float t_distance, float t_angle, int t_timeout);

        /**
         * @brief Blocks the calling thread until the robot reaches the end of 
         * the specified path or until the execution time reaches a limit.
         *
         * @param[in] t_path The path to follow.
         * @param t_timeout The limit time for task's execition.
         */
        bool syncMove(nav_msgs::Path &t_path, int t_timeout);

        /**
         * @brief Blocks the calling thread until the robot reaches the 
         * specified pose or until the execution time reaches a limit.
         *
         * @param t_x The x robot's goal position.
         * @param t_y The y robot's goal position.
         * @param t_angle The angle robot's goal.
         * @param t_timeout The limit time for task's execition.
         */
        bool syncGoToPose(float t_x, float t_y, float t_angle, int t_timeout);

        /**
         * @brief Blocks the calling thread until the robot reaches the 
         * specified relative pose or until the execution time reaches a limit.
         *
         * @param t_relX The relative x robot's goal position.
         * @param t_relY The relative y robot's goal position.
         * @param t_relAngle The relative angle robot's goal.
         * @param t_timeout The limit time for task's execition.
         */
        bool syncGoToRelPose(float t_relX, float t_relY, float t_relAngle,
                int t_timeout);

        /**
         * @brief Blocks the calling thread until the robot reaches the 
         * specified (x,y) global position or until the execution time reaches
         * a limit.
         *
         * @param t_x The x goal position.
         * @param t_y The y goal position.
         * @param t_timeout The limit time for task's execution.
         */
        bool syncGetClose(float t_x, float t_y, int t_timeout);

        /**
         * @brief Blocks the calling thread until the robot reaches the 
         * specified (x,y, angle) global position or until the execution time 
         * reaches a limit.
         *
         * @param t_x The goal x position.
         * @param t_y The goal y position.
         * @param t_angle The goal angle.
         * @param t_timeout The limit time for task's execution.
         */
        bool syncGetClose(float t_x, float t_y, float t_angle, int t_timeout);

        /**
         * @brief Blocks the calling thread until the robot reaches the 
         * specified string location or until the execution time reaches a 
         * limit.
         *
         * @param t_location The location to reach.
         * @param t_timeout The limit time for task's execution.
         */
        bool syncGetClose(std::string t_location, int t_timeout);

        /**
         * @brief Obtains the occupancy grid from the global map by calling to 
         * the navigation node.
         *
         * @param[out] t_map The obtained occupancy grid.
         * @return True if the occupancy grid was obtained succesfully. False
         * otherwise.
         */
        bool getOccupancyGrid(nav_msgs::OccupancyGrid &t_map);

        /**
         * @brief Calculates a path from a designated start position to a
         * designated goal position using the A* algorithm and the grid map.
         *
         * @param t_startX The start x position.
         * @param t_startY The start y position.
         * @param t_goalX The goal x position.
         * @param t_goalY The goal y position.
         * @param t_path[out] The calculated path.
         * @return True if the path was calculated succesfully. False otherwise.
         */
        bool calcAStarPathFromMap(float t_startX, float t_startY, 
                float t_goalX, float t_goalY, nav_msgs::Path &t_path);

        /**
         * @brief Calculates a path from a designated start position to a
         * designated goal position using the wave front algorithm and the 
         * grid map.
         *
         * @param t_startX The start x position.
         * @param t_startY The start y position.
         * @param t_goalX The goal x position.
         * @param t_goalY The goal y position.
         * @param t_path[out] The calculated path.
         * @return True if the path was calculated succesfully. False otherwise.
         */
        bool calcWaveFrontPathFromMap(float t_startX, float t_startY, 
                float t_goalX, float t_goalY, nav_msgs::Path &t_path);

        /**
         * @brief Calculates a path from the robot current position to a
         * designated goal position using the A* algorithm and the grid map.
         *
         * @param t_goalX The goal x position.
         * @param t_goalY The goal y position.
         * @param t_path[out] The calculated path.
         * @return True if the path was calculated succesfully. False otherwise.
         */
        bool calcAStarPathFromMap(float t_goalX, float t_goalY, 
                nav_msgs::Path &t_path);

        /**
         * @brief Calculates a path from the robot current position to a
         * designated goal position using the wave front algorithm and the 
         * grid map.
         *
         * @param t_goalX The goal x position.
         * @param t_goalY The goal y position.
         * @param t_path[out] The calculated path.
         * @return True if the path was calculated succesfully. False otherwise.
         */
        bool calcWaveFrontPathFromMap(float t_goalX, float t_goalY, 
                nav_msgs::Path &t_path);

        bool planPath(float t_startX, float t_startY, float t_goalX, 
                float t_goalY, nav_msgs::Path &t_path);
        bool planPath(float t_goalX, float t_goalY, nav_msgs::Path &t_path);
        bool planPath(std::string t_startLocation, std::string t_goalLocation, 
                nav_msgs::Path &t_path);
        bool planPath(std::string t_goalLocation, nav_msgs::Path &t_path);
        bool planPath(std::string t_startLocation, float t_goalX, 
                float t_goalY, nav_msgs::Path &t_path);
        bool planPath(float t_startX, float t_startY, 
                std::string t_goalLocation, nav_msgs::Path &t_path);
};
#endif
