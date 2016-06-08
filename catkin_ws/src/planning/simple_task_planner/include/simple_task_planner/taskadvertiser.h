/**
 * @class TaskAdvertiser
 * @brief Class to advertise the simple tasks as ROS services.
 *
 * This class use the tasks libraries in order to advertise each (ideally) to
 * ROS services. For this, this clase uses the needed justina messages packages
 * such as: planning_msgs, hri_msgs, etc.
 *
 * @author R. Nonato Lagunas (nonato)
 * @version 0.1
*/
#ifndef _JUSTINA_TASKADVERTISER_H_
#define _JUSTINA_TASKADVERTISER_H_

#include <vector>
#include <utility>
#include "simple_task_planner/simpletasks.h"
#include "planning_msgs/wait_for_confirm.h"
#include "planning_msgs/wait_for_switch.h"
#include "planning_msgs/wait_for_command.h"
#include "planning_msgs/search_remember_face.h"
#include "planning_msgs/ask_store_name.h"
#include "planning_msgs/place_object.h"
#include "planning_msgs/find_person.h"
#include "planning_msgs/CFRParams.h"
#include "ros/ros.h"

class TaskAdvertiser
{
    private:

        ros::NodeHandle m_nh; /**< ROS node handler. */
        
        bool m_isAdvertising; /**< Indicates if the tasks are advertised. */

        SimpleTasks m_simpleTasks; /**< To call the simple tasks. */

        ros::ServiceServer m_askAndWaitForConfirmSrv; /**< ROS object to 
                                                        advertise the wait for 
                                                        confirmation task as a 
                                                        service. */

        ros::ServiceServer m_waitStartFollowSrv; /**< ROS object to advertise 
                                                   the wait for start follow 
                                                   task as a service. */

        ros::ServiceServer m_waitForCommandSrv; /**< ROS object to advertise 
                                                   the wait for command 
                                                   task as a service. */

        ros::ServiceServer m_rememberFaceSrv; /**< ROS object to advertise 
                                                the search and remember face 
                                                task as a service. */

        ros::ServiceServer m_waitForStartGuideSrv; /**< ROS object to advertise 
                                                     the wait for star guide 
                                                     command task as a service.
                                                     */

        ros::ServiceServer m_askForNameSrv; /**< ROS object to advertise 
                                              the ask and store name task 
                                              as a service. */
        
        ros::ServiceServer m_dropObjectSrv; /**< ROS object to advertise 
                                              the drop object task 
                                              as a service. */

        ros::ServiceServer m_placeObjectSrv; /**< ROS object to advertise 
                                              the place object task 
                                              as a service. */
        
        ros::ServiceServer m_findPersonInLocationSrv; /**< ROS object to 
                                            advertise the find a person in 
                                            place object as a service. */

        /**
         * @brief Callback for the Find Person in Location service.
         *
         * For more information about this task (the execution sequence and the
         * parameters) you must take a look at the documentation of the 
         * SimpleTasks::findPersonInLocation method and to the 
         * **find_person.srv** of the planning_msgs package.
         *
         * To perform this task on your node, you must make a call to the 
         * **find_person_in_location** service of the simple_task_planner node 
         * (be careful of the namespace!!!!).
         */
        bool findPersonInLocationCallback(
                planning_msgs::find_person::Request &req,
                planning_msgs::find_person::Response &resp
                );


        /**
         * @brief Callback for the Place Object service.
         *
         * For more information about this task (the execution sequence and the
         * parameters) you must take a look at the documentation of the 
         * SimpleTasks::placeObject method and to the 
         * **place_object.srv** of the planning_msgs package.
         *
         * To perform this task on your node, you must make a call to the 
         * **place_object** service of the simple_task_planner node 
         * (be careful of the namespace!!!!).
         */
        bool placeObjectCallback(
                planning_msgs::place_object::Request &req,
                planning_msgs::place_object::Response &resp
                );

        /**
         * @brief Callback for the Drop Object service.
         *
         * For more information about this task (the execution sequence and the
         * parameters) you must take a look at the documentation of the 
         * SimpleTasks::dropObject method and to the 
         * **place_object.srv** of the planning_msgs package.
         *
         * To perform this task on your node, you must make a call to the 
         * **drop_object** service of the simple_task_planner node 
         * (be careful of the namespace!!!!).
         */
        bool dropObjectCallback(
                planning_msgs::place_object::Request &req,
                planning_msgs::place_object::Response &resp
                );

        /**
         * @brief Callback for the Ask and Store Name service.
         *
         * For more information about this task (the execution sequence and the
         * parameters) you must take a look at the documentation of the 
         * SimpleTasks::askForName method and to the 
         * **ask_store_name.srv** of the planning_msgs package.
         *
         * To perform this task on your node, you must make a call to the 
         * **ask_store_name** service of the simple_task_planner node 
         * (be careful of the namespace!!!!).
         */
        bool askForNameCallback(
                planning_msgs::ask_store_name::Request &req,
                planning_msgs::ask_store_name::Response &resp
                );

        /**
         * @brief Callback for the Search and Remember Face service.
         *
         * For more information about this task (the execution sequence and the
         * parameters) you must take a look at the documentation of the 
         * SimpleTasks::faceSearchAndRemeber method and to the 
         * **search_remember_face.srv** of the planning_msgs package.
         *
         * To perform this task on your node, you must make a call to the 
         * **search_and_remember_face** service of the simple_task_planner node 
         * (be careful of the namespace!!!!).
         */
        bool rememberFaceCallback(
                planning_msgs::search_remember_face::Request &req,
                planning_msgs::search_remember_face::Response &resp
                );

        /**
         * @brief Callback for the Ask and Wait for Confirmation service.
         *
         * For more information about this task (the execution sequence and the
         * parameters) you must take a look at the documentation of the 
         * SimpleTasks::askAndWaitForConfirm method and to the 
         * **wait_for_confirm.srv** of the planning_msgs package.
         *
         * To perform this task on your node, you must make a call to the 
         * **wait_for_confirm** service of the simple_task_planner node (be 
         * careful of the namespace!!!!).
         */
        bool askAndWaitForConfirmCallback(
                planning_msgs::wait_for_confirm::Request &req,
                planning_msgs::wait_for_confirm::Response &resp
                );

        /**
         * @brief Callback for the Wait For Start Guide service. 
         *
         * For more information about this task (the execution sequence and the
         * parameters) you must take a look at the documentation of the 
         * SimpleTasks::waitForStartGuideCommand method and to the 
         * **wait_for_switch.srv** of the planning_msgs package.
         *
         * To perform this task on your node, you must make a call to the 
         * **wait_for_start_guide** service of the simple_task_planner node (be 
         * careful of the namespace!!!!).
         */
        bool waitForStartGuideCallback(
                planning_msgs::wait_for_switch::Request &req,
                planning_msgs::wait_for_switch::Response &resp
                );

        /**
         * @brief Callback for the Wait For Start Follow service. 
         *
         * For more information about this task (the execution sequence and the
         * parameters) you must take a look at the documentation of the 
         * SimpleTasks::waitForStartFollowCommand method and to the 
         * **wait_for_switch.srv** of the planning_msgs package.
         *
         * To perform this task on your node, you must make a call to the 
         * **wait_for_start_follow** service of the simple_task_planner node (be 
         * careful of the namespace!!!!).
         */
        bool waitStartFollowCallback(
                planning_msgs::wait_for_switch::Request &req,
                planning_msgs::wait_for_switch::Response &resp
                );

        /**
         * @brief Callback for the Wait For Command service. For more 
         * information about this task (the execution sequence and the 
         * parameters) you must take a look at the documentation
         * of the SimpleTasks::waitForCommand method and to the 
         * **wait_for_command.srv** of the planning_msgs package.
         *
         * To perform this task on your node, you must make a call to the 
         * **wait_for_command** service of the simple_task_planner node (be 
         * careful of the namespace!!!!).
         */
        bool waitForCommandCallback(
                planning_msgs::wait_for_command::Request &req,
                planning_msgs::wait_for_command::Response &resp
                );
    public:

        TaskAdvertiser(
                ros::NodeHandle t_nh = ros::NodeHandle("simple_task_planner")
                );

        /**
         * @brief Advertise the simple tasks as services.
         */
        void startAdvertising();

};

#endif
