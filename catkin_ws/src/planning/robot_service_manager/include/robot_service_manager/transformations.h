/**
 * @class Transformations
 * @brief Perform transformation by calling to the respective services.
 *
 * @author R. Nonato Lagunas (nonato)
 * @author Marco Negrete
 * @version 0.1
*/
#ifndef _JUSTINA_TRANSFORMATIONS_H_
#define _JUSTINA_TRANSFORMATIONS_H_
#include <string>
#include <vector>
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "manip_msgs/InverseKinematicsFloatArray.h"
#include "manip_msgs/InverseKinematicsPath.h"
#include "manip_msgs/InverseKinematicsPose.h"
#include "manip_msgs/DirectKinematics.h"
#include "ros/ros.h"

class Transformations
{
    private:

        ros::NodeHandle *m_nh; /**< Stores the nodehandler used for the 
                                 subscriptions to topics/services. */

        bool m_connectionInitialized; /**< Indicates if the object is 
                                        conncected with ROS */

        std::string m_IKFloatArrayServiceName; /**< Stores the name of the 
                                                 service which perform the 
                                                 float array inverse kinematics
                                                 */

        std::string m_IKPathServiceName; /**< Stores the name of the service
                                           which perform the path inverse 
                                           kinematics. */

        std::string m_IKPoseServiceName; /**< Stores the name of the service 
                                           which perform the pose inverse
                                           kinematics. */

        std::string m_DKServiceName; /**< Stores the name of the service which
                                       perform the direct kinematics. */

        ros::ServiceClient m_srvIKFloatArray; /**< Service client ros object to
                                                call the float array inverse 
                                                kinematics service. */

        ros::ServiceClient m_srvIKPath; /**< Service client ros object to call 
                                          the path inverse kinematics service. 
                                          */

        ros::ServiceClient m_srvIKPose; /**< Service client ros object to call 
                                          the pose inverse kinematics service. 
                                          */

        ros::ServiceClient m_srvDK; /**< Service client ros object to call 
                                      the direct kinematics service. */

        /**
         * @brief Method to subscribe to the topics.
         */
        void prepareRosConnection();

    public:

        /**
         * @brief Class constructor
         * 
         * Creates a new Transformations object.
         *
         * @param t_nh[in] The ROS Node Handler of the calling node. If no node 
         * handler is provided then a new one will be created.
         * @param t_IKFloatArrayServiceName The name of the float array inverse
         * kinematics service.
         * @param t_IKPathServiceName The name of the path inverse kinematics 
         * service.
         * @param t_IKPoseServiceName The name of the pose inverse kinematics 
         * service.
         * @param t_DKServiceName The name of the direct kinematics service.
         */
        Transformations( 
                ros::NodeHandle *t_nh = 0,
                std::string t_IKFloatArrayServiceName = 
                "/manipulation/ik_geometric/ik_float_array",
                std::string t_IKPathServiceName = 
                "/manipulation/ik_geometric/ik_path",
                std::string t_IKPoseServiceName =
                "/manipulation/ik_geometric/ik_pose",
                std::string t_DKServiceName = 
                "/manipulation/ik_geometric/direct_kinematics"
                );

        /**
         * @brief Initialize the communication of the object with ROS.
         *
         * @param t_nh[in] The ROS node handler of the calling node. If no node 
         * handler provided, the the node will create one later.
         */
        void initRosConnection(ros::NodeHandle *t_nh=0);
        
        /**
         * @brief Computes the inverse kinematics from cartesian to articular
         * by calling to the respective ros service. 
         *
         * @param t_cartesian[in] The cartesian coordinates.
         * @param t_articular[out] The resulting articular coordinates after 
         * the inverse kinematics computing.
         * @return True if the compute was doing succesfully, False otherwise.
         */
        bool inverseKinematics(std::vector<float> &t_cartesian, 
                std::vector<float> &t_articular);

        /**
         * @brief Computes the direct kinematics from cartesian to articular
         * by calling to the respective ros service. 
         *
         * @param t_cartesian[in] The resulting cartesian coordinates after
         * the direct kinematics computing.
         * @param t_articular[out] The articular coordinates.
         * @return True if the compute was doing succesfully, False otherwise.
         */
        bool directKinematics(std::vector<float> &t_cartesian, 
                std::vector<float> &t_articular);

};
#endif
