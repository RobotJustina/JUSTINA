/**
 * @class SensorsTasks
 * @brief Class to perform tasks related to the robot's sensors (laser, kinect,
 * etc).
 *
 * The tasks you can do with this class are those like:
 *  - Record a RGBD video.
 *  - Get an RGBD image from kinect.
 *  - Store a lasser lecture on a file.
 *
 * @author R. Nonato Lagunas (nonato)
 * @author Marco Negrete
 * @version 0.1
*/
#ifndef _JUSTINA_SENSORSTASKS_H_
#define _JUSTINA_SENSORSTASKS_H_
#include <string>
#include <vector>
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/PointCloud2.h"
#include "point_cloud_manager/GetRgbd.h"
#include "ros/ros.h"

class SensorsTasks
{
    private:

        ros::NodeHandle *m_nh; /**< Stores the nodehandler used for the 
                                 subscriptions to topics/services. */

        bool m_connectionInitialized; /**< Indicates if the object is 
                                        conncected with ROS */

        std::string m_srvGetKinnectRGBDName; /**< Stores the name of the 
                                               service used to get a rgbd cloud
                                               with respect to kinnect. */

        std::string m_srvGetRobotRGBDName; /**< Stores the name of th service 
                                              used to get an rgbd cloud with
                                              respect to robot. */

        std::string m_saveCloudTopicName; /**< Stores the name of the save 
                                            cloud topic. */

        std::string m_stopSavingCloudTopicName; /**< Stores the name of the 
                                                  stop saving cloud topic. */

        ros::ServiceClient m_srvGetKinnectRGBD; /**< Service client ros object 
                                                  to call to the get rgbd with
                                                  respect to kinnect service.*/

        ros::ServiceClient m_srvGetRobotRGBD; /**< Service client ros object
                                                to call to the get rgbd with
                                                respect to robot service. */

        ros::Publisher m_pubSaveCloudTopic; /**< Publisher ros object to 
                                              publish data on the save cloud 
                                              topic. */

        ros::Publisher m_pubStopSavingCloud; /**< Publisher ros object to 
                                               publish data on the stop saving
                                               cloud topic. */

        /**
         * @brief Method to subscribe to the topics.
         */
        void prepareRosConnection();

    public:
        /**
         * @brief Class constructor
         * 
         * Creates a new SensorsTasks object.
         *
         * @param t_nh[in] The ROS Node Handler of the calling node. If no node 
         * handler is provided then a new one will be created.
         * @param t_srvGetKinnectRGBDName The name of the get rgbd with respect 
         * to kinnect service.
         * @param t_srvGetRobotRGBDName The name of the get rgbd with respect 
         * to robot service.
         * @param t_saveCloudTopicName The name of the save cloud topic.
         * @param t_stopSavingCloudTopicName The name of the stop saving cloud 
         * topic.
         */
        SensorsTasks( 
                ros::NodeHandle *t_nh = 0,
                std::string t_srvGetKinnectRGBDName = 
                  "/hardware/point_cloud_man/get_rgbd_wrt_kinect",
                std::string t_srvGetRobotRGBDName = 
                  "/hardware/point_cloud_man/get_rgbd_wrt_robot",
                std::string t_saveCloudTopicName = 
                  "/hardware/point_cloud_man/save_cloud",
                std::string t_stopSavingCloudTopicName = 
                  "/hardware/point_cloud_man/stop_saving_cloud"
                );

        /**
         * @brief Initialize the communication of the object with ROS.
         *
         * @param t_nh[in] The ROS node handler of the calling node. If no node 
         * handler provided, the the node will create one later.
         */
        void initRosConnection(ros::NodeHandle *t_nh=0);

        /**
         * @brief Gets a copy of a point cloud with respect to the kinnect 
         * sensor from the point coud manager node by calling the corresponding
         * service.
         *
         * @param t_pointCloud[out] Stores the obtained point cloud from the 
         * point cloud manager.
         * @return True if the point cloud was obtained succesfully, False
         * otherwise.
         */
        bool getKinnectRGBD(sensor_msgs::PointCloud2 &t_pointCloud);

        /**
         * @brief Gets a copy of a point cloud with respect to the robot from 
         * the point cloud manager node by calling the corresponding service.
         * 
         * @param t_pointCloud[out] Stores the obtained point cloud from the 
         * point cloud manager.
         * @return True if the point cloud was obtained succesfully, False
         * otherwise.
         */
        bool getRobotRGBD(sensor_msgs::PointCloud2 &t_pointCloud);

        /**
         * @brief Starts to store a point cloud to a external file by publish a 
         * value to the save cloud topic.
         *
         * @param t_fileName The name of the file where the point cloud will be
         * stored.
         * @return void
         */
        void startSavingCloud(std::string t_fileName);

        /**
         * @brief Stops the current saving cloud proccess by publish a value to
         * the stop saving cloud topic.
         *
         * @return void
         */
        void stopSavingCloud();
};
#endif
