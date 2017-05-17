#include <iostream>
#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Vector3.h>
#include "vision_msgs/DetectObjects.h"
#include "manip_msgs/InverseKinematicsFloatArray.h"

visualization_msgs::Marker centroid_marker, axis_list_marker;

bool markerSetup()
{

    centroid_marker.header.frame_id = "base_link";
    axis_list_marker.header.frame_id = "base_link";
    centroid_marker.header.stamp = ros::Time::now();
    axis_list_marker.header.stamp = ros::Time::now();
    centroid_marker.ns = "centroid";
    axis_list_marker.ns = "principal axis";
    centroid_marker.pose.orientation.w = 1.0;
    axis_list_marker.pose.orientation.w = 1.0;

    centroid_marker.id = 0;
    axis_list_marker.id = 1;

    centroid_marker.type = visualization_msgs::Marker::SPHERE;
    axis_list_marker.type = visualization_msgs::Marker::LINE_LIST;

    // POINTS markers use x and y scale for width/height respectively
    centroid_marker.scale.x = 0.035;
    centroid_marker.scale.y = 0.035;
    centroid_marker.scale.z = 0.035;

    axis_list_marker.scale.x = 0.03;
    axis_list_marker.scale.y = 0.03;
    axis_list_marker.scale.z = 0.03;

    centroid_marker.color.b = 1.0f;
    centroid_marker.color.a = 1.0;

    axis_list_marker.color.r = 1.0f;
    axis_list_marker.color.a = 1.0;

    return true;
}

visualization_msgs::Marker buildMarkerAxis(geometry_msgs::Vector3 PCA_axis_0,
                                          geometry_msgs::Vector3 PCA_axis_1,
                                          geometry_msgs::Vector3 PCA_axis_2,
                                          geometry_msgs::Pose centroid_pose)
{
    axis_list_marker.points.clear();
    geometry_msgs::Point px_centroid;
    geometry_msgs::Point p_0, p_1, p_2;

    px_centroid = centroid_pose.position;

    p_0.x = px_centroid.x + PCA_axis_0.x;
    p_0.y = px_centroid.y + PCA_axis_0.y;
    p_0.z = px_centroid.z + PCA_axis_0.z;

    p_1.x = px_centroid.x + PCA_axis_1.x;
    p_1.y = px_centroid.y + PCA_axis_1.y;
    p_1.z = px_centroid.z + PCA_axis_1.z;

    p_2.x = px_centroid.x + PCA_axis_2.x;
    p_2.y = px_centroid.y + PCA_axis_2.y;
    p_2.z = px_centroid.z + PCA_axis_2.z;

    axis_list_marker.points.push_back(px_centroid);
    axis_list_marker.points.push_back(p_0);

    axis_list_marker.points.push_back(px_centroid);
    axis_list_marker.points.push_back(p_1);

    axis_list_marker.points.push_back(px_centroid);
    axis_list_marker.points.push_back(p_2);

    return axis_list_marker;
}




int main(int argc, char** argv)
{
    std::cout << "INITIALIZING A TEST FOR GRASP OBJECT BY EDGAR-II..." << std::endl;
    ros::init(argc, argv, "grasp_pln");
    ros::NodeHandle n;

    std::vector<float> cartesian;

    vision_msgs::DetectObjects srv_detectObj;
    manip_msgs::InverseKinematicsFloatArray srv_ki;


    geometry_msgs::Pose centroid;
    geometry_msgs::Vector3 axis_resp_0, axis_resp_1, axis_resp_2;

    ros::ServiceClient cltDetectObjectsPCA;
    ros::ServiceClient cltInverseKinematics;

    ros::Publisher marker_pub;

    cltInverseKinematics = n.serviceClient<manip_msgs::InverseKinematicsFloatArray>("/manipulation/ik_geometric/ik_float_array");
    cltDetectObjectsPCA = n.serviceClient<vision_msgs::DetectObjects>("/detect_object/PCA_calculator");

    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);


    // Data request in format [x, y, z, roll, pitch, yaw, elbow]
    cartesian.push_back(0.0);
    cartesian.push_back(0.0);
    cartesian.push_back(0.0);
    cartesian.push_back(0.0);

    cartesian.push_back(0.2);
    cartesian.push_back(0.2);
    cartesian.push_back(0.2);
    srv_ki.request.cartesian_pose.data  = cartesian;

    markerSetup();

    ros::Rate loop(10);

    while(ros::ok())
    {
        if(!cltDetectObjectsPCA.call(srv_detectObj))
        {
            std::cout << std::endl << "Justina::Vision can't detect anything" << std::endl << std::endl;
            return false;
        }

        if(!cltInverseKinematics.call(srv_ki))
        {
            std::cout << std::endl << "Justina::Manip can't calling inverse kinematics service" << std::endl << std::endl;
            return false;
        }

        centroid = srv_detectObj.response.recog_objects[0].pose;
        axis_resp_0 = srv_detectObj.response.recog_objects[0].principal_axis[0];
        axis_resp_1 = srv_detectObj.response.recog_objects[0].principal_axis[1];
        axis_resp_2 = srv_detectObj.response.recog_objects[0].principal_axis[2];

        //std::cout << "Centroid: " << centroid.position << std::endl;
        //std::cout << std::endl;

        //std::cout << "Principal_axis: " << std::endl <<srv.response.recog_objects[0].principal_axis[0] << std::endl;
        //std::cout << srv.response.recog_objects[0].principal_axis[1] << std::endl;
        //std::cout << srv.response.recog_objects[0].principal_axis[2] << std::endl;

        centroid_marker.pose.position = centroid.position;
        marker_pub.publish(centroid_marker);
        axis_list_marker = buildMarkerAxis(axis_resp_0,
                    axis_resp_1,
                    axis_resp_2,
                    centroid);
        marker_pub.publish(axis_list_marker);


        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
