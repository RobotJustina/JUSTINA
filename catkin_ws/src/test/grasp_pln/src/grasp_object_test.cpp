#include <iostream>
#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include "vision_msgs/DetectObjects.h"

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

}




int main(int argc, char** argv)
{
    std::cout << "INITIALIZING A TEST FOR GRASP OBJECT BY EDGAR-II..." << std::endl;
    ros::init(argc, argv, "grasp_pln");
    ros::NodeHandle n;

    vision_msgs::DetectObjects srv;
    geometry_msgs::Pose centroid;

    ros::ServiceClient cltDetectObjectsPCA;
    ros::Publisher marker_pub;

    cltDetectObjectsPCA = n.serviceClient<vision_msgs::DetectObjects>("/detect_object/PCA_calculator");
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    markerSetup();

    ros::Rate loop(10);

    while(ros::ok())
    {
        if(!cltDetectObjectsPCA.call(srv))
        {
            std::cout << std::endl << "Justina::Vision can't detect anything" << std::endl << std::endl;
            return false;
        }

        centroid = srv.response.recog_objects[0].pose;

        std::cout << "Centroid: " << centroid.position << std::endl;
        std::cout << std::endl;

        std::cout << "Principal_axis: " << srv.response.recog_objects[0].principal_axis[0] << std::endl;
        std::cout << srv.response.recog_objects[0].principal_axis[1] << std::endl;
        std::cout << srv.response.recog_objects[0].principal_axis[2] << std::endl;


        centroid_marker.pose.position = centroid.position;
        marker_pub.publish(centroid_marker);
        marker_pub.publish(axis_list_marker);


        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
