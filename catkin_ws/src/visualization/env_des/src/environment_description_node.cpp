#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>

#include <env_des/ParserEnvironment.hpp>
#include <env_msgs/AddUpdateObjectViz.h>
#include "tf/transform_listener.h"

#include <visualization_msgs/MarkerArray.h>

std::string node_log_name = "environment_description_node.->";

std::map<std::string, visualization_msgs::Marker> cubesMapMarker;
visualization_msgs::MarkerArray markerArray;
visualization_msgs::MarkerArray objectMarker;

tf::TransformListener * transformListener;

bool addUpdateObject(env_msgs::AddUpdateObjectViz::Request &req, env_msgs::AddUpdateObjectViz::Response &resp){

    std::map<std::string, visualization_msgs::Marker>::iterator cubeIt = cubesMapMarker.find(req.object.id.data);
    tf::StampedTransform transform;
    transformListener->waitForTransform(req.object.frame_goal.data, req.object.frame_original.data, ros::Time(0), ros::Duration(10.0));
    transformListener->lookupTransform(req.object.frame_goal.data, req.object.frame_original.data, ros::Time(0), transform);

    tf::Vector3 cubeCentroidOri(req.object.centroid.x, req.object.centroid.y, req.object.centroid.z);
    tf::Vector3 cubeCentroidGoal = transform * cubeCentroidOri;
    tf::Vector3 cubeMinOri(req.object.minPoint.x, req.object.minPoint.y, req.object.minPoint.z);
    tf::Vector3 cubeMinGoal = transform * cubeMinOri;
    tf::Vector3 cubeMaxOri(req.object.maxPoint.x, req.object.maxPoint.y, req.object.maxPoint.z);
    tf::Vector3 cubeMaxGoal = transform * cubeMaxOri;
    if(cubeIt == cubesMapMarker.end()){
        visualization_msgs::Marker marker;
        marker.header.frame_id = req.object.frame_goal.data;
        marker.header.stamp = ros::Time();
        marker.ns = "cubes_marker";
        marker.id = cubesMapMarker.size();
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = cubeCentroidGoal.x();
        marker.pose.position.y = cubeCentroidGoal.y();
        marker.pose.position.z = cubeCentroidGoal.z();
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        /*marker.scale.x = fabs(cubeMinGoal.x() - cubeMaxGoal.x());
        marker.scale.y = fabs(cubeMinGoal.y() - cubeMaxGoal.y());
        marker.scale.z = fabs(cubeMinGoal.z() - cubeMaxGoal.z());*/
        marker.scale.x = 0.09;
        marker.scale.y = 0.08;
        marker.scale.z = 0.08;
        marker.color.a = 0.8;
        marker.color.r = req.object.color.x;
        marker.color.g = req.object.color.y;
        marker.color.b = req.object.color.z;
        cubesMapMarker[req.object.id.data] = marker; 
    }
    else{
        visualization_msgs::Marker marker = cubeIt->second;
        marker.header.frame_id = req.object.frame_goal.data;
        marker.pose.position.x = cubeCentroidGoal.x();
        marker.pose.position.y = cubeCentroidGoal.y();
        marker.pose.position.z = cubeCentroidGoal.z();
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        if(req.object.frame_goal.data.compare("left_arm_grip_center") == 0 || req.object.frame_goal.data.compare("right_arm_grip_center") == 0)
            marker.pose.orientation.y = 1.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        /*marker.scale.x = fabs(cubeMinGoal.x() - cubeMaxGoal.x());
          marker.scale.y = fabs(cubeMinGoal.y() - cubeMaxGoal.y());
          marker.scale.z = fabs(cubeMinGoal.z() - cubeMaxGoal.z());
          marker.color.r = req.object.color.x;
          marker.color.g = req.object.color.y;
          marker.color.b = req.object.color.z;*/
        cubesMapMarker[req.object.id.data] = marker; 
    }
}

int main(int argc, char ** argv){

    ros::init(argc, argv, "environment_description_node");
    ros::NodeHandle nh;
    ros::Rate rate(30);

    std::string configFile = "/opt/codigo/JUSTINA/catkin_ws/src/visualization/env_des/config/bioroboanexo_config.yaml";
    std::string modelsPath = "/opt/codigo/JUSTINA/catkin_ws/src/visualization/env_des/models/";

    if(ros::param::has("~configFile")){
        ros::param::get("~configFile", configFile);
    }
    if(ros::param::has("~modelsPath")){
        ros::param::get("~modelsPath", modelsPath);
    }

    ros::Publisher pubEnvMarker = nh.advertise<visualization_msgs::MarkerArray>("environment_description", 1);
    ros::ServiceServer service = nh.advertiseService("object_description", addUpdateObject);
    ros::Publisher pubCubesMarker = nh.advertise<visualization_msgs::MarkerArray>("cubes_segmentation/cubes_markers", 1);
    // ros::Subscriber subAddTableByHight = nh.subscribe("add_table_hight", 1, callback);

    ParserEnvironment pe("environment_description.->");
    std::vector<Model> wrlModels = pe.parser(configFile, modelsPath);

    transformListener = new tf::TransformListener();

    while(ros::ok()){

        objectMarker.markers.clear();
        for(std::map<std::string, visualization_msgs::Marker>::iterator it = cubesMapMarker.begin(); it != cubesMapMarker.end(); it++)
            objectMarker.markers.push_back(it->second);
        pubCubesMarker.publish(objectMarker);
        
        int id = 0;
        for(int i = 0; i < wrlModels.size(); i++){
            Model wrlModel = wrlModels[i];
            tf::Transform wrlT;
            wrlT.setOrigin(tf::Vector3(wrlModel.pose.position.x, wrlModel.pose.position.y, wrlModel.pose.position.z));
            tf::Quaternion qS;
            qS.setX(wrlModel.pose.orientation.x);
            qS.setY(wrlModel.pose.orientation.y);
            qS.setZ(wrlModel.pose.orientation.z);
            qS.setW(wrlModel.pose.orientation.w);
            wrlT.setRotation(qS);
            // std::cout << "x:" << wrlModel.pose.position.x << std::endl;
            // std::cout << "y:" << wrlModel.pose.position.y << std::endl;
            // std::cout << "z:" << wrlModel.pose.position.z << std::endl;
            // std::cout << "ex:" << wrlModel.pose.orientation.x << std::endl;
            // std::cout << "ey:" << wrlModel.pose.orientation.y << std::endl;
            // std::cout << "ez:" << wrlModel.pose.orientation.z << std::endl;
            // std::cout << "ew:" << wrlModel.pose.orientation.w << std::endl;
            for(int j = 0; j < wrlModel.models.size(); j++){
                Model model = wrlModel.models[j];
                tf::Transform modelT;
                modelT.setOrigin(tf::Vector3(model.pose.position.x, model.pose.position.y, model.pose.position.z));
                tf::Quaternion qS;
                qS.setX(model.pose.orientation.x);
                qS.setY(model.pose.orientation.y);
                qS.setZ(model.pose.orientation.z);
                qS.setW(model.pose.orientation.w);
                modelT.setRotation(qS);
                // std::cout << "x:" << model.pose.position.x << std::endl;
                // std::cout << "y:" << model.pose.position.y << std::endl;
                // std::cout << "z:" << model.pose.position.z << std::endl;
                // std::cout << "ex:" << model.pose.orientation.x << std::endl;
                // std::cout << "ey:" << model.pose.orientation.y << std::endl;
                // std::cout << "ez:" << model.pose.orientation.z << std::endl;
                // std::cout << "ew:" << model.pose.orientation.w << std::endl;
                for(int k = 0; k < model.models.size(); k++){

                    Model shape = model.models[k];
                    tf::Transform shapeT;
                    shapeT.setOrigin(tf::Vector3(shape.pose.position.x, shape.pose.position.y, shape.pose.position.z));
                    tf::Quaternion qS;
                    qS.setX(shape.pose.orientation.x);
                    qS.setY(shape.pose.orientation.y);
                    qS.setZ(shape.pose.orientation.z);
                    qS.setW(shape.pose.orientation.w);
                    shapeT.setRotation(qS);
                    // std::cout << "x:" << shape.pose.position.x << std::endl;
                    // std::cout << "y:" << shape.pose.position.y << std::endl;
                    // std::cout << "z:" << shape.pose.position.z << std::endl;
                    // std::cout << "ex:" << shape.pose.orientation.x << std::endl;
                    // std::cout << "ey:" << shape.pose.orientation.y << std::endl;
                    // std::cout << "ez:" << shape.pose.orientation.z << std::endl;
                    // std::cout << "ew:" << shape.pose.orientation.w << std::endl;

                    tf::Transform gT = wrlT.inverse() * modelT * shapeT;
                    // tf::Transform gT = shapeT;

                    visualization_msgs::Marker marker;
                    marker.header.frame_id = "map";
                    marker.header.stamp = ros::Time();
                    marker.ns = "environment_description";
                    marker.id = id++;
                    marker.type = shape.shape;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.pose.position.x = gT.getOrigin().getX();
                    marker.pose.position.y = gT.getOrigin().getY();
                    marker.pose.position.z = gT.getOrigin().getZ();
                    marker.pose.orientation.x = gT.getRotation().getX();
                    marker.pose.orientation.y = gT.getRotation().getY();
                    marker.pose.orientation.z = gT.getRotation().getZ();
                    marker.pose.orientation.w = gT.getRotation().getW();
                    marker.scale.x = shape.size.data[0];
                    marker.scale.y = shape.size.data[1];
                    marker.scale.z = shape.size.data[2];
                    marker.color.a = 1.0; // Don't forget to set the alpha!
                    marker.color.r = shape.color.data[0];
                    marker.color.g = shape.color.data[1];
                    marker.color.b = shape.color.data[2];
                    /*std::cout << "x:" << gT.getOrigin().getX() << std::endl;
                      std::cout << "y:" << gT.getOrigin().getY() << std::endl;
                      std::cout << "z:" << gT.getOrigin().getZ() << std::endl;
                      std::cout << "ex:" << gT.getRotation().getX() << std::endl;
                      std::cout << "ey:" << gT.getRotation().getY() << std::endl;
                      std::cout << "ez:" << gT.getRotation().getZ() << std::endl;
                      std::cout << "ew:" << gT.getRotation().getW() << std::endl;*/

                    markerArray.markers.push_back(marker);
                }
            }
        }

        pubEnvMarker.publish(markerArray);
        rate.sleep();
        ros::spinOnce();
        markerArray.markers.clear();
    }

    return 1;
}
