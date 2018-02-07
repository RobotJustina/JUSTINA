#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/tf.h>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <stdexcept>

#include <visualization_msgs/MarkerArray.h>

std::string node_log_name = "environment_description_node.->";

typedef struct _Model {
    std::string type;
    std::string id;
    int shape;
    geometry_msgs::Pose pose;
    std_msgs::Float32MultiArray color;
    std_msgs::Float32MultiArray size;
    std::vector<_Model> models;
} Model;

int main(int argc, char ** argv){

    ros::init(argc, argv, "environment_description_node");
    ros::NodeHandle nh;
    ros::Rate rate(30);

    std::string configFile = "/opt/codigo/JUSTINA/catkin_ws/src/visualization/environment_description/config/bioroboanexo_config.yaml";
    std::string modelsPath = "/opt/codigo/JUSTINA/catkin_ws/src/visualization/environment_description/models/";

    if(ros::param::has("~configFile")){
        ros::param::get("~configFile", configFile);
    }
    if(ros::param::has("~modelsPath")){
        ros::param::get("~modelsPath", modelsPath);
    }

    ros::Publisher pubEnvMarker = nh.advertise<visualization_msgs::MarkerArray>("environment_description", 1);

    YAML::Node configWrl = YAML::LoadFile(configFile);
    YAML::Node world = configWrl["world"];
    
    if(world.IsNull() && !world.IsDefined()){
        std::cout << node_log_name << "Cant not load the config file environment descripcion YAML." << std::endl;
        throw;
    }

    std::vector<Model> wrlModels;

    for(YAML::const_iterator seq = world.begin(); seq != world.end(); seq++){
        YAML::Node nodeWrl = *seq;
        YAML::Node nodeWrlPose = nodeWrl["pose"];

        Model modelWrl;

        modelWrl.type = nodeWrl["type"].as<std::string>();
        modelWrl.pose.position.x = nodeWrlPose["x"].as<double>();
        modelWrl.pose.position.y = nodeWrlPose["y"].as<double>();
        modelWrl.pose.position.z = nodeWrlPose["z"].as<double>();

        YAML::Node nodeTheta = nodeWrlPose["t"];
        tf::Quaternion q;
        if(!nodeTheta.IsNull() && nodeTheta.IsDefined())
            q.setRotation(tf::Vector3(0.0f, 0.0f, 1.0f), nodeTheta.as<double>());
        else{
            q.setX(0.0f);
            q.setY(0.0f);
            q.setZ(0.0f);
            q.setW(1.0f);
        }

        modelWrl.pose.orientation.x = q.x();
        modelWrl.pose.orientation.y = q.y();
        modelWrl.pose.orientation.z = q.z();
        modelWrl.pose.orientation.w = q.w();
                    
        std::cout << "World type:" << modelWrl.type << std::endl;
        std::cout << "x:" << modelWrl.pose.position.x << std::endl;
        std::cout << "y:" << modelWrl.pose.position.y << std::endl;
        std::cout << "z:" << modelWrl.pose.position.z << std::endl;
        std::cout << "ex:" << modelWrl.pose.orientation.x << std::endl;
        std::cout << "ey:" << modelWrl.pose.orientation.y << std::endl;
        std::cout << "ez:" << modelWrl.pose.orientation.z << std::endl;
        std::cout << "ew:" << modelWrl.pose.orientation.w << std::endl;

        std::stringstream ss;
        ss << modelsPath << modelWrl.type << "/model.yaml";
        std::cout << ss.str() << std::endl;
        std::ifstream infile(ss.str()); 

        if(infile.good()){
            YAML::Node configModel = YAML::LoadFile(ss.str());

            if(!configModel.IsNull()){
                for(YAML::const_iterator seq = configModel.begin(); seq != configModel.end(); seq++){
                    YAML::Node nodeModel = *seq;
                    YAML::Node nodeModelPose = nodeModel["pose"];

                    Model model;

                    model.type = nodeModel["type"].as<std::string>(); 
                    model.id = nodeModel["id"].as<std::string>();
                    model.pose.position.x = nodeModelPose["x"].as<double>();
                    model.pose.position.y = nodeModelPose["y"].as<double>();
                    model.pose.position.z = nodeModelPose["z"].as<double>();

                    YAML::Node nodeModelTheta = nodeModelPose["t"];
                    tf::Quaternion q;
                    if(!nodeModelTheta.IsNull() && nodeModelTheta.IsDefined())
                        q.setRotation(tf::Vector3(0.0f, 0.0f, 1.0f), nodeModelTheta.as<double>());
                    else{
                        q.setX(0.0f);
                        q.setY(0.0f);
                        q.setZ(0.0f);
                        q.setW(1.0f);
                    }

                    model.pose.orientation.x = q.x();
                    model.pose.orientation.y = q.y();
                    model.pose.orientation.z = q.z();
                    model.pose.orientation.w = q.w();

                    std::cout << "World type:" << model.type << std::endl;
                    std::cout << "x:" << model.pose.position.x << std::endl;
                    std::cout << "y:" << model.pose.position.y << std::endl;
                    std::cout << "z:" << model.pose.position.z << std::endl;
                    std::cout << "ex:" << model.pose.orientation.x << std::endl;
                    std::cout << "ey:" << model.pose.orientation.y << std::endl;
                    std::cout << "ez:" << model.pose.orientation.z << std::endl;
                    std::cout << "ew:" << model.pose.orientation.w << std::endl;


                    ss.str("");
                    ss <<modelsPath << model.type << "/model.yaml";
                    std::cout << ss.str() << std::endl;
                    std::ifstream infile(ss.str()); 

                    if(infile.good()){
                        YAML::Node configModelShape = YAML::LoadFile(ss.str());
                        YAML::Node nodeColor = configModelShape["color"];
                        YAML::Node nodeShapes = configModelShape["models"];

                        std_msgs::Float32MultiArray color;
                        color.data.push_back(nodeColor["r"].as<float>());
                        color.data.push_back(nodeColor["g"].as<float>());
                        color.data.push_back(nodeColor["b"].as<float>());
                    
                        std::cout << "r:" << color.data[0] << std::endl;
                        std::cout << "g:" << color.data[1] << std::endl;
                        std::cout << "b:" << color.data[2] << std::endl;
                       
                        for(YAML::const_iterator seqS = nodeShapes.begin(); seqS != nodeShapes.end(); seqS++){
                            YAML::Node nodeShape = *seqS;
                            YAML::Node nodeShapePose = nodeShape["pose"];
                            YAML::Node nodeShapeSize = nodeShape["size"];

                            Model modelShape;
                            modelShape.color = color;

                            std::string typeShape = nodeShape["shape"].as<std::string>();
                            if(typeShape.compare("box") == 0)
                                modelShape.shape = 1;
                            if(typeShape.compare("sphere") == 0)
                                modelShape.shape = 2;
                            if(typeShape.compare("cylinder") == 0)
                                modelShape.shape = 3;

                            std_msgs::Float32MultiArray size;
                            size.data.push_back(nodeShapeSize["x"].as<float>());
                            size.data.push_back(nodeShapeSize["y"].as<float>());
                            size.data.push_back(nodeShapeSize["z"].as<float>());
                            modelShape.size = size;
                            modelShape.pose.position.x = nodeShapePose["x"].as<float>(); 
                            modelShape.pose.position.y = nodeShapePose["y"].as<float>(); 
                            modelShape.pose.position.z = nodeShapePose["z"].as<float>(); 
                            
                            YAML::Node nodeShapeTheta = nodeShapePose["t"];
                            tf::Quaternion q;
                            if(!nodeShapeTheta.IsNull() && nodeShapeTheta.IsDefined())
                                q.setRotation(tf::Vector3(0.0f, 0.0f, 1.0f), nodeShapeTheta.as<double>());
                            else{
                                q.setX(0.0f);
                                q.setY(0.0f);
                                q.setZ(0.0f);
                                q.setW(1.0f);
                            }
                            
                            modelShape.pose.orientation.x = q.x();
                            modelShape.pose.orientation.y = q.y();
                            modelShape.pose.orientation.z = q.z();
                            modelShape.pose.orientation.w = q.w();
                            model.models.push_back(modelShape);
                        }
                    }
                    modelWrl.models.push_back(model);
                }
            }
        }
        
        wrlModels.push_back(modelWrl);
    }

    while(ros::ok()){

        visualization_msgs::MarkerArray markerArray;
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

                    tf::Transform gT = wrlT * modelT * shapeT;
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
