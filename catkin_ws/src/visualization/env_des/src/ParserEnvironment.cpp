#include <env_des/ParserEnvironment.hpp>

std::vector<Model> ParserEnvironment::parser(std::string filePath, std::string modelsPath)
{
    YAML::Node configWrl = YAML::LoadFile(filePath);
    YAML::Node world = configWrl["world"];

    if(world.IsNull() && !world.IsDefined()){
        std::cout << nodeName << "Cant not load the config file environment descripcion YAML." << std::endl;
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

        /*std::cout << "World type:" << modelWrl.type << std::endl;
        std::cout << "x:" << modelWrl.pose.position.x << std::endl;
        std::cout << "y:" << modelWrl.pose.position.y << std::endl;
        std::cout << "z:" << modelWrl.pose.position.z << std::endl;
        std::cout << "ex:" << modelWrl.pose.orientation.x << std::endl;
        std::cout << "ey:" << modelWrl.pose.orientation.y << std::endl;
        std::cout << "ez:" << modelWrl.pose.orientation.z << std::endl;
        std::cout << "ew:" << modelWrl.pose.orientation.w << std::endl;*/

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
    return wrlModels;
}
