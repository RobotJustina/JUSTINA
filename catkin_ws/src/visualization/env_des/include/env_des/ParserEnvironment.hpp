#include <geometry_msgs/Pose.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/tf.h>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <stdexcept>

typedef struct _Model {
    std::string type;
    std::string id;
    int shape;
    geometry_msgs::Pose pose;
    std_msgs::Float32MultiArray color;
    std_msgs::Float32MultiArray size;
    std::vector<_Model> models;
} Model;

class ParserEnvironment
{
    public:
        ParserEnvironment(std::string nodeName){
            this->nodeName = nodeName;
        }

        std::vector<Model> parser(std::string filePath, std::string modelsPath);
    private:
        std::string nodeName;
};
