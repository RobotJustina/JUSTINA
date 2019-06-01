#include <iostream>
#include <sstream>

#include "ros/ros.h"

#include "knowledge_msgs/kdbFilePath.h"

#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "std_msgs/ColorRGBA.h"

std::string locationsFilePath;
std::string objectsFilePath;
std::string categorysFilePath;
std::string peopleFilePath;

bool srvLocationPath(knowledge_msgs::kdbFilePath::Request &req, knowledge_msgs::kdbFilePath::Response &res){
    
    std::cout << "location server"<< std::endl;
    res.kdb_file_path = locationsFilePath; 
    return true;
}


int main(int argc, char ** argv) {

    std::cout << "Nodo para conseguir las rutas de los archivos" << std::endl;

    
    ros::init(argc, argv, "kdb_node");
    ros::NodeHandle nh;

    ros::Rate rate(10);

    std::string locationsFilePath = "";
    for (int i = 0; i < argc; i++) {
        std::string strParam(argv[i]);
        if (strParam.compare("-l") == 0)
            locationsFilePath = argv[++i];
    }

    std::string objectsFilePath = "";
    for (int i = 0; i < argc; i++) {
        std::string strParam(argv[i]);
        if (strParam.compare("-o") == 0)
            objectsFilePath = argv[++i];
    }
    
    std::string categorysFilePath = "";
    for (int i = 0; i < argc; i++) {
        std::string strParam(argv[i]);
        if (strParam.compare("-c") == 0)
            categorysFilePath = argv[++i];
    }

    std::string peopleFilePath = "";
    for (int i = 0; i < argc; i++) {
        std::string strParam(argv[i]);
        if (strParam.compare("-p") == 0)
            peopleFilePath = argv[++i];
    }

    ros::ServiceServer serviceLocationPath = nh.advertiseService("/knowledge_representation/getLocationPath", srvLocationPath);

    while (ros::ok()) {

        rate.sleep();
        ros::spinOnce();

    }
   
   return 1; 
}
