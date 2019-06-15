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
std::string categoriesFilePath;
std::string peopleFilePath;

bool srvLocationPath(knowledge_msgs::kdbFilePath::Request &req, knowledge_msgs::kdbFilePath::Response &res){
    
    std::cout << "location server"<< std::endl;
    res.kdb_file_path = locationsFilePath; 
    return true;
}

bool srvObjectPath(knowledge_msgs::kdbFilePath::Request &req, knowledge_msgs::kdbFilePath::Response &res){
    
    std::cout << "objects server"<< std::endl;
    res.kdb_file_path = objectsFilePath; 
    return true;
}

bool srvCategoryPath(knowledge_msgs::kdbFilePath::Request &req, knowledge_msgs::kdbFilePath::Response &res){
    
    std::cout << "categories server"<< std::endl;
    res.kdb_file_path = categoriesFilePath; 
    return true;
}

bool srvPeoplePath(knowledge_msgs::kdbFilePath::Request &req, knowledge_msgs::kdbFilePath::Response &res){
    
    std::cout << "people server"<< std::endl;
    res.kdb_file_path = peopleFilePath; 
    return true;
}


int main(int argc, char ** argv) {

    std::cout << "Node for set kdb paths" << std::endl;

    
    ros::init(argc, argv, "kdb_node");
    ros::NodeHandle nh;

    ros::Rate rate(10);

    locationsFilePath = "";
    for (int i = 0; i < argc; i++) {
        std::string strParam(argv[i]);
        if (strParam.compare("-l") == 0)
            locationsFilePath = argv[++i];
    }

    objectsFilePath = "";
    for (int i = 0; i < argc; i++) {
        std::string strParam(argv[i]);
        if (strParam.compare("-o") == 0)
            objectsFilePath = argv[++i];
    }
    
    categoriesFilePath = "";
    for (int i = 0; i < argc; i++) {
        std::string strParam(argv[i]);
        if (strParam.compare("-c") == 0)
            categoriesFilePath = argv[++i];
    }

    peopleFilePath = "";
    for (int i = 0; i < argc; i++) {
        std::string strParam(argv[i]);
        if (strParam.compare("-p") == 0)
            peopleFilePath = argv[++i];
    }

    ros::ServiceServer serviceLocationPath = nh.advertiseService("/knowledge_representation/getLocationPath", srvLocationPath);
    ros::ServiceServer serviceObjectPath = nh.advertiseService("/knowledge_representation/getObjectPath", srvObjectPath);
    ros::ServiceServer serviceCategoryPath = nh.advertiseService("/knowledge_representation/getCategoryPath", srvCategoryPath);
    ros::ServiceServer servicePeoplePath = nh.advertiseService("/knowledge_representation/getPeoplePath", srvPeoplePath);

    while (ros::ok()) {

        rate.sleep();
        ros::spinOnce();

    }
   
   return 1; 
}
