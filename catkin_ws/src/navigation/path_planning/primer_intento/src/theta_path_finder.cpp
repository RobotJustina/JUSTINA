#include <iostream>
#include "ros/ros.h"
#include "navig_msgs/PathFromMap.h"
#include "geometry_msgs/PoseStamped.h"
#include "tileadaptor.hpp"

//int mapSizeX = 10, mapSizeY = 10;

bool callbackThetaAlgorithm(navig_msgs::PathFromMap::Request &req, navig_msgs::PathFromMap::Response &resp)
{

    std::cout << "MONSE TEST.->Calculating path...." << std::endl;
    std::vector<std::vector<char> > map;

    std::cout << "Monse Test.->Resizing character map" << std::endl;
    map.resize(req.map.info.width);
    for(int i=0; i < map.size(); i++)
        map[i].resize(req.map.info.height);

    std::cout << "Monse Test.->Populatin map" <<  std::endl;
    for(int i=0; i < req.map.data.size(); i++)
    {
        int idx = i%req.map.info.width;
        int idy = i/req.map.info.width;
        if(req.map.data[i] > 50)
            map[idx][idy] = '#';
        else
            map[idx][idy] = ' ';
    }

    TileAdaptor adaptor({req.map.info.width, req.map.info.height}, [&map](const Vectori& vec){return map[vec.x][vec.y] != '#';});
    //TileAdaptor adaptor({mapSizeX, mapSizeY}, false);
    Pathfinder pathfinder(adaptor, 100.f /*weight*/);

    pathfinder.generateNodes();

    int startX = (int)(req.start_pose.position.x - req.map.info.origin.position.x)/req.map.info.resolution;
    int startY = (int)(req.start_pose.position.y - req.map.info.origin.position.y)/req.map.info.resolution;
    Vectori startPoint = {startX, startY};
    int endX = (int)(req.goal_pose.position.x - req.map.info.origin.position.x)/req.map.info.resolution;
    int endY = (int)(req.goal_pose.position.y - req.map.info.origin.position.y)/req.map.info.resolution;
    Vectori endPoint = {endX, endY};
    
    std::vector<uint32_t> nodePath = pathfinder.search(adaptor.posToId(startPoint), adaptor.posToId(endPoint));
    std::vector<Vectori> path;
    path.reserve(nodePath.size());

    for(const auto id : nodePath)
        path.push_back(adaptor.idToPos(id));

    for(int i=0; i < path.size(); i++)
        std::cout << path[i].x << "  " << path[i].y << std::endl;

    resp.path.poses.clear();
    resp.path.header.frame_id = "map";
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "map";
    for(int i=0; i < path.size(); i++)
    {
        
        p.pose.position.x = path[i].x * req.map.info.resolution + req.map.info.origin.position.x;
        p.pose.position.y = path[i].y * req.map.info.resolution + req.map.info.origin.position.y;
        resp.path.poses.push_back(p);
    }
    
    return true;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING THETA PATH FINDER BY MONSE..." << std::endl;
    ros::init(argc, argv, "theta_path_finder");
    ros::NodeHandle n;
    ros::ServiceServer srvPath = n.advertiseService("path_calculator/a_star_from_map", callbackThetaAlgorithm);
    ros::Rate loop(10);

    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
