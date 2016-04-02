#include <iostream>
#include <vector>
#include <climits>
#include <cmath>
#include "ros/ros.h"
#include "navig_msgs/PathFromMap.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "tf/tf.h"

nav_msgs::Path lastCalcPath;

bool callbackWaveFront(navig_msgs::PathFromMap::Request &req, navig_msgs::PathFromMap::Response &resp)
{
    //It is assumed that origin pose has an orientation of (0,0,0)
    int width = req.map.info.width;
    int height = req.map.info.height;
    float cellSize = req.map.info.resolution;
    float originX = req.map.info.origin.position.x;
    float originY = req.map.info.origin.position.y;
    float startX = req.start_pose.position.x;
    float startY = req.start_pose.position.y;
    float goalX = req.goal_pose.position.x;
    float goalY = req.goal_pose.position.y;
    int startCellX = (int)(startX - originX)/cellSize;
    int startCellY = (int)(startX - originX)/cellSize;
    int goalCellX = (int)(goalX - originX)/cellSize;
    int goalCellY = (int)(goalY - originY)/cellSize;
    int startCell = startCellY * width + startCellX;
    int goalCell = goalCellY * width + goalCellX;
    std::cout << "PathCalculator.->Calc path from " << startX << "  " << startY << " to " << goalX << "  " <<  goalY << std::endl;
    std::cout << "PathCalculator.->Start cell: " << startCell << "  Goal cell: " << goalCell << std::endl;
    //Cells in req.map have values in [0,100]. 0 are the completely free cells and 100 are the occupied ones.
    /*Cells are uint8 values, but, since map could be really big, wave_front can assign
     *values much greater than 255 to the cells, thus, we need a list of ints instead of uint8  to represent
     *the values assigned to each cell. 
    */
    //If the goal cell is an occupied one, then it will be not possible to find a path
    if(req.map.data[goalCell] > 50)
        return false;
    
    //First, we make all cells to have 0 for free space and 1 for the occupied one.
    std::vector<int> waveFrontMap;
    for(size_t i=0; i< req.map.data.size(); i++)
        waveFrontMap.push_back(req.map.data[i] < 50 ? 0 : 1);

    //We will use the BREATH-FIRST-SEARCH (BFS) algorithm to find the path.
    std::vector<int> fringe; //List of all nodes to be visited.
    fringe.push_back(goalCell); //We initialize the fringe with the root node. Wave front starts in the goal position
    waveFrontMap[goalCell] = 2; //Wave front starts by assigning 2 to the goal cell
    //Usually, BFS needs a list of visited nodes, but, since in the wave-front algorithm, all visited nodes
    //have values > 0, we don't need a visited-list, we just need to check if a cell has a value > 0.
    bool success = false;
    int attempts = 0;
    while(fringe.size() > 0 && !success) //While there are nodes to be visited
    {
        int currentCell = fringe[0];
        if(currentCell == startCell) //Wave front starts in the goal point and finishes when we reach the start point
            success = true;
        else
        {//Each current node has four children since we are using four-connectivity
            int westCell = currentCell - 1;
            int eastCell = currentCell + 1;
            int northCell = currentCell - width;
            int southCell = currentCell + width;
            if(waveFrontMap[westCell] == 0) //Check if cell==0 is equivalent to check if cell is NOT in the visited list
            {
                waveFrontMap[westCell] = waveFrontMap[currentCell] + 1; 
                fringe.push_back(westCell); //We put the westCell in the fringe to visit it in the next loop-cycle
            }
            if(waveFrontMap[eastCell] == 0) //Check if cell==0 is equivalent to check if cell is NOT in the visited list
            {
                waveFrontMap[eastCell] = waveFrontMap[currentCell] + 1; 
                fringe.push_back(eastCell); //We put the eastCell in the fringe to visit it in the next loop-cycle
            }
            if(waveFrontMap[northCell] == 0) //Check if cell==0 is equivalent to check if cell is NOT in the visited list
            {
                waveFrontMap[northCell] = waveFrontMap[currentCell] + 1; 
                fringe.push_back(northCell); //We put the northCell in the fringe to visit it in the next loop-cycle
            }
            if(waveFrontMap[southCell] == 0) //Check if cell==0 is equivalent to check if cell is NOT in the visited list
            {
                waveFrontMap[southCell] = waveFrontMap[currentCell] + 1; 
                fringe.push_back(southCell); //We put the southCell in the fringe to visit it in the next loop-cycle
            }
            fringe.erase(fringe.begin());
        }
        attempts++;
    }
    std::cout << "PathCalculator.-> Cannot find a path to goal pose :'(" << std::endl;
    if(!success) return false;
    //After assigning values to each cell, we get the path by gradient descend
    geometry_msgs::PoseStamped p;
    int currentCell = startCell;
    p.pose.position.x = (currentCell % width)*cellSize + originX;
    p.pose.position.y = (currentCell / width)*cellSize + originY;
    p.pose.orientation.w = 1;
    p.header.frame_id = "map";
    resp.path.poses.push_back(p);
    while(currentCell != goalCell)
    {
        //We will find the smallest value of the eight neighbor cells
        int minNeighbor = INT_MAX;
        int minNeighborCell = 0;
        std::vector<int> neighbors;
        neighbors.push_back(currentCell - width - 1);
        neighbors.push_back(currentCell - width);
        neighbors.push_back(currentCell - width + 1);
        neighbors.push_back(currentCell - 1);
        neighbors.push_back(currentCell + 1);
        neighbors.push_back(currentCell + width - 1);
        neighbors.push_back(currentCell + width);
        neighbors.push_back(currentCell + width + 1);
        for(size_t i=0; i < neighbors.size(); i++)
        {
            if(waveFrontMap[neighbors[i]] > 0 && waveFrontMap[neighbors[i]] < minNeighbor)
            {
                minNeighbor = waveFrontMap[neighbors[i]];
                minNeighborCell = neighbors[i];
            }
        }
        currentCell = minNeighborCell;
        p.pose.position.x = (currentCell % width)*cellSize + originX;
        p.pose.position.y = (currentCell / width)*cellSize + originY;
        resp.path.poses.push_back(p);
    }
    resp.path.header.frame_id = "map";
    std::cout << "PathCalculator.->Wave-front finished after " << attempts << " attempts" << std::endl;
    std::cout << "PathCalculator.->Calculated path has " << resp.path.poses.size() << " poses" << std::endl;
    lastCalcPath = resp.path;
    return success;
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING PATH CALCULATOR BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "path_calculator");
    ros::NodeHandle n;
    ros::ServiceServer srvPathWaveFront = n.advertiseService("path_calculator/wave_front", callbackWaveFront);
    ros::Publisher pubLastPath = n.advertise<nav_msgs::Path>("path_calculator/last_calc_path", 1);
    ros::Rate loop(10);

    while(ros::ok())
    {
        pubLastPath.publish(lastCalcPath);
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}

