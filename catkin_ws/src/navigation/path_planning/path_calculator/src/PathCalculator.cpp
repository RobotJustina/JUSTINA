#include "PathCalculator.h"

nav_msgs::Path PathCalculator::WaveFront(navig_msgs::OccupancyGrid& map, geometry_msgs::Pose start, geometry_msgs::Pose goal, nav_msgs::Path& resultPath)
{
    int width = map.info.width;
    int height = map.info.height;
    float cellSize = map.info.resolution;
    float originX = map.info.origin.position.x;
    float originY = map.info.origin.position.y;
    float startX = start.position.x;
    float startY = start.position.y;
    float goalX = goal.position.x;
    float goalY = goal.position.y;
    int startCellX = (int)((startX - originX)/cellSize);
    int startCellY = (int)((startX - originX)/cellSize);
    int goalCellX = (int)((goalX - originX)/cellSize);
    int goalCellY = (int)((goalY - originY)/cellSize);
    int startCell = startCellY * width + startCellX;
    int goalCell = goalCellY * width + goalCellX;

    resultPath.header.frame_id = "map";
    resultPath.header.stamp = ros::Time::now();
    std::cout << "PathCalculator.->Calc path from " << startX << "  " << startY << " to " << goalX << "  " <<  goalY << std::endl;
    //std::cout << "PathCalculator.->Start cell: " << startCell << "  Goal cell: " << goalCell << std::endl;
    //Cells in req.map have values in [0,100]. 0 are the completely free cells and 100 are the occupied ones.
    /*Cells are uint8 values, but, since map could be really big, wave_front can assign
     *values much greater than 255 to the cells, thus, we need a list of ints instead of uint8  to represent
     *the values assigned to each cell. 
    */
    //If the goal cell is an occupied one, then it will be not possible to find a path
    if(map.data[goalCell] > 50 || map.data[goalCell] < 0)
    {
        std::cout << "PathCalculator.-> Cannot calculate path: goal point is inside occupied space" << std::endl;
        return false;
    }
    
    //First, we make all cells to have 0 for free space and 1 for the occupied one.
    std::vector<int> waveFrontMap;
    for(size_t i=0; i< map.data.size(); i++)
        waveFrontMap.push_back((map.data[i] >= 0 && map.data[i] < 40) ? 0 : 1);

    int freeCounter = 0;
    int occCounter = 0;
    for(size_t i=0; i< map.data.size(); i++)
    {
        if (waveFrontMap[i] == 0) freeCounter++;
        else occCounter++;
    }
    std::cout << "PathCalculator.-> Free cells: " << freeCounter << "  Occupied cells: " << occCounter << std::endl;

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
    
    if(!success)
    {
        std::cout << "PathCalculator.-> Cannot find a path to goal pose :'(" << std::endl;
        return false;
    }
    std::cout << "PathCalculator.-> Finished assignment of cell potentials " << std::endl;
    //After assigning values to each cell, we get the path by gradient descend
    geometry_msgs::PoseStamped p;
    int currentCell = startCell;
    p.pose.position.x = (currentCell % width)*cellSize + originX;
    p.pose.position.y = (currentCell / width)*cellSize + originY;
    p.pose.orientation.w = 1;
    p.header.frame_id = "map";
    resultPath.poses.clear();
    resultPath.poses.push_back(p);
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
        {//Valid cells should have values > 1 since all 0 (free) were assigned to some value >= 2
            if(waveFrontMap[neighbors[i]] > 1 && waveFrontMap[neighbors[i]] < minNeighbor)
            {
                minNeighbor = waveFrontMap[neighbors[i]];
                minNeighborCell = neighbors[i];
            }
        }
        currentCell = minNeighborCell;
        p.pose.position.x = (currentCell % width)*cellSize + originX;
        p.pose.position.y = (currentCell / width)*cellSize + originY;
        resultPath.poses.push_back(p);
    }
    std::cout << "PathCalculator.->Wave-front finished after " << attempts << " attempts" << std::endl;
    std::cout << "PathCalculator.->Calculated path has " << resultPath.poses.size() << " poses" << std::endl;
    return success;
}
