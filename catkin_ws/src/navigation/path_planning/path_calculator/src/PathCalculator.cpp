#include "PathCalculator.h"

bool PathCalculator::WaveFront(nav_msgs::OccupancyGrid& map, geometry_msgs::Pose& startPose,
                                         geometry_msgs::Pose& goalPose, nav_msgs::Path& resultPath)
{
    //HAY UN MEGABUG EN ESTE ALGORITMO PORQUE NO ESTOY TOMANDO EN CUENTA QUE EN LOS BORDES DEL
    //MAPA NO SE PUEDE APLICAR CONECTIVIDAD CUATRO NI OCHO. FALTA RESTRINGIR EL RECORRIDO A LOS BORDES MENOS UNO.
    //POR AHORA FUNCIONA XQ CONFÍO EN QUE EL MAPA ES MUCHO MÁS GRANDE QUE EL ÁREA REAL DE NAVEGACIÓN
    int width = map.info.width;
    int height = map.info.height;
    float cellSize = map.info.resolution;
    float originX = map.info.origin.position.x;
    float originY = map.info.origin.position.y;
    float startX = startPose.position.x;
    float startY = startPose.position.y;
    float goalX = goalPose.position.x;
    float goalY = goalPose.position.y;
    int startCellX = (int)((startX - originX)/cellSize);
    int startCellY = (int)((startX - originX)/cellSize);
    int goalCellX = (int)((goalX - originX)/cellSize);
    int goalCellY = (int)((goalY - originY)/cellSize);
    int startCell = startCellY * width + startCellX;
    int goalCell = goalCellY * width + goalCellX;

    //First, we grow the map the half the diameter of the robot
    map = PathCalculator::GrowObstacles(map, 0.25);

    resultPath.header.frame_id = "map";
    resultPath.header.stamp = ros::Time::now();
    std::cout << "PathCalculator.->Calculating by wavefront from " << startX << "  " << startY << " to " << goalX << "  " <<  goalY << std::endl;
    //std::cout << "PathCalculator.->Start cell: " << startCell << "  Goal cell: " << goalCell << std::endl;
    //Cells in req.map have values in [0,100]. 0 are the completely free cells and 100 are the occupied ones.
    /*Cells are uint8 values, but, since map could be really big, wave_front can assign
     *values much greater than 255 to the cells, thus, we need a list of ints instead of uint8  to represent
     *the values assigned to each cell. 
    */
    //If the goal cell is an occupied one, then it will be not possible to find a path
    if(map.data[goalCell] > 40 || map.data[goalCell] < 0)
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

bool PathCalculator::Dijkstra(nav_msgs::OccupancyGrid& map, geometry_msgs::Pose& startPose, geometry_msgs::Pose& goalPose,
                         nav_msgs::Path& resultPath)
{
    //HAY UN MEGABUG EN ESTE ALGORITMO PORQUE NO ESTOY TOMANDO EN CUENTA QUE EN LOS BORDES DEL
    //MAPA NO SE PUEDE APLICAR CONECTIVIDAD CUATRO NI OCHO. FALTA RESTRINGIR EL RECORRIDO A LOS BORDES MENOS UNO.
    //POR AHORA FUNCIONA XQ CONFÍO EN QUE EL MAPA ES MUCHO MÁS GRANDE QUE EL ÁREA REAL DE NAVEGACIÓN
    std::cout << "PathCalculator.-> Calculating by dijkstra from " << startPose.position.x << "  ";
    std::cout << startPose.position.y << "  to " << goalPose.position.x << "  " << goalPose.position.y << std::endl;
    int startCellX = (int)((startPose.position.x - map.info.origin.position.x)/map.info.resolution);
    int startCellY = (int)((startPose.position.y - map.info.origin.position.y)/map.info.resolution);
    int goalCellX = (int)((goalPose.position.x - map.info.origin.position.x)/map.info.resolution);
    int goalCellY = (int)((goalPose.position.y - map.info.origin.position.y)/map.info.resolution);
    int startCell = startCellY * map.info.width + startCellX;
    int goalCell = goalCellY * map.info.width + goalCellX;
    
    if(map.data[goalCell] > 40 || map.data[goalCell] < 0)
    {
        std::cout << "PathCalculator.-> Cannot calculate path: goal point is inside occupied space" << std::endl;
        return false;
    }

    std::cout << "Creating arrays for dijkstra data" << std::endl;
    std::vector<bool> isKnown;
    std::vector<int> accDist;
    std::vector<int> previous;
    int currentCell = startCell;
    std::cout << "Initializing aux arrays for dijkstra" << std::endl;
    //std::cout << "Map data size: " << map.data.size() << std::endl;
    for(size_t i=0; i< map.data.size(); i++)
    {
        isKnown.push_back(false);
        accDist.push_back(INT_MAX);
        previous.push_back(-1);
    }
    std::cout << "First acc dist: " << accDist[0] << std::endl;
    std::cout << "Setting start node.." << std::endl;
    isKnown[currentCell] = true;
    accDist[currentCell] = 0;
    
    std::vector<int> neighbors;
    for(int i=0; i< 8; i++) neighbors.push_back(0);
    std::vector<int> visitedAndNotKnown;
    bool fail = false;
    int attempts = 0;
    std::cout << "Starting search.." << std::endl;
    while(currentCell != goalCell && !fail && attempts < 10)
    {
        std::cout << "Current cell: " << currentCell << std::endl;
        neighbors[0] = currentCell - map.info.width - 1;
        neighbors[1] = currentCell - map.info.width;
        neighbors[2] = currentCell - map.info.width + 1;
        neighbors[3] = currentCell - 1;
        neighbors[4] = currentCell + 1;
        neighbors[5] = currentCell + map.info.width - 1;
        neighbors[6] = currentCell + map.info.width;
        neighbors[7] = currentCell + map.info.width + 1;
        for (int i=0; i< 8; i++)
        {
            if(isKnown[neighbors[i]]) continue;
            if(map.data[neighbors[i]] > 40 || map.data[neighbors[i]] < 0) continue;
            int tempDist = accDist[currentCell] + 1;
            //std::cout << "Neighbor: " << neighbors[i] << " with acc dist: " << accDist[neighbors[i]] << std::endl;
            if(tempDist < accDist[neighbors[i]])
            {
                //std::cout << "Assigning acc dist " << tempDist << " to cell: " << neighbors[i] << std::endl;
                accDist[neighbors[i]] = tempDist;
                previous[neighbors[i]] = currentCell;
            }
            visitedAndNotKnown.push_back(neighbors[i]);
        }
        int minDistIdx = -1;
        int minAccDist = std::numeric_limits<int>::max();
        std::cout << "Acc distances: ";
        for(int i=0; i< visitedAndNotKnown.size(); i++)
        {
            std::cout << accDist[visitedAndNotKnown[i]] << " ";
            if(accDist[visitedAndNotKnown[i]] < minAccDist)
            {
                minDistIdx = i;
                minAccDist = accDist[visitedAndNotKnown[i]];
            }
        }
        std::cout << std::endl;
        if(minDistIdx >= 0)
        {
            currentCell = visitedAndNotKnown[minDistIdx];
            isKnown[currentCell] = true;
            visitedAndNotKnown.erase(visitedAndNotKnown.begin() + minDistIdx);
        }
        else fail = true;
        attempts++;
    }
    std::cout << "PathCalculator.->Dijkstra finished after " << attempts << " attempts" << std::endl;
    if(!fail)
        std::cout << "PathCalculator.->Goal acc distance: " << accDist[goalCell] << std::endl;
    else std::cout << "PathCalculator.-> Cannot find path to goal point :'(" << std::endl;
}

nav_msgs::OccupancyGrid PathCalculator::GrowObstacles(nav_msgs::OccupancyGrid& map, float growDist)
{
    //HAY UN MEGABUG EN ESTE ALGORITMO PORQUE NO ESTOY TOMANDO EN CUENTA QUE EN LOS BORDES DEL
    //MAPA NO SE PUEDE APLICAR CONECTIVIDAD CUATRO NI OCHO. FALTA RESTRINGIR EL RECORRIDO A LOS BORDES MENOS UNO.
    //POR AHORA FUNCIONA XQ CONFÍO EN QUE EL MAPA ES MUCHO MÁS GRANDE QUE EL ÁREA REAL DE NAVEGACIÓN
    nav_msgs::OccupancyGrid newMap = map;
    if(growDist <= 0)
    {
        std::cout << "PathCalculator.->Cannot grow map. Grow dist must be greater than zero." << std::endl;
        return map;
    }
    int growSteps = (int)(growDist / map.info.resolution);
    std::cout << "Growing " << growSteps << "steps" << std::endl;
    int width = map.info.width;
    int j0 = width + 1;
    int j1 = map.data.size() - width - 1;
    for(int i=0; i < growSteps; i++)
    {
        for(int j = j0; j< j1; j++)
        {//Cells with values > 40 are considered as occupied
            if(map.data[j] > 40)
            {
                newMap.data[j - width - 1] = 100;
                newMap.data[j - width] = 100;
                newMap.data[j - width + 1] = 100;
                newMap.data[j - 1] = 100;
                newMap.data[j + 1] = 100;
                newMap.data[j + width - 1] = 100;
                newMap.data[j + width] = 100;
                newMap.data[j + width + 1] = 100;
            }
        }
        map = newMap;
    }

    return newMap;
}
