#include "PathCalculator.h"

bool PathCalculator::WaveFront(nav_msgs::OccupancyGrid& map, geometry_msgs::Pose& startPose, geometry_msgs::Pose& goalPose,
                               nav_msgs::Path& resultPath)
{
    //HAY UN MEGABUG EN ESTE ALGORITMO PORQUE NO ESTOY TOMANDO EN CUENTA QUE EN LOS BORDES DEL
    //MAPA NO SE PUEDE APLICAR CONECTIVIDAD CUATRO NI OCHO. FALTA RESTRINGIR EL RECORRIDO A LOS BORDES MENOS UNO.
    //POR AHORA FUNCIONA XQ CONFÍO EN QUE EL MAPA ES MUCHO MÁS GRANDE QUE EL ÁREA REAL DE NAVEGACIÓN
    std::cout << "PathCalculator.-> Calculating by Wave-front from " << startPose.position.x << "  ";
    std::cout << startPose.position.y << "  to " << goalPose.position.x << "  " << goalPose.position.y << std::endl;
    int startCellX = (int)((startPose.position.x - map.info.origin.position.x)/map.info.resolution);
    int startCellY = (int)((startPose.position.y - map.info.origin.position.y)/map.info.resolution);
    int goalCellX = (int)((goalPose.position.x - map.info.origin.position.x)/map.info.resolution);
    int goalCellY = (int)((goalPose.position.y - map.info.origin.position.y)/map.info.resolution);
    int startCell = startCellY * map.info.width + startCellX;
    int goalCell = goalCellY * map.info.width + goalCellX;

    //First, we grow the map the half the diameter of the robot
    map = PathCalculator::GrowObstacles(map, 0.25);
    
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
    if(map.data[startCell] > 40 || map.data[startCell] < 0)
    {
        std::cout << "PathCalculator.-> Cannot calculate path: start point is inside occupied space" << std::endl;
        return false;
    }

    int* waveFrontMap = new int[map.data.size()];
    if(!PathCalculator::WaveFront(map, startPose, goalPose, waveFrontMap))
    {
        std::cout << "PathCalculator.->Cannot assign potentials to all cells u.u" << std::endl;
        return false;
    }
    
    //After assigning values to each cell, we get the path by gradient descend
    resultPath.header.frame_id = "map";
    resultPath.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped p;
    int currentCell = startCell;
    p.pose.position.x = (currentCell % map.info.width)*map.info.resolution + map.info.origin.position.x;
    p.pose.position.y = (currentCell / map.info.width)*map.info.resolution + map.info.origin.position.y;
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
        neighbors.push_back(currentCell - map.info.width - 1);
        neighbors.push_back(currentCell - map.info.width);
        neighbors.push_back(currentCell - map.info.width + 1);
        neighbors.push_back(currentCell - 1);
        neighbors.push_back(currentCell + 1);
        neighbors.push_back(currentCell + map.info.width - 1);
        neighbors.push_back(currentCell + map.info.width);
        neighbors.push_back(currentCell + map.info.width + 1);
        for(size_t i=0; i < neighbors.size(); i++)
        {//Valid cells should have values > 1 since all 0 (free) were assigned to some value >= 2
            if(waveFrontMap[neighbors[i]] > 1 && waveFrontMap[neighbors[i]] < minNeighbor)
            {
                minNeighbor = waveFrontMap[neighbors[i]];
                minNeighborCell = neighbors[i];
            }
        }
        currentCell = minNeighborCell;
        p.pose.position.x = (currentCell % map.info.width)*map.info.resolution + map.info.origin.position.x;
        p.pose.position.y = (currentCell / map.info.width)*map.info.resolution + map.info.origin.position.y;
        resultPath.poses.push_back(p);
    }

    delete[] waveFrontMap;
    std::cout << "PathCalculator.->Calculated path has " << resultPath.poses.size() << " poses" << std::endl;
    return true;
}

bool PathCalculator::WaveFront(nav_msgs::OccupancyGrid& map, geometry_msgs::Pose& startPose, geometry_msgs::Pose& goalPose,
                               int*& resultWaveFront)
{
    std::cout << "PathCalculator.->Starting potential assignment..." << std::endl;
    if(resultWaveFront == 0)
    {
        std::cout << "PathCalculator.->Null pointer 'resultWaveFront' must be not null." << std::endl;
        return false;
    }
    int startCellX = (int)((startPose.position.x - map.info.origin.position.x)/map.info.resolution);
    int startCellY = (int)((startPose.position.y - map.info.origin.position.y)/map.info.resolution);
    int goalCellX = (int)((goalPose.position.x - map.info.origin.position.x)/map.info.resolution);
    int goalCellY = (int)((goalPose.position.y - map.info.origin.position.y)/map.info.resolution);
    int startCell = startCellY * map.info.width + startCellX;
    int goalCell = goalCellY * map.info.width + goalCellX;

    //First, we make all cells to have 0 for free space and 1 for the occupied one.
    for(size_t i=0; i< map.data.size(); i++)
        resultWaveFront[i] = ((map.data[i] >= 0 && map.data[i] < 40) ? 0 : 1);

    /*
    int freeCounter = 0;
    int occCounter = 0;
    for(size_t i=0; i< map.data.size(); i++)
    {
        if (resultWaveFront[i] == 0) freeCounter++;
        else occCounter++;
    }
    std::cout << "PathCalculator.-> Free cells: " << freeCounter << "  Occupied cells: " << occCounter << std::endl;*/

    //We will use the BREATH-FIRST-SEARCH (BFS) algorithm to find the path.
    std::vector<int> fringe; //List of all nodes to be visited.
    fringe.push_back(goalCell); //We initialize the fringe with the root node. Wave front starts in the goal position
    resultWaveFront[goalCell] = 2; //Wave front starts by assigning 2 to the goal cell
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
            int northCell = currentCell - map.info.width;
            int southCell = currentCell + map.info.width;
            if(resultWaveFront[westCell] == 0) //Check if cell==0 is equivalent to check if cell is NOT in the visited list
            {
                resultWaveFront[westCell] = resultWaveFront[currentCell] + 1; 
                fringe.push_back(westCell); //We put the westCell in the fringe to visit it in the next loop-cycle
            }
            if(resultWaveFront[eastCell] == 0) //Check if cell==0 is equivalent to check if cell is NOT in the visited list
            {
                resultWaveFront[eastCell] = resultWaveFront[currentCell] + 1; 
                fringe.push_back(eastCell); //We put the eastCell in the fringe to visit it in the next loop-cycle
            }
            if(resultWaveFront[northCell] == 0) //Check if cell==0 is equivalent to check if cell is NOT in the visited list
            {
                resultWaveFront[northCell] = resultWaveFront[currentCell] + 1; 
                fringe.push_back(northCell); //We put the northCell in the fringe to visit it in the next loop-cycle
            }
            if(resultWaveFront[southCell] == 0) //Check if cell==0 is equivalent to check if cell is NOT in the visited list
            {
                resultWaveFront[southCell] = resultWaveFront[currentCell] + 1; 
                fringe.push_back(southCell); //We put the southCell in the fringe to visit it in the next loop-cycle
            }
            fringe.erase(fringe.begin());
        }
        attempts++;
    }
    for(int i=0; i < map.data.size(); i++)
        if(resultWaveFront[i] == 0)
            resultWaveFront[i] = INT_MAX;
    
    if(!success)
    {
        std::cout << "PathCalculator.-> Cannot assign potentials from start to goal pose :'(" << std::endl;
        return false;
    }
    std::cout << "PathCalculator.->Wave-front potential assignment finished after " << attempts << " attempts" << std::endl;
    std::cout << "PathCalculator.-> Finished assignment of cell potentials " << std::endl;
    return true;
}

bool PathCalculator::AStar(nav_msgs::OccupancyGrid& map, geometry_msgs::Pose& startPose, geometry_msgs::Pose& goalPose,
                         nav_msgs::Path& resultPath)
{
    //HAY UN MEGABUG EN ESTE ALGORITMO PORQUE NO ESTOY TOMANDO EN CUENTA QUE EN LOS BORDES DEL
    //MAPA NO SE PUEDE APLICAR CONECTIVIDAD CUATRO NI OCHO. FALTA RESTRINGIR EL RECORRIDO A LOS BORDES MENOS UNO.
    //POR AHORA FUNCIONA XQ CONFÍO EN QUE EL MAPA ES MUCHO MÁS GRANDE QUE EL ÁREA REAL DE NAVEGACIÓN
    std::cout << "PathCalculator.-> Calculating by A* from " << startPose.position.x << "  ";
    std::cout << startPose.position.y << "  to " << goalPose.position.x << "  " << goalPose.position.y << std::endl;
    int startCellX = (int)((startPose.position.x - map.info.origin.position.x)/map.info.resolution);
    int startCellY = (int)((startPose.position.y - map.info.origin.position.y)/map.info.resolution);
    int goalCellX = (int)((goalPose.position.x - map.info.origin.position.x)/map.info.resolution);
    int goalCellY = (int)((goalPose.position.y - map.info.origin.position.y)/map.info.resolution);
    int startCell = startCellY * map.info.width + startCellX;
    int goalCell = goalCellY * map.info.width + goalCellX;

    map = PathCalculator::GrowObstacles(map, 0.15);
    
    if(map.data[goalCell] > 40 || map.data[goalCell] < 0)
    {
        std::cout << "PathCalculator.-> Cannot calculate path: goal point is inside occupied space" << std::endl;
        return false;
    }
    if(map.data[startCell] > 40 || map.data[startCell] < 0)
    {
        std::cout << "PathCalculator.-> Cannot calculate path: start point is inside occupied space" << std::endl;
        return false;
    }

    //std::cout << "Creating arrays for dijkstra data" << std::endl;
    bool* isKnown = new bool[map.data.size()];
    int* g_values = new int[map.data.size()];
    int* f_values = new int[map.data.size()];
    int* previous = new int[map.data.size()];
    bool* visited = new bool[map.data.size()];
    int* neighbors = new int[map.data.size()];
    int* waveFrontPotentials = new int[map.data.size()];
    int* nearnessToObstacles = new int[map.data.size()];
    std::vector<int> visitedAndNotKnown;
    
    int currentCell = startCell;

    //std::cout << "Initializing aux arrays for dijkstra" << std::endl;
    //std::cout << "Map data size: " << map.data.size() << std::endl;
    
    if(!PathCalculator::NearnessToObstacles(map, 0.6, nearnessToObstacles))
    {
        std::cout << "PathCalculator.->Cannot calculate nearness to obstacles u.u" << std::endl;
        return false;
    }

    for(int i=0; i< map.data.size(); i++)
    {
        isKnown[i] = map.data[i] > 40 || map.data[i] < 0;
        g_values[i] = INT_MAX;
        f_values[i] = INT_MAX;
        previous[i] = -1;
        visited[i] = map.data[i] > 40 || map.data[i] < 0;
    }
    for(int i=0; i< 8; i++)
        neighbors[i] = 0;

    //std::cout << "First acc dist: " << accDist[0] << std::endl;
    //std::cout << "Setting start node.." << std::endl;
    isKnown[currentCell] = true;
    g_values[currentCell] = 0;
    bool fail = false;
    int attempts = 0;
    //std::cout << "Starting search.." << std::endl;
    //std::cout << "GoalCellX: " << goalCellX << " GoalCellY: " << goalCellY << std::endl;
    while(currentCell != goalCell && !fail && attempts < map.data.size())
    {
        //std::cout << "Current cell: " << currentCell << std::endl;
        //4-connectivity
        neighbors[0] = currentCell - map.info.width;
        neighbors[1] = currentCell - 1;
        neighbors[2] = currentCell + 1;
        neighbors[3] = currentCell + map.info.width;
        //8-connectivity
        neighbors[4] = currentCell - map.info.width - 1;
        neighbors[5] = currentCell - map.info.width + 1;
        neighbors[6] = currentCell + map.info.width - 1;
        neighbors[7] = currentCell + map.info.width + 1;
        //for (int i=0; i< 8; i++)
        for(int i=0; i<8; i++) //Only check neighbors with 4-connectivity
        {
            if(isKnown[neighbors[i]]) continue;
            //g_value is accumulated distance + nearness to obstacles
            int g_value = g_values[currentCell] + 1 + nearnessToObstacles[neighbors[i]]; 
            //h_value is the manhattan distance from the cell to the goal
            //int h_value = abs((neighbors[i]%map.info.width) - goalCellX) + abs((neighbors[i]/map.info.width) - goalCellY);
	    int h_value_x = neighbors[i]%map.info.width - goalCellX;
	    int h_value_y = neighbors[i]/map.info.width - goalCellY;
	    int h_value = (int)(sqrt(h_value_x*h_value_x + h_value_y*h_value_y));
            //std::cout<<"n:"<<neighbors[i]<<" nX: "<<neighborX<<" nY: "<<neighborY<<" g: "<<g_value<<" h: "<<h_value<<" f: "<<(h_value+g_value)<< std::endl;
            if(g_value < g_values[neighbors[i]])
            {
                //std::cout << "Assigning acc dist " << tempDist << " to cell: " << neighbors[i] << std::endl;
                g_values[neighbors[i]] = g_value;
                f_values[neighbors[i]] = g_value + h_value;
                previous[neighbors[i]] = currentCell;
            }
            if(!visited[neighbors[i]])
                visitedAndNotKnown.push_back(neighbors[i]);
            visited[neighbors[i]] = true;
        }
        int min_f_value_idx = -1;
        int min_f_value = std::numeric_limits<int>::max();
        //std::cout << "Acc distances: ";
        for(int i=0; i< visitedAndNotKnown.size(); i++)
        {
            //std::cout << visitedAndNotKnown[i] << ":" <<  accDist[visitedAndNotKnown[i]] << "  ";
            if(f_values[visitedAndNotKnown[i]] < min_f_value)
            {
                min_f_value_idx = i;
                min_f_value = f_values[visitedAndNotKnown[i]];
            }
        }
        //std::cout << std::endl;
        if(min_f_value_idx >= 0)
        {
            currentCell = visitedAndNotKnown[min_f_value_idx];
            isKnown[currentCell] = true;
            //std::cout << "New current cell: " << currentCell << std::endl;
            visitedAndNotKnown.erase(visitedAndNotKnown.begin() + min_f_value_idx);
        }
        else fail = true;
        
        attempts++;
    }
    //std::cout << "PathCalculator.->A* finished after " << attempts << " attempts" << std::endl;
    if(fail)
    {
        std::cout << "PathCalculator.-> Cannot find path to goal point by A* :'(" << std::endl;
        return false;
    }
    //std::cout << "PathCalculator.->Total path cost: " << g_values[goalCell] << std::endl;

    geometry_msgs::PoseStamped p;
    currentCell = goalCell;
    p.pose.position.x = (currentCell % map.info.width)*map.info.resolution + map.info.origin.position.x;
    p.pose.position.y = (currentCell / map.info.width)*map.info.resolution + map.info.origin.position.y;
    p.pose.orientation.w = 1;
    p.header.frame_id = "map";
    resultPath.header.frame_id = "map";
    resultPath.poses.clear();
    resultPath.poses.push_back(p);

    while(previous[currentCell] != -1)
    {
        currentCell = previous[currentCell];
        p.pose.position.x = (currentCell % map.info.width)*map.info.resolution + map.info.origin.position.x;
        p.pose.position.y = (currentCell / map.info.width)*map.info.resolution + map.info.origin.position.y;
        resultPath.poses.insert(resultPath.poses.begin(), p);
    }

    delete[] isKnown;
    delete[] g_values;
    delete[] f_values;
    delete[] previous;
    delete[] neighbors;
    delete[] visited;

    std::cout << "PathCalculator.->Resulting path by A* has " << resultPath.poses.size() << " points." << std::endl;
    return true;
}

nav_msgs::OccupancyGrid PathCalculator::GrowObstacles(nav_msgs::OccupancyGrid& map, float growDist)
{
    //HAY UN MEGABUG EN ESTE ALGORITMO PORQUE NO ESTOY TOMANDO EN CUENTA QUE EN LOS BORDES DEL
    //MAPA NO SE PUEDE APLICAR CONECTIVIDAD CUATRO NI OCHO. FALTA RESTRINGIR EL RECORRIDO A LOS BORDES MENOS UNO.
    //POR AHORA FUNCIONA XQ CONFÍO EN QUE EL MAPA ES MUCHO MÁS GRANDE QUE EL ÁREA REAL DE NAVEGACIÓN
    if(growDist <= 0)
    {
        std::cout << "PathCalculator.->Cannot grow map. Grow dist must be greater than zero." << std::endl;
        return map;
    }
    nav_msgs::OccupancyGrid newMap = map;
    
    int growSteps = (int)(growDist / map.info.resolution);
    int boxSize = (2*growSteps + 1) * (2*growSteps + 1);
    int* neighbors = new int[boxSize];
    int counter = 0;

    //std::cout << "PathCalculator.->Growing map " << growSteps << " steps" << std::endl;
    for(int i=-growSteps; i<=growSteps; i++)
        for(int j=-growSteps; j<=growSteps; j++)
        {
            neighbors[counter] = j*map.info.width + i;
            counter++;
        }
    /*
    std::cout << "Calculation of neighbors finished: " << std::endl;
    for(int i=0; i <boxSize; i++)
        std::cout << neighbors[i] << std::endl;
    */
    int startIdx = growSteps*map.info.width + growSteps;
    int endIdx = map.data.size() - growSteps*map.info.width - growSteps;

    if(endIdx <= 0)
    {
        std::cout << "PathCalculator.->Cannot grow map. Map is smaller than desired growth." << std::endl;
        return map;
    }

    for(int i=startIdx; i < endIdx; i++)
        if(map.data[i] > 40) //Then, is an occupied cell
            for(int j=0; j < boxSize; j++) //If it is occupied, mark as occupied all neighbors in the neighbor-box
                newMap.data[i+neighbors[j]] = 100;

    delete[] neighbors;
    //std::cout << "PathCalculator.->Map-growth finished." << std::endl;
    return newMap;
}

bool PathCalculator::NearnessToObstacles(nav_msgs::OccupancyGrid& map, float distOfInfluence, int*& resultPotentials)
{
    //This function calculates the "nearness to obstacles", e.g., for the following grid:
    /*
      0 0 0 0 0 0 0 0 0 0 0 0 0 0                           2 3 3 3 3 3 2 1 0 1 1 1 1 1
      0 0 x x x 0 0 0 0 0 0 0 0 0                           2 3 x x x 3 2 1 0 1 2 2 2 2
      0 0 x x 0 0 0 0 0 0 0 0 0 0   the resulting nearness  2 3 x x 3 3 2 1 0 1 2 3 3 3
      0 0 x x 0 0 0 0 0 0 0 0 x x   values would be:        2 3 x x 3 2 2 1 0 1 2 3 x x
      0 0 0 0 0 0 0 0 0 0 0 0 x x                           2 3 3 3 3 2 1 1 0 1 2 3 x x
      0 0 0 0 0 0 0 0 0 0 0 0 0 0                           2 2 2 2 2 2 1 0 0 1 2 3 3 3
      Max nearness value will depend on the distance of influence. 
     */
    if(distOfInfluence < 0)
    {
        std::cout << "PathCalculator.->Cannot calc brushfire. DistOfIncluence must be greater than zero." << std::endl;
    }
    if(resultPotentials == 0)
    {
        std::cout << "PathCalculator.->Cannot calc brushfire. 'resultPotentials' param must be not null." << std::endl;
        return false;
    }
    
    int steps = (int)(distOfInfluence / map.info.resolution);
    //std::cout << "PathCalculator.->Calculating nearness with " << steps << " steps. " << std::endl;
    
    int boxSize = (steps*2 + 1) * (steps*2 + 1);
    int* distances = new int[boxSize];
    int* neighbors = new int[boxSize];
    int startIdx = steps*map.info.width + steps;
    int endIdx = map.data.size() - steps*map.info.width - steps;
    if(endIdx <= 0)
    {
        std::cout << "PathCalculator.->Cannot calc brushfire. There is an error in index calculation. Sorry." << std::endl;
        return false;
    }
    int counter = 0;
    for(int i=-steps; i<=steps; i++)
        for(int j=-steps; j<=steps; j++)
        {
            neighbors[counter] = i*map.info.width + j;
            distances[counter] = (steps - std::max(std::abs(i), std::abs(j)) + 1)/2; //Use value/2 just for getting a smaller value
            counter++;
        }

    //std::cout << "Nearness values to be used: " << std::endl;
    //for(int i=0; i < boxSize; i++)
    //    std::cout << distances[i] << " ";
    //std::cout << std::endl;

    for(int i=0; i < map.data.size(); i++)
        resultPotentials[i] = 0;
    
    for(int i=startIdx; i < endIdx; i++)
        if(map.data[i] > 40)
            for(int j = 0; j < boxSize; j++)
                if(resultPotentials[i+neighbors[j]] < distances[j])
                    resultPotentials[i+neighbors[j]] = distances[j];

    delete[] distances;
    delete[] neighbors;
    //std::cout << "PathCalculator.->Finished, calculation of nearness to obstacles :D" << std::endl;
    return true;
}

nav_msgs::Path PathCalculator::SmoothPath(nav_msgs::Path& path, float weight_data, float weight_smooth, float tolerance)
{
    nav_msgs::Path newPath;
    for(int i=0; i< path.poses.size(); i++)
        newPath.poses.push_back(path.poses[i]);
    newPath.header.frame_id = "map";
    
    if(path.poses.size() < 3)
        return newPath;

    //std::cout <<"PathCalculator.->Smoothing path with w_data="<< weight_data<<" w_smooth="<<weight_smooth<<" and tol=" << tolerance << std::endl;
    int attempts = 0;
    tolerance *= path.poses.size();
    float change = tolerance + 1;
    while(change >= tolerance && ++attempts < 1000)
    {
        change = 0;
        for(int i=1; i< path.poses.size() - 1; i++)
        {
            geometry_msgs::Point old_p = path.poses[i].pose.position;
            geometry_msgs::Point new_p = newPath.poses[i].pose.position;
            geometry_msgs::Point new_p_next = newPath.poses[i+1].pose.position;
            geometry_msgs::Point new_p_prev = newPath.poses[i-1].pose.position;
            float last_x = newPath.poses[i].pose.position.x;
            float last_y = newPath.poses[i].pose.position.y;
            new_p.x += weight_data*(old_p.x - new_p.x) + weight_smooth*(new_p_next.x + new_p_prev.x - 2.0*new_p.x);
            new_p.y += weight_data*(old_p.y - new_p.y) + weight_smooth*(new_p_next.y + new_p_prev.y - 2.0*new_p.y);
            change += fabs(new_p.x - last_x) + fabs(new_p.y - last_y);
            newPath.poses[i].pose.position = new_p;
        }
    }
    //std::cout << "PathCalculator.->Smoothing finished after " << attempts << " attempts" <<  std::endl;
    return newPath;
}
