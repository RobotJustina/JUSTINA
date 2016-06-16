#include "LegFinder.h"

LegFinder::LegFinder()
{
    this->umbraldis = 0.35;
    this->motionlessLegInFront = false;     //This flag is set in the funcion findPiernasFrente(float miX, float miY)
    this->legsInFrontCounter = 0;
    this->robotX = 0;
    this->robotY = 0;
    this->robotTheta = 0;
    for (int i = 0; i < 4; ++i)
    {
    	this->legsY.push_back(0.0);
    	this->filteredLegsY.push_back(0.0);
    	this->legsX.push_back(0.0);
    	this->filteredLegsX.push_back(0.0);
    }
}

LegFinder::~LegFinder()
{
}

bool LegFinder::findBestLegs(std::vector<float>& laser_ranges, std::vector<float>& laser_angles, pcl::PointXYZ& ten, float& distan)
{
    std::vector<pcl::PointXYZ> pier;
    distan = 10;
    float aux = 0;
    ten.x = 0;
    ten.y = 0;
    ten.z = 0;
    this->findLegs(laser_ranges, laser_angles, pier);
    for(size_t i=0; i < pier.size(); i++)
    {
        aux = pow(pier[i].x - this->hum.x, 2) + pow(pier[i].y - this->hum.y, 2);
        if(aux < umbraldis && aux < distan)
        {
            distan = aux;
            ten = pier[i];
        }
    }
    if(distan == 10)
        return false;
    else
    {
        //Antes de hacer esta asignación, hay que filtrar "hum".
        
       	float x,y;
    	y= BFB0Y*ten.y + BFB1Y*this->legsY[0] + BFB2Y*this->legsY[1] + BFB3Y*this->legsY[2]// + BFB3Y*this->legsY[3]
    		- BFA1Y*this->filteredLegsY[0] -BFA2Y*this->filteredLegsY[1] -BFA3Y*this->filteredLegsY[2];// -BFA3Y*this->filteredLegsY[3];
		x= BFB0X*ten.x + BFB1X*this->legsX[0] + BFB2X*this->legsX[1] + BFB3X*this->legsX[2]// +BFB3X*this->legsX[3] 
			- BFA1X*this->filteredLegsX[0] -BFA2X*this->filteredLegsX[1] -BFA3X*this->filteredLegsX[2];// -BFA3X*this->filteredLegsX[3];

    	this->legsY.pop_back();
        this->legsY.insert(this->legsY.begin(),ten.y);
        this->legsX.pop_back();
        this->legsX.insert(this->legsX.begin(),ten.x);
        this->filteredLegsY.pop_back();
        this->filteredLegsY.insert(this->filteredLegsY.begin(),y);
        this->filteredLegsX.pop_back();
        this->filteredLegsX.insert(this->filteredLegsX.begin(),x);

        ten.x=x;
        ten.y=y; 
        this->hum=ten;

        return true;
    }
}

bool LegFinder::findLegs(std::vector<float>& laser_ranges, std::vector<float>& laser_angles, 
                         std::vector<pcl::PointXYZ>& legs, int opc, float miX, float miY)
{
    legs.clear();
    if(laser_ranges.size() != laser_angles.size())
    {
        std::cout << "LegFinder.->Something went wrong while getting laser readings :'(" << std::endl;
        return false;
    }
    
    this->laserCallback(laser_ranges, laser_angles);
    
    bool success = false;
    if(opc == 0)
    {
        success = this->findPiernas(legs);
    }
    else if(opc == 1)
    {
        success = this->findPiernasFrente(legs, miX, miY);
    }
    else
    {
        success = this->findPiernasCentrada(legs);
    }
    return success;
}

void LegFinder::laserCallback(std::vector<float>& laser_r, std::vector<float>& laser_t)
{
    if(laser_r.size() != laser_t.size())
    {
        std::cout << "LegFinder.->Something went wrong while getting laser readings :'(" << std::endl;
        return;
    }

    this->rec.clear();
    std::vector<float> laser_x;
    std::vector<float> laser_y;

    this->laserFilter_Mean(laser_r);
    
    for(size_t i=0; i < laser_r.size(); i++)
    {
        laser_x.push_back(laser_r[i] * cos(laser_t[i]));
        laser_y.push_back(laser_r[i] * sin(laser_t[i]));
    }

    std::vector<float> laser_flank;
    std::vector<float> flank_id0;
    std::vector<float> flank_id1;
    std::vector<bool> flank_id2;
    int ant2 = 0;
    
    for(int i= 1; i < laser_r.size(); i++)
    {
        pcl::PointXYZ cua;
        float sumax, sumay, px, py, m1, m2, ang;
        int ant = ant2;
        if(fabs(laser_r[i] - laser_r[i-1]) > FLANK_THRESHOLD)
        {
            if((pow(laser_x[ant] - laser_x[i-1], 2) + pow(laser_y[ant] - laser_y[i-1], 2)) > PIERNA_DELGADA &&
               (pow(laser_x[ant] - laser_x[i-1], 2) + pow(laser_y[ant] - laser_y[i-1], 2)) < PIERNA_GRUESA)
            {
                if(esPierna(laser_x[ant], laser_y[ant], laser_x[i-1], laser_y[i-1]) ||
                   esPierna(laser_x[ant+1], laser_y[ant+1], laser_x[i-2], laser_y[i-2]))
                {
                    sumax = 0;
                    sumay = 0;
                    for(int j= ant; j < i; j++)
                    {
                        sumax += laser_x[j];
                        sumay += laser_y[j];
                    }
                    flank_id0.push_back(sumax / (float)(i - ant));
                    flank_id1.push_back(sumay / (float)(i - ant));
                    flank_id2.push_back(false);
                }
            }
            else 
            {
                if((pow(laser_x[ant] - laser_x[i-1], 2) + pow(laser_y[ant] - laser_y[i-1], 2)) > DOS_PIERNAS_DELGADAS &&
                    (pow(laser_x[ant] - laser_x[i-1], 2) + pow(laser_y[ant] - laser_y[i-1], 2)) < DOS_PIERNAS_GRUESAS)
                {
                    if(esPierna(laser_x[ant], laser_y[ant], laser_x[i-1], laser_y[i-1]) ||
                       esPierna(laser_x[ant+1], laser_y[ant+1], laser_x[i-2], laser_y[i-2]))
                    {
                        sumax = 0;
                        sumay = 0;
                        for(int j= ant; j < i; j++)
                        {
                            sumax += laser_x[j];
                            sumay += laser_y[j];
                        }
                        cua.x = sumax / (float)(i - ant);
                        cua.y = sumay / (float)(i - ant);
                        cua.z = 2;
                        this->rec.push_back(cua);
                    }
                }
            }
            ant2 = i;
        }
        else
        {
            laser_flank.push_back(0);
        }
    }
    
    for(int i=0; i < (int)(flank_id1.size())-2; i++)
    {
        for(int j=1; j < 3; j++)
        {
            pcl::PointXYZ cua;
            float px, py;

            if((pow(flank_id0[i] - flank_id0[i+j], 2) + pow(flank_id1[i] - flank_id1[i+j], 2)) > DOS_PIERNAS_CERCAS &&
               (pow(flank_id0[i] - flank_id0[i+j], 2) + pow(flank_id1[i] - flank_id1[i+j], 2)) < DOS_PIERNAS_LEJOS)
            {
                px = (flank_id0[i] + flank_id0[i + j])/2;
                py = (flank_id1[i] + flank_id1[i + j])/2;
                if((px*px + py*py) < HORIZON_THRESHOLD)
                {
                    cua.x = px;
                    cua.y = py;
                    cua.z = 2;
                    this->rec.push_back(cua);
                    flank_id2[i] = true;
                    flank_id2[i+j] = true;
                }
            }
        }
    }
    
    if(flank_id1.size() > 1)
    {
        pcl::PointXYZ cua;
        float px, py;

        if((pow(flank_id0[flank_id1.size()-2] - flank_id0[flank_id1.size()-1], 2) +
            pow(flank_id1[flank_id1.size()-2] - flank_id1[flank_id1.size()-1], 2)) > DOS_PIERNAS_CERCAS &&
           (pow(flank_id0[flank_id1.size()-2] - flank_id0[flank_id1.size()-1], 2) +
            pow(flank_id1[flank_id1.size()-2] - flank_id1[flank_id1.size()-1], 2)) < DOS_PIERNAS_LEJOS)
        {
            px = (flank_id0[flank_id1.size()-2] + flank_id0[flank_id1.size()-1])/2.0;
            py = (flank_id1[flank_id1.size()-2] + flank_id1[flank_id1.size()-1])/2.0;
            if((px*px + py*py) < HORIZON_THRESHOLD)
            {
                cua.x = px;
                cua.y = py;
                cua.z = 2;
                this->rec.push_back(cua);
                flank_id2[flank_id1.size() - 2] = true;
                flank_id2[flank_id1.size() - 1] = true;
            }
        }
    }
    for(int i=0; i < flank_id1.size(); i++)
    {
        if(!flank_id2[i])
        {
            pcl::PointXYZ cua;
            cua.x = flank_id0[i];
            cua.y = flank_id1[i];
            cua.z = 1;
            this->rec.push_back(cua);
        }
    }
}

void LegFinder::laserFilter_Mean(std::vector<float>& vector_r)
{
    std::vector<float> vec;
    vec.push_back(vector_r[0]);
    int i=1, cl = vector_r.size() - 1;
    bool de = false;
    float mean, a;
    
    while(i < cl)
    {
        if(fabs(vector_r[i-1] - vector_r[i]) < FILTER_THRESHOLD)
        {
            de = true;
            do
            {
                if(fabs(vector_r[i+1] - vector_r[i]) < FILTER_THRESHOLD)
                {
                    mean = 0;
                    for(int k= -1; k < 2; k++)
                        mean+= vector_r[i+k];
                    a = (mean / 3.0);
                    vec.push_back(a);
                    i++;
                }
                else
                {
                    vec.push_back(vector_r[i]);
                    i++;
                    if( i < vector_r.size() - 1)
                    {
                        vec.push_back(vector_r[i]);
                        i++;
                    }
                    de = false;
                }
            }while( de  && i < cl);
        }
        else
        {
            vec.push_back(vector_r[i]);
            i++;
        }
    }
    vec.push_back(vector_r[i]);
    vector_r.clear();
    for(size_t i=0; i < vec.size(); i++)
        vector_r.push_back(vec[i]);
}

bool LegFinder::findPiernas(std::vector<pcl::PointXYZ>& piernas)
{
    piernas.clear();    
    float cosTheta = cos(robotTheta);
    float sinTheta = sin(robotTheta);

    for(size_t i=0; i< this->rec.size(); i++)
    {
        pcl::PointXYZ recAbs;
        recAbs.x = robotX + this->rec[i].x*cosTheta - this->rec[i].y*sinTheta;
        recAbs.y = robotY + this->rec[i].x*sinTheta + this->rec[i].y*cosTheta;
        piernas.push_back(recAbs);
    }
    return true;
}

bool LegFinder::findPiernasFrente(float miX, float miY)
{
    float cosTheta = cos(robotTheta);
    float sinTheta = sin(robotTheta);
    int p = -1;
    for(size_t i=0; i< this->rec.size(); i++)
    {
        if((fabs(this->rec[i].y) < LEG_IN_FRONT_Y_RANGE && this->rec[i].x < LEG_IN_FRONT_X_RANGE  && this->rec[i].x > 0.15) ||
           (fabs(this->rec[i].y) < miY && this->rec[i].x < miX))
        {
            p = i;
        }
    }
    if( p != -1)
    {
        pcl::PointXYZ recAbs;
        recAbs.x = robotX + this->rec[p].x*cosTheta - this->rec[p].y*sinTheta;
        recAbs.y = robotY + this->rec[p].x*sinTheta + this->rec[p].y*cosTheta;

        this->hum = recAbs;
        return true;
    }
    else
        return false;
}

bool LegFinder::findPiernasFrente(std::vector<pcl::PointXYZ>& legs, float miX, float miY)
{
    int p = -1;
    float cosTheta = cos(robotTheta);
    float sinTheta = sin(robotTheta);
    legs.clear();
    for(size_t i=0; i< this->rec.size(); i++)
    {
        if((fabs(this->rec[i].y) < LEG_IN_FRONT_Y_RANGE && this->rec[i].x < LEG_IN_FRONT_X_RANGE && this->rec[i].x > 0.15) ||
           (fabs(this->rec[i].y) < miY && this->rec[i].x < miX))
        {
            p = i;
        }
    }
    if( p != -1)
    {
        pcl::PointXYZ recAbs;
        recAbs.x = robotX + this->rec[p].x*cosTheta - this->rec[p].y*sinTheta;
        recAbs.y = robotY + this->rec[p].x*sinTheta + this->rec[p].y*cosTheta;

        legs.push_back(recAbs);
        return true;
    }
    else
        return false;
}

bool LegFinder::findPiernasFrente(pcl::PointXYZ& legPose, float miX, float miY)
{
    std::vector<pcl::PointXYZ> legs;
    bool success = findPiernasFrente(legs, miX, miY);
    if(success)
    {
        legPose = legs[0];
        this->lastHum = this->hum;
        this->hum = legPose;
        float diffX = lastHum.x - hum.x;
        float diffY = lastHum.y - hum.y;
        if((diffX*diffX + diffY*diffY) < 0.1)
            this->legsInFrontCounter++;
        if(this->legsInFrontCounter > 20)
            this->legsInFrontCounter = 20;
    }
    else
        this->legsInFrontCounter = 0;

    if(this->legsInFrontCounter == 20)
        this->motionlessLegInFront = true;
    else
        this->motionlessLegInFront = false;
    
    return success;
}

bool LegFinder::findPiernasCentrada(std::vector<pcl::PointXYZ>& piernas)
{
    piernas.clear();
    float cent = MAX_FLOAT;
    float m = 0;
    pcl::PointXYZ minimo;
    for(size_t i=0; i < this->rec.size(); i++)
    {
        if(this->rec[i].x > 0)
        {
            m = fabs(this->rec[i].y) / this->rec[i].x;
            if(m < cent)
            {
                minimo = this->rec[i];
                cent = m;
            }
        }
    }
    piernas.push_back(minimo);
    return true;
}

bool LegFinder::esPierna(float x1, float y1, float x2, float y2)
{
    bool res = false;
    float m1, m2, px, py, ang;
    if(x1 != x2)
        m1 = (y1 - y2) / (x1 - x2);
    else
        m1 = MAX_FLOAT;

    px = (x1 + x2) / 2;
    py = (y1 + y2) / 2;
    if((px*px + py*py) < HORIZON_THRESHOLD)
    {
        if(px != 0)
            m2 = py / px;
        else
            m2 = MAX_FLOAT;
        ang = fabs((m2 - m1) / (1 + (m2*m1)));
        if(ang > 1.999)
            res = true;
    }
    return res;
}

void LegFinder::setRobotPose(float robotX, float robotY, float robotTheta)
{
    this->robotX = robotX;
    this->robotY = robotY;
    this->robotTheta = robotTheta;
}

bool LegFinder::isThereMotionlessLegInFront()
{
    //This flag is set in the funcion findPiernasFrente(float miX, float miY)
    return this->motionlessLegInFront;
}
