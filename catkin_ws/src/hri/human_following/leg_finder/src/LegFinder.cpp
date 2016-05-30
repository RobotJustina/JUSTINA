#include "LegFinder.h"


LegFinder::LegFinder()
{
	LegFinder::hum =  pcl::PointXYZ();
	LegFinder::rec = std::vector<pcl::PointXYZ>();
}
bool LegFinder::findBestLegs(pcl::PointCloud<pcl::PointXYZ>::Ptr laserCyl, pcl::PointCloud<pcl::PointXYZ>::Ptr laserCart, pcl::PointXYZ robotPos, pcl::PointXYZ& humPos)
{	
	std::vector<pcl::PointXYZ> legs;
	double distance=10;
	double aux=0;
	hum.x=0 , hum.y=0 ,hum.z=0;
	LegFinder::findLegs(laserCart,laserCyl,legs,robotPos);
	for (int i = 0; i < (int)legs.size(); ++i)
	{
		
		aux= ((legs[i].x-hum.x)*(legs[i].x-hum.x))+((legs[i].y-hum.y)*(legs[i].y-hum.y));
		if (aux < DISTANCE_THRESHOLD && aux < distance)
		{
			distance = aux;
			humPos = legs[i];
		}
	}
	if (distance == 10)
		return false;
	LegFinder::hum = humPos;
	return true;
}
bool LegFinder::findLegs(pcl::PointCloud<pcl::PointXYZ>::Ptr laserCart, pcl::PointCloud<pcl::PointXYZ>::Ptr laserCyl, std::vector<pcl::PointXYZ>& legs,pcl::PointXYZ robotPos)/***/
{
	int opc = 0;
	double miX=0,miY=0;
	
   
	if (laserCart->empty() || laserCyl->empty() || laserCyl->size() != laserCart->size())
	{
		std::cout<<"LegsFinder.-> Something went wrong while getting laser readings"<<std::endl;
		return false;
	}
	bool success = false;
	
	LegFinder::laserCallback(laserCart,laserCyl);

	if (opc == 0)
	{
		success=LegFinder::findLegs(legs,robotPos);

	}else if (opc == 1)
	{
		success= LegFinder::findFrontLegs(legs, miX, miY, robotPos);
	}else
	{
		success= LegFinder::findCenterLegs(legs);
	}
	return true;
}
void LegFinder::laserCallback(pcl::PointCloud<pcl::PointXYZ>::Ptr laserCart, pcl::PointCloud<pcl::PointXYZ>::Ptr laserCyl)
{
	LegFinder::rec.clear();
	LegFinder::laserFilterMean(laserCyl);

	
	std::vector<double> laser_flank;
	std::vector<double> flank_id0;
	std::vector<double> flank_id1;
	std::vector<bool> flank_id2;
	int ant = 0;
	for (int i = 1; i < (int)laserCyl->size(); ++i)
	{
		pcl::PointXYZ cua;
		double sumax, sumay;
		//int ant = ant2; por si no jala descomentar culpa del señor jebus 
		if (std::fabs(laserCyl->points[i].x-laserCyl->points[i-1].x) <= FLANK_THRESHOLD)
		{
			laser_flank.push_back(0.0);
			continue; 
		}

		double laserXPow2 =(laserCart->points[ant].x - laserCart->points[i-1].x)*(laserCart->points[ant].x - laserCart->points[i-1].x);
		double laserYPow2 =(laserCart->points[ant].y - laserCart->points[i-1].y)*(laserCart->points[ant].y - laserCart->points[i-1].y);
		double dist = laserXPow2+laserYPow2;

		bool isLeg1 = LegFinder::isLeg(laserCart->points[ant].x, laserCart->points[ant].y, laserCart->points[i - 1].x, laserCart->points[i - 1].y); 
		bool isLeg2 = LegFinder::isLeg(laserCart->points[ant + 1].x, laserCart->points[ant + 1].y, laserCart->points[i - 2].x, laserCart->points[i - 2].y); 
		if (isLeg1  || isLeg2)
		{
			if (dist > PIERNA_DELGADA && dist < PIERNA_GRUESA)
			{
				sumax = 0;
				sumay = 0;
				for (int j = ant; j < i; ++j)
				{
					sumax+= laserCart->points[j].x;
					sumay+= laserCart->points[j].y;
				}
				flank_id0.push_back(sumax / (double)(i-ant));
				flank_id1.push_back(sumay / (double)(i-ant));
				flank_id2.push_back(false);
				
			}
			else if(dist > DOS_PIERNAS_DELGADAS && dist < DOS_PIERNAS_GRUESAS)
			{
				sumax = 0;
				sumay = 0;
				for (int j = ant; j < i; ++j)
				{
					sumax+= laserCart->points[j].x;
					sumay+= laserCart->points[j].y;
				}
				cua.x = sumax / (double)(i-ant);
				cua.y = sumay / (double)(i-ant);
				cua.z = 2.0;
				rec.push_back(cua);
			}
		}

		ant = i;
	}

	for (int i = 0; i < (int)flank_id1.size()-2; ++i)
	{
		
		for (int j = 1; j < 3; ++j)
		{
			double flank_id0Pow2 = (flank_id0[i] - flank_id0[i + j]) * (flank_id0[i] - flank_id0[i + j]);
			double flank_id1Pow2 = (flank_id1[i] - flank_id1[i + j]) * (flank_id1[i] - flank_id1[i + j]);
			double dist = flank_id0Pow2 + flank_id1Pow2;

			if (dist > DOS_PIERNAS_CERCAS && dist < DOS_PIERNAS_LEJOS)
			{
				double px = (flank_id0[i] + flank_id0[i + j]) / 2.0;
				double py = (flank_id1[i] + flank_id1[i + j]) / 2.0;

				if( (px*px) + (py*py) < HORIZON_THRESHOLD ) 
				{
					pcl::PointXYZ cua;
					cua.x = (flank_id0[i] + flank_id0[i+j]) / 2.0;
					cua.y = (flank_id1[i] + flank_id1[i+j]) / 2.0;
					cua.z = 2.0; 
					rec.push_back(cua); 
					flank_id2[i] = true;
					flank_id2[i+j] = true; 
				}

			}

		}
	}
	if ((int)flank_id1.size() > 1)
	{
		int i=(int)flank_id1.size();
		double flank_id0Pow2 = (flank_id0[i-2] - flank_id0[i-1]) * (flank_id0[i-2] - flank_id0[i-1]);
		double flank_id1Pow2 = (flank_id1[i-2] - flank_id1[i-1]) * (flank_id1[i-2] - flank_id1[i-1]);
		double dist = flank_id0Pow2 + flank_id1Pow2;

		if (dist > DOS_PIERNAS_CERCAS && dist < DOS_PIERNAS_LEJOS)
		{
			double px = (flank_id0[i-2] + flank_id0[i-1]) / 2.0;
			double py = (flank_id1[i-2] + flank_id1[i-1]) / 2.0;
			if( (px*px) + (py*py) < HORIZON_THRESHOLD ) 
			{
				pcl::PointXYZ cua;
				cua.x = (flank_id0[i-2] + flank_id0[i-1]) / 2.0;
				cua.y = (flank_id1[i-2] + flank_id1[i-1]) / 2.0;
				cua.z = 2.0; 
				rec.push_back(cua); 
				flank_id2[i-2] = true;
				flank_id2[i-1] = true; 
			}
		}
	}
	for (int i = 0; i < (int)flank_id1.size(); ++i)
	{
		if (flank_id2[i])
			continue;
		pcl::PointXYZ cua;
		cua.x = flank_id0[i];
		cua.y = flank_id1[i];
		cua.z = 1;
		rec.push_back(cua); 
	}	

}
void LegFinder::laserFilterMean(pcl::PointCloud<pcl::PointXYZ>::Ptr& laserCyl)
{
	int i = 1, cl = (int)laserCyl->size()-1;
	bool de = false;
	while(i<cl)
	{
		if (std::fabs(laserCyl->points[i-1].x - laserCyl->points[i].x ) < FILTER_THRESHOLD)
		{
			de = true;
			do
			{
				if (std::fabs(laserCyl->points[i+1].x - laserCyl->points[i].x ) < FILTER_THRESHOLD)
				{
					laserCyl->points[i].x = (laserCyl->points[i-1].x + laserCyl->points[i].x + laserCyl->points[i+1].x)/3;////mean
					++i;
				}else
				{
					++i;
					i=(i<cl)?i+1:i;
					de = false;
				}
			} while (de && i<cl);
		}else
		{
			++i;
		}
	}
}

bool LegFinder::findLegs(std::vector<pcl::PointXYZ>& legs, pcl::PointXYZ robotPos)/***/
{	
	//double cosTheta = std::cos(robotPos.z);//cos(RobotAngle)
	//double sinTheta = std::sin(robotPos.z);//sin(RobotAngle)
	for (int i = 0; i < (int)LegFinder::rec.size(); ++i)
	{
		pcl::PointXYZ recAbs = pcl::PointXYZ();
		//recAbs.x = LegFinder::rec[i].x * cosTheta - LegFinder::rec[i].y * sinTheta;
		//recAbs.y = LegFinder::rec[i].x * sinTheta + LegFinder::rec[i].y * cosTheta;
		recAbs.x = LegFinder::rec[i].x;
		recAbs.y = LegFinder::rec[i].y;
		recAbs.z = 0.0;
		legs.push_back(recAbs);
	}
	return true;
}		
bool LegFinder::findFrontLegs(double miX, double miY, pcl::PointXYZ robotPos)/****/
{
	int p=-1;
	double cosTheta = std::cos(robotPos.z);//cos(RobotAngle)
	double sinTheta = std::sin(robotPos.z);//sin(RobotAngle)
	double fabsrecY; 
	for (int i = 0; i < (int)LegFinder::rec.size(); ++i)
	{
		fabsrecY = std::fabs(LegFinder::rec[i].y);
		p=((fabsrecY < 0.3 && LegFinder::rec[i].x < 1.5) || (fabsrecY < miY && LegFinder::rec[i].x < miX))?i:p;
	}
	if (p<0)
	{
		return false;
	}else
	{
		pcl::PointXYZ recAbs = pcl::PointXYZ();
		recAbs.x = robotPos.x + LegFinder::rec[p].x * cosTheta - LegFinder::rec[p].y * sinTheta;
		recAbs.y = robotPos.y + LegFinder::rec[p].x * sinTheta + LegFinder::rec[p].y * cosTheta;
		hum = recAbs;
		return true;		
	}
}
bool LegFinder::findFrontLegs(std::vector<pcl::PointXYZ>& legs, double miX, double miY, pcl::PointXYZ robotPos)/****/
{	
	int p=-1;
	double cosTheta = std::cos(robotPos.z);//cos(RobotAngle)
	double sinTheta = std::sin(robotPos.z);//sin(RobotAngle)
	double fabsrecY; 
	for (int i = 0; i < (int)LegFinder::rec.size(); ++i)
	{
		fabsrecY = std::fabs(LegFinder::rec[i].y);
		p=((fabsrecY < 0.3 && LegFinder::rec[i].x < 1.5) || (fabsrecY < miY && LegFinder::rec[i].x < miX))?i:p;
	}
	if (p<0)
	{
		return false;
	}else
	{
		pcl::PointXYZ recAbs = pcl::PointXYZ();
		recAbs.x = robotPos.x + LegFinder::rec[p].x * cosTheta - LegFinder::rec[p].y * sinTheta;
		recAbs.y = robotPos.y + LegFinder::rec[p].x * sinTheta + LegFinder::rec[p].y * cosTheta;
		legs.push_back(recAbs);
		return true;		
	}
}
bool LegFinder::findCenterLegs(std::vector<pcl::PointXYZ>& legs)
{	
	double center=DBL_MAX;
	double m=0.0;
	pcl::PointXYZ minimo = pcl::PointXYZ();
	for (int i=0 ; i<(int)LegFinder::rec.size() ; ++i)
	{
		m= std::fabs(LegFinder::rec[i].y)/LegFinder::rec[i].x;
		if (m<center)
		{
			minimo=LegFinder::rec[i];
			center=m;
		}
	}
	legs.push_back(minimo);
	return true;
}
bool LegFinder::isLeg(double x1, double y1, double x2, double y2)
{
	bool res = false;
	double m1,m2,px,py,ang;
	m1 = (x1 != x2)? (y1 - y2) / (x1 - x2): DBL_MAX;
	px = (x1 + x2) / 2.0;
	py = (y1 + y2) / 2.0;
	if (((px*px) + (py*py)) < HORIZON_THRESHOLD)
	{
		m2=(px != 0.0)? (py/px):DBL_MAX;
		ang = std::fabs((m2 - m1) / (1.0 + (m2 * m1)));
		res=(ang > 1.999)?true:res;
	}
	return res;
}