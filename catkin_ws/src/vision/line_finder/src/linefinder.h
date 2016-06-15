///3D FRONT LINE FINDER
#ifndef LINEFINDER_H
#define LINEFINDER_H

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Empty.h"
#include "vision_msgs/Skeletons.h"
#include "justina_tools/JustinaTools.h"

#include "lineransac.h"

#include <math.h>

//Data structures for changing the point of view
struct limits
{
    float minX;
	float minY;
	float minZ;

	float X;
	float Y;
	float Z;

    float maxX;
	float maxY;
	float maxZ;

	float deltaX;
	float deltaY;
	float deltaZ;

	limits();
	~limits();
};

struct infoPixel
{
	int i; // Point's i coordinate in pixels
	int j; // Point's j coordinate in pixels
	
	float x; // Point's x coordinate in metric units
	float y; // Point's y coordinate in metric units
	float z; // Point's z coordinate in metric units

	infoPixel();
	~infoPixel();
};

struct pointOfViewParameters 
{
	cv::Mat src; //Original image
	cv::Mat pov; //Upper View
	cv::Mat front; //Closest point relative to the kinect(TM)'s origin

	bool debug;
	bool fullData;
	bool color;

	double angle;

	limits areaLimits; // Data structure for re-drawing the new point of view 
	infoPixel *pixelArray; // Pointers array to infoPixel elements

	int k;
	int pixelCorner[1000][2];

	pointOfViewParameters();
	~pointOfViewParameters();
};
//******************

//Change coordinate system
//x-axis to the right
//y-axis going up
//z-axis depth
cv::Mat getXYZPoint (cv::Mat xyzCloud, int x, int y);

//Change coordinate system
//x-axis depth
//y-axis to the left
//z-axis going up
cv::Mat getCloudPoint (cv::Mat point);

//Perspective correction of a point with respect to the Kinect(TM)'s camera plane
cv::Mat pointRotation (cv::Mat point, double angle);

// Calculate the normal module of point(x, y) 
// with respect to the canonincal (natural) horizontal vector
double horizontalNormalModule (cv::Mat xyzCloud, int x, int y, double angle);

// Changes XY view to XZ
int changeViewPerspective ( cv::Mat bgrImg, cv::Mat xyzCloud, pointOfViewParameters &povParams);

// Finds the closest point to bottom center (Kinect(TM)'s origin) in the XZ view
cv::Mat frontPoint(pointOfViewParameters &povParams);

// Finds the closest line to bottom center (Kinect(TM)'s origin) in the XZ view
cv::Mat frontLine(pointOfViewParameters &povParams, bool line3d);

#endif
