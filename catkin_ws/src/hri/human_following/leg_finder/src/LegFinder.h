#include <iostream>
#include <cmath>
#include <pcl/io/openni_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#define HORIZON_THRESHOLD    25
#define FILTER_THRESHOLD     0.081f
#define FLANK_THRESHOLD      0.04f
#define HORIZON_THRESHOLD    25
#define DISTANCE_THRESHOLD   1.0f
//#define MAX                  57295779500
#define PIERNA_DELGADA       0.006241f//7.9CM,0.006241,6241
#define PIERNA_GRUESA        0.037f//19.23CM,0.037,37000
#define DOS_PIERNAS_DELGADAS 0.056644f//23.8CM,0.056644,56644
#define DOS_PIERNAS_GRUESAS  0.25f//50CM,0.25,250000
#define DOS_PIERNAS_CERCAS   0.022201f//14.9CM,0.022201,22201
#define DOS_PIERNAS_LEJOS    0.16f//40CM,0.16,160000

class LegFinder
{

private:
	pcl::PointXYZ hum;
	std::vector<pcl::PointXYZ> rec;

public:
	LegFinder();
	bool findBestLegs(pcl::PointCloud<pcl::PointXYZ>::Ptr laserCyl, pcl::PointCloud<pcl::PointXYZ>::Ptr laserCart, pcl::PointXYZ robotPos, pcl::PointXYZ& humPos);
	bool findLegs(pcl::PointCloud<pcl::PointXYZ>::Ptr laserCart, pcl::PointCloud<pcl::PointXYZ>::Ptr laserCyl, std::vector<pcl::PointXYZ>& legs,pcl::PointXYZ robotPos);		/***/
	void laserCallback(pcl::PointCloud<pcl::PointXYZ>::Ptr laserCart, pcl::PointCloud<pcl::PointXYZ>::Ptr laserCyl);
	void laserFilterMean(pcl::PointCloud<pcl::PointXYZ>::Ptr& laserCyl);
	bool findLegs(std::vector<pcl::PointXYZ>& legs, pcl::PointXYZ robotPos);		/***/
	bool findFrontLegs(double miX, double miY, pcl::PointXYZ robotPos);/****/
	bool findFrontLegs(std::vector<pcl::PointXYZ>& legs, double miX, double miY, pcl::PointXYZ robotPos);/****/
	bool findCenterLegs(std::vector<pcl::PointXYZ>& legs);
	bool isLeg(double x1, double y1, double x2, double y2);
};