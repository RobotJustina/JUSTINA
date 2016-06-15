#include <iostream>
#include <vector>
#include <cmath>
#include <pcl/io/openni_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#define FILTER_THRESHOLD  .081
#define FLANK_THRESHOLD  .04
#define HORIZON_THRESHOLD  25
#define MAX_FLOAT  57295779500
#define PIERNA_DELGADA  0.006241//7.9CM,0.006241,6241
#define PIERNA_GRUESA  0.037//19.23CM,0.037,37000
#define DOS_PIERNAS_DELGADAS  0.056644//23.8CM,0.056644,56644
#define DOS_PIERNAS_GRUESAS  0.25//50CM,0.25,250000
#define DOS_PIERNAS_CERCAS  0.022201//14.9CM,0.022201,22201
#define DOS_PIERNAS_LEJOS  0.16//40CM,0.16,160000

#define LEG_IN_FRONT_X_RANGE 1.5
#define LEG_IN_FRONT_Y_RANGE 0.3

//BUTTER FILTER A Ó B EN X O Y
//cutoff frequency X: 0.7
//                 Y: 0.2
//lowpass filter

#define BFA0X 1.0
#define BFA1X 1.161917483671732
#define BFA2X 0.695942755789651
#define BFA3X 0.137761301259893
#define BFB0X 0.374452692590159
#define BFB1X 1.123358077770478
#define BFB2X 1.123358077770478
#define BFB3X 0.374452692590159
#define BFA0Y 1.0
#define BFA1Y -1.760041880343169
#define BFA2Y 1.182893262037831
#define BFA3Y -0.278059917634546
#define BFB0Y 0.018098933007514
#define BFB1Y 0.054296799022543
#define BFB2Y 0.054296799022543
#define BFB3Y 0.018098933007514



class LegFinder
{
private:
    std::vector<pcl::PointXYZ> rec; //The recognized leg positions. Are stored in values wrt robot
    pcl::PointXYZ hum;
    pcl::PointXYZ lastHum;
    float umbraldis;
    float robotX, robotY, robotTheta;
    bool motionlessLegInFront;
    int legsInFrontCounter;
    std::vector<float> legsY;
    std::vector<float> filteredLegsY;
    std::vector<float> legsX;
    std::vector<float> filteredLegsX;


public:
    LegFinder();
    ~LegFinder();
    

    bool findBestLegs(std::vector<float>& laser_ranges, std::vector<float>& laser_angles, pcl::PointXYZ& ten, float& distan);
    bool findLegs(std::vector<float>& laser_ranges, std::vector<float>& laser_angles,
                  std::vector<pcl::PointXYZ>& legs, int opc = 0, float miX = 0, float miY = 0);

    void laserCallback(std::vector<float>& laser_ranges, std::vector<float>& laser_angles);
    void laserFilter_Mean(std::vector<float>& vector_r);
    bool findPiernas(std::vector<pcl::PointXYZ>& piernas);
    bool findPiernasFrente(float miX, float miY);
    bool findPiernasFrente(std::vector<pcl::PointXYZ>& legs, float miX, float miY);
    bool findPiernasFrente(pcl::PointXYZ& legPose, float miX, float miY);
    bool findPiernasCentrada(std::vector<pcl::PointXYZ>& piernas);
    bool esPierna(float x1, float y1, float x2, float y2);
    void setRobotPose(float robotX, float robotY, float robotTheta);
    bool isThereMotionlessLegInFront();
};
