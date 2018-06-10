#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <map>
#include <stdlib.h>
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include <ros/package.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "justina_tools/JustinaTools.h"
#include "geometry_msgs/Point.h"
#include "tf/transform_listener.h"
#include <cv_bridge/cv_bridge.h>

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include "vision_msgs/Cube.h"
#include "vision_msgs/CubesSegmented.h"
#include "vision_msgs/GetCubes.h"
#include "vision_msgs/FindPlane.h"
#include "vision_msgs/DetectObjects.h"

#include "visualization_msgs/MarkerArray.h"

using namespace std;
using namespace cv;

enum TYPE_CULTLERY{CUTLERY, BOWL, DISH, GLASS};

ros::NodeHandle* node;

ros::ServiceServer srvCubesSeg;
ros::ServiceServer srvCutlerySeg;
ros::ServiceClient cltRgbdRobot;
ros::ServiceClient cltFindPlane;
ros::ServiceClient cltExtObj;
ros::ServiceClient cltExtCut;
ros::Subscriber subCalibColor;
tf::TransformListener * transformListener;


visualization_msgs::MarkerArray cubesMarker;
std::map<std::string, visualization_msgs::Marker> cubesMapMarker;

int Hmin=0, Smin=0, Vmin=0, Hmax=0, Smax=0, Vmax=0;

float minX = 0.10, maxX = 1.0;
float minY = -0.5, maxY = 0.5;
float minZ = 0.65, maxZ = 2.0;

string colour;

std::stringstream Huemin, Huemax, Satmin, Satmax, Valmin, Valmax;

bool cropping = false;
bool getRoi = false;
bool getPointColor = false;
int xmin, ymin, xmax, ymax;
bool priorityFlag = true;

typedef struct _Data{
    int hmin;
    int hmax;
    int vmin;
    int vmax;
    int smin;
    int smax;
    std::vector<double> eigen_val;
    std::vector<cv::Vec2d> eigen_vec;
    double minArea;
    double maxArea;
} Data;

void drawAxis(Mat& img, Point p, Point q, Scalar colour, const float scale = 0.2){
    double angle;
    double hypotenuse;
    angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
    hypotenuse = sqrt( (double) (p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));
    double degrees = angle * 180 / CV_PI;
    q.x = (int) (p.x - scale * hypotenuse * cos(angle));
    q.y = (int) (p.y - scale * hypotenuse * sin(angle));
    cv::line(img, p, q, colour, 1, CV_AA);
    p.x = (int) (q.x + 9 * cos(angle + CV_PI / 4));
    p.y = (int) (q.y + 9 * sin(angle + CV_PI / 4));
    cv::line(img, p, q, colour, 1, CV_AA);
    p.x = (int) (q.x + 9 * cos(angle - CV_PI / 4));
    p.y = (int) (q.y + 9 * sin(angle - CV_PI / 4));
    cv::line(img, p, q, colour, 1, CV_AA);
}

void pcaAnalysis(cv::Mat &mask, cv::Mat &xyzCloud, float &roll, float &pitch, float &yaw){
    std::vector<cv::Vec3f> points;
    cv::Mat data_pts;
    for (int row = 0; row < mask.rows; ++row)
        for (int col = 0; col < mask.cols; ++col)
            if (mask.at<uchar>(row,col)>0){
                points.push_back(xyzCloud.at<cv::Vec3f>(row, col));
            }
    int sz = static_cast<int>(points.size());
    data_pts = cv::Mat(sz, 3, CV_64FC1);
    for(int i = 0; i < data_pts.rows; i++){
        cv::Vec3f p = points[i];
        data_pts.at<double>(i, 0) = static_cast<double>(p[0]);
        data_pts.at<double>(i, 1) = static_cast<double>(p[1]);
        data_pts.at<double>(i, 2) = static_cast<double>(p[2]);
    }
    cv::PCA pca_analysis(data_pts, cv::Mat(), CV_PCA_DATA_AS_ROW);
    cv::Vec3d cntr = cv::Point3d(pca_analysis.mean.at<double>(0, 0), pca_analysis.mean.at<double>(0, 1), pca_analysis.mean.at<double>(0, 2));

    std::vector<cv::Vec3d> eigen_vecs(3);
    std::vector<cv::Vec3d> eigen_vecsn(3);
    std::vector<double> eigen_val(3);
    for (int i = 0; i < 3; i++) {
        eigen_vecs[i] = cv::Vec3d(pca_analysis.eigenvectors.at<double>(i, 0), pca_analysis.eigenvectors.at<double>(i, 1), pca_analysis.eigenvectors.at<double>(i, 2));
        eigen_vecsn[i] = cv::normalize(eigen_vecs[i]);
        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
    }

    /*std::cout << "Eigen values: 0 " << eigen_val[0] << std::endl;
    std::cout << "Eigen values: 1 " << eigen_val[1] << std::endl;
    std::cout << "Eigen values: 2 " << eigen_val[2] << std::endl;*/

    /*
    if(eigen_val[1] / eigen_val[0] > 0.6 ){
        std::cout << "cubes_segmentation_node.->The main axis and the next axis of closest size is almost equal." << std::endl;
        roll = 0.0;
        pitch = 0.0;
        yaw = 1.5708;
    }
    else{
        std::cout << "cubes_segmentation_node.->The main axis is biger that the others axes" << std::endl;
        double angle = atan2(eigen_vecs[0][1], eigen_vecs[0][0]);
        roll = (float) angle;
        pitch = 0.0;
        yaw = 0.0;
    }*/
    if(eigen_val[1] / eigen_val[0] > 0.6 ){
        roll = 0.0;
        pitch = 0.0;
        yaw = 0.0;
    }
    else{
        double angle = -atan2(eigen_vecs[0][1], eigen_vecs[0][0]);
        /*if(angle >= M_PI)
          angle = M_PI - angle;*/
        roll = (float) angle;
        pitch = 0.0;
        yaw = 0.0;
    }
    std::cout << "cubes_segmentation_node.->Orientation of gripper, roll:" << roll << ", pitch:" << pitch << ", yaw:" << yaw  << std::endl;
}

void getPCAAnalysis(const std::vector<cv::Point> &pts, cv::Mat &img, std::vector<double> &eigenVal, std::vector<cv::Vec2d> &eigenVec){
    int sz = static_cast<int>(pts.size());
    cv::Mat data_pts = cv::Mat(sz, 2, CV_64FC1);
    for (int i = 0; i < data_pts.rows; ++i){
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }

    cv::PCA pca_analysis(data_pts, cv::Mat(), CV_PCA_DATA_AS_ROW);
    cv::Point cntr = cv::Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)), static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
    std::vector<cv::Vec2d> eigen_vecs(2);
    std::vector<double> eigen_val(2);
    for (int i = 0; i < 2; ++i){
        eigen_vecs[i] = cv::Vec2d(pca_analysis.eigenvectors.at<double>(i, 0), pca_analysis.eigenvectors.at<double>(i, 1));
        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
    }

    eigenVal = eigen_val;
    eigenVec = eigen_vecs;

    //cv::circle(img, cntr, 3, cv::Scalar(255, 0, 255), 2);
    cv::Point p1 = cntr + 0.02 * cv::Point(static_cast<int>(eigen_vecs[0][0] * eigen_val[0]), static_cast<int>(eigen_vecs[0][1] * eigen_val[0]));
    cv::Point p2 = cntr - 0.02 * cv::Point(static_cast<int>(eigen_vecs[1][0] * eigen_val[1]), static_cast<int>(eigen_vecs[1][1] * eigen_val[1]));
    drawAxis(img, cntr, p1, cv::Scalar(0, 255, 0), 1);
    drawAxis(img, cntr, p2, cv::Scalar(255, 255, 0), 5);
    //double angle = atan2(eigen_vecs[0][1], eigen_vecs[0][0]); // orientation in radians

    //std::cout << "Eigen values: 0 " << eigen_val[0] << std::endl;
    //std::cout << "Eigen values: 1 " << eigen_val[1] << std::endl;
}

void comparePCA(cv::Mat &mask, cv::Mat &xyzCloud, TYPE_CULTLERY &typeCutlery, float &roll, float &pitch, float &yaw){
    std::vector<cv::Vec3d> points;
    cv::Mat data_pts;
    for (int row = 0; row < mask.rows; ++row)
        for (int col = 0; col < mask.cols; ++col)
            if (mask.at<uchar>(row,col)>0)
                points.push_back(xyzCloud.at<cv::Vec3f>(row, col));
    int sz = static_cast<int>(points.size());
    data_pts = cv::Mat(sz, 3, CV_64FC1);
    std::cout << "size PCA:" << points.size() << std::endl;
    for(int i = 0; i < data_pts.rows; i++){
        cv::Vec3d p = points[i];
        data_pts.at<double>(i, 0) = static_cast<double>(p[0]);
        data_pts.at<double>(i, 1) = static_cast<double>(p[1]);
        data_pts.at<double>(i, 2) = static_cast<double>(p[2]);
    }
    cv::PCA pca_analysis(data_pts, cv::Mat(), CV_PCA_DATA_AS_ROW);
    cv::Vec3d cntr = cv::Point3d(pca_analysis.mean.at<double>(0, 0), pca_analysis.mean.at<double>(0, 1), pca_analysis.mean.at<double>(0, 2));

    std::vector<cv::Vec3d> eigen_vecs(3);
    std::vector<cv::Vec3d> eigen_vecsn(3);
    std::vector<double> eigen_val(3);
    for (int i = 0; i < 3; i++) {
        eigen_vecs[i] = cv::Vec3d(pca_analysis.eigenvectors.at<double>(i, 0), pca_analysis.eigenvectors.at<double>(i, 1), pca_analysis.eigenvectors.at<double>(i, 2));
        eigen_vecsn[i] = cv::normalize(eigen_vecs[i]);
        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
    }
    
    cv::Vec3d p1 = cntr + 100.0 * eigen_vecs[0] * eigen_val[0];
    cv::Vec3d p2 = cntr - 100.0 * eigen_vecs[1] * eigen_val[1];
    cv::Vec3d p3 = cntr + 100.0 * eigen_vecs[2] * eigen_val[2];
    printf("ComparePCA.->:ev0:%f, ev1:%f, ev2:%f \n", eigen_val[0], eigen_val[1], eigen_val[2]);
    printf("ComparePCA.->:eva0:%f, eva1:%f, eva2:%f \n", eigen_vecs[0][0], eigen_vecs[0][1], eigen_vecs[0][2]);
    printf("ComparePCA.->:eva0:%f, eva1:%f, eva2:%f \n", eigen_vecs[1][0], eigen_vecs[1][1], eigen_vecs[1][2]);
    printf("ComparePCA.->CNX:%f, CNY:%f, CNZ:%f \n", cntr[0], cntr[1], cntr[2]);
    printf("ComparePCA.->P1:%f, P1:%f, P1:%f \n", p1[0], p1[1], p1[2]);
    printf("ComparePCA.->P2:%f, P2:%f, P2:%f \n", p2[0], p2[1], p2[2]);

    double v2 = sqrt(pow(cntr[0] - p1[0], 2) + pow(cntr[1] - p1[1], 2) + pow(cntr[2] - p1[2], 2));
    double v1 = sqrt(pow(cntr[0] - p2[0], 2) + pow(cntr[1] - p2[1], 2) + pow(cntr[2] - p2[2], 2));
    double v0 = sqrt(pow(cntr[0] - p3[0], 2) + pow(cntr[1] - p3[1], 2) + pow(cntr[2] - p3[2], 2));
    
    if(points.size() < 1500){
        if(v1 < 0.008){
            typeCutlery = CUTLERY;
            double angle = -atan2(eigen_vecs[0][1], eigen_vecs[0][0]);
            /*if(angle >= M_PI)
              angle = M_PI - angle;*/
            roll = (float) angle;
            pitch = 0.0;
            yaw = 0.0;
        }else{
            typeCutlery = GLASS;
            roll = 0.0;
            pitch = 0.0;
            yaw = 1.5708;
        }
    }
    else{
        roll = 0.0;
        pitch = 0.0;
        yaw = 0.0;
        if(v1 < 0.1)
            typeCutlery = BOWL;
        else
            typeCutlery = DISH;
    }

    printf("ComparePCA.->v2:%f, v1:%f, v0%f\n", v2, v1, v0);

}

bool comparePCA2D(const std::vector<cv::Point> &pts, double area, Data data){
    int sz = static_cast<int>(pts.size());
    cv::Mat data_pts = cv::Mat(sz, 2, CV_64FC1);
    for (int i = 0; i < data_pts.rows; ++i){
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }

    cv::PCA pca_analysis(data_pts, cv::Mat(), CV_PCA_DATA_AS_ROW);
    cv::Point cntr = cv::Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)), static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
    std::vector<cv::Vec2d> eigen_vecs(2);
    std::vector<double> eigen_val(2);
    for (int i = 0; i < 2; ++i){
        eigen_vecs[i] = cv::Vec2d(pca_analysis.eigenvectors.at<double>(i, 0), pca_analysis.eigenvectors.at<double>(i, 1));
        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
    }
    std::cout << "minArea.->" << data.minArea << std::endl;
    std::cout << "maxArea.->" << data.maxArea << std::endl;
    std::cout << "area.->" << area << std::endl;
    if(area >= data.minArea && area <= data.maxArea)
        return true;
    return false;
}

bool comparePCA2D(const std::vector<cv::Point> &pts, double area, cv::Mat &img, Data data, double threshold){
    int sz = static_cast<int>(pts.size());
    cv::Mat data_pts = cv::Mat(sz, 2, CV_64FC1);
    for (int i = 0; i < data_pts.rows; ++i){
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }

    cv::PCA pca_analysis(data_pts, cv::Mat(), CV_PCA_DATA_AS_ROW);
    cv::Point cntr = cv::Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)), static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
    std::vector<cv::Vec2d> eigen_vecs(2);
    std::vector<double> eigen_val(2);
    for (int i = 0; i < 2; ++i){
        eigen_vecs[i] = cv::Vec2d(pca_analysis.eigenvectors.at<double>(i, 0), pca_analysis.eigenvectors.at<double>(i, 1));
        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
    }
    std::cout << "minArea.->" << data.minArea << std::endl;
    std::cout << "maxArea.->" << data.maxArea << std::endl;
    std::cout << "area.->" << area << std::endl;
    if(eigen_val.size() != data.eigen_val.size())
        return false;
    for(int i = 0; i <= data.eigen_val.size(); i++)
        if((data.eigen_val[i] > eigen_val[i] &&  data.eigen_val[i] / eigen_val[i] < threshold) || (data.eigen_val[i] < eigen_val[i] &&  eigen_val[i] / data.eigen_val[i] < threshold))
            return false;
    //if(!(area >= data.minArea && area <= data.maxArea))
        //return false;
    //std::cout << "Condición 1:" << (area > data.maxArea && data.maxArea / area < threshold) << std::endl; 
    //std::cout << "Condición 2:" << (area < data.maxArea && area / data.maxArea < threshold) << std::endl; 
    //if(area == 0 || (area > 0.0 && ((area > data.maxArea && data.maxArea / area < threshold) || (area < data.maxArea && area / data.maxArea < threshold))))
        //return false;
    if(abs(data.eigen_val[1] / data.eigen_val[0] - eigen_val[1] / eigen_val[0]) > 0.01)
        return false;
    cv::Point p1 = cntr + 0.02 * cv::Point(static_cast<int>(eigen_vecs[0][0] * eigen_val[0]), static_cast<int>(eigen_vecs[0][1] * eigen_val[0]));
    cv::Point p2 = cntr - 0.02 * cv::Point(static_cast<int>(eigen_vecs[1][0] * eigen_val[1]), static_cast<int>(eigen_vecs[1][1] * eigen_val[1]));
    drawAxis(img, cntr, p1, cv::Scalar(0, 255, 0), 1);
    drawAxis(img, cntr, p2, cv::Scalar(255, 255, 0), 5);
    return true;
}

void on_trackbar(int, void*) 
{
    /*if (bloques <= 1) {
      bloques = 3;
      } else {
      if (bloques % 2 != 1) {
      bloques += 1;
      }
      }*/
}

void on_mouse(int event, int x, int y, int flags, void* param) 
{
    if (event == CV_EVENT_LBUTTONDOWN) 
	{
		xmin = x;
		xmax = x;
		ymin = y;
		ymax = y;
		cropping = true;
	}
	else if (event == CV_EVENT_MOUSEMOVE && cropping) 
	{
		xmax = x;
		ymax = y;
	} 
	else if (event == CV_EVENT_LBUTTONUP) 
	{
		xmax = x;
		ymax = y;
		cropping = false;
        if(xmin >= xmax || ymin >= ymax){
		    getRoi = false;
            getPointColor = true;
        }
        else{
		    getRoi = true;
            getPointColor = false;
        }
	}
}

void loadValuesFromFile(string color, bool test)
{
	std::string configDir = ros::package::getPath("cubes_segmentation") + "/ConfigDir";
	std::string configFile;
	if( !boost::filesystem::exists(configDir ) )
		boost::filesystem::create_directory(configDir); 
	if (test)
		configFile = configDir + "/Cubes_config.xml";
	else
		configFile = configDir + "/Cutlery_config.xml";
	cv::FileStorage fs;


	Huemin << "H_min" << color;
    Huemax << "H_max" << color;
    Satmin << "S_min" << color;
    Satmax << "S_max" << color;
    Valmin << "V_min" << color;
    Valmax << "V_max" << color;

	fs.open(configFile, fs.READ );
    
	Hmin = (int)fs[Huemin.str()]; 
	Smin = (int)fs[Satmin.str()]; 
	Vmin = (int)fs[Valmin.str()]; 
	Hmax = (int)fs[Huemax.str()]; 
	Smax = (int)fs[Satmax.str()]; 
	Vmax = (int)fs[Valmax.str()]; 
		
	fs.release();

	Huemin.str(std::string());
	Huemax.str(std::string());
	Satmin.str(std::string());
	Satmax.str(std::string());
	Valmin.str(std::string());
	Valmax.str(std::string());

}

void loadValuesFromFile2(std::map<std::string, Data> &data, bool test)
{
	std::string configDir = ros::package::getPath("cubes_segmentation") + "/ConfigDir";
	std::string configFile;
	if( !boost::filesystem::exists(configDir ) )
		boost::filesystem::create_directory(configDir); 
	if (test)
		configFile = configDir + "/Cubes_config2.xml";
	else
		configFile = configDir + "/Cutlery_config2.xml";
	cv::FileStorage fs;
	fs.open(configFile, fs.READ );
    FileNode n = fs["trainning_ids"];

    if (n.type() != FileNode::MAP){
        cerr << "trainning_ids is not a sequence! FAIL" << endl;
        return;
    }

    FileNodeIterator it = n.begin(), it_end = n.end(); // Go through the node
    for (; it != it_end; ++it){
        FileNode n2 = *it;
        Data d;
        d.hmin = n2["huemin"];
        d.hmax = n2["huemax"];
        d.smin = n2["satmin"];
        d.smax = n2["satmax"];
        d.vmin = n2["valmin"];
        d.vmax = n2["valmax"];
        n2["eigen_vec"] >> d.eigen_vec;
        d.minArea = n2["min_area"];
        d.maxArea = n2["max_area"];
       
        FileNode n3 = n2["eigen_val"];                         // Read string sequence - Get node
        if (n3.type() == FileNode::SEQ){
            std::vector<double> eigen_val;
            FileNodeIterator it2 = n3.begin(), it_end2 = n3.end();
            for (; it2 != it_end2; ++it2)
                eigen_val.push_back((double)*it2);
            d.eigen_val = eigen_val;
        } 
        data[n2.name()] = d;
    }

    fs.release();

}

bool GetImagesFromJustina( cv::Mat& imaBGR, cv::Mat& imaPCL)
{
    point_cloud_manager::GetRgbd srv;
    if(!cltRgbdRobot.call(srv))
    {
        std::cout << "CubesSegmentation.->Cannot get point cloud" << std::endl;
        return false;
    }
    JustinaTools::PointCloud2Msg_ToCvMat(srv.response.point_cloud, imaBGR, imaPCL);
    return true; 
}

void callbackCalibrateCutlery(const std_msgs::String::ConstPtr& msg)
{
	std::cout << "CubesSegmentation.->Calibrate colour cutlery" << std::endl;
    bool init = false;
    colour = msg->data;
    cv::Mat bgrImg;
    cv::Mat xyzCloud;
    cv::Mat frameWork;
    cv::Mat frameHSV;
    cv::Mat maskRange;
    cv::Mat mask;

    loadValuesFromFile(colour, false);

    if(Hmin == 0 && Hmax == 0){
        Hmin = 255; Hmax = 0;
    }if(Smin == 0 && Smax == 0){
        Smin = 255; Smax = 0;
    }if(Vmin == 0 && Vmax == 0){
        Vmin = 255; Vmax = 0;
    }
  
    Huemin.str("");
    Huemax.str("");
    Satmin.str("");
    Satmax.str("");
    Valmin.str("");
    Valmax.str("");

    Huemin << "H_min" << colour;
    Huemax << "H_max" << colour;
    Satmin << "S_min" << colour;
    Satmax << "S_max" << colour;
    Valmin << "V_min" << colour;
    Valmax << "V_max" << colour;
    
    std::string configDir = ros::package::getPath("cubes_segmentation") + "/ConfigDir";
	if( !boost::filesystem::exists(configDir ) )
		boost::filesystem::create_directory(configDir); 

	std::string configFile =configDir + "/Cutlery_config.xml";
	cv::FileStorage fs;

	if(!boost::filesystem::exists(configFile))
	{
		fs.open(configFile, fs.WRITE);
		fs.release();	
	}
	
    ros::Rate loop(30);

    while(ros::ok() && cv::waitKey(1) != 'q')
	{
    	GetImagesFromJustina(bgrImg,xyzCloud);
    	bgrImg.copyTo(frameWork);
        blur( frameWork, frameWork, Size(4, 4) , Point(-1,-1) );

    	if(!cropping && !getRoi)
			imshow("Original", bgrImg);
		else if (cropping && !getRoi) 
		{
			cv::rectangle(bgrImg, cv::Point(xmin, ymin), cv::Point(xmax, ymax),cv::Scalar(0, 255, 0), 2);
			imshow("Original", bgrImg);
		} 
		else if (!cropping && getRoi) 
		{
			cv::rectangle(bgrImg, cv::Point(xmin, ymin), cv::Point(xmax, ymax),cv::Scalar(0, 255, 0), 2);
			imshow("Original", bgrImg);
		}

		cv::createTrackbar("HMIN", "Original", &Hmin, 255, on_trackbar);
		cv::createTrackbar("HMAX", "Original", &Hmax, 255, on_trackbar);
		cv::createTrackbar("SMIN", "Original", &Smin, 255, on_trackbar);
		cv::createTrackbar("SMAX", "Original", &Smax, 255, on_trackbar);
		cv::createTrackbar("VMIN", "Original", &Vmin, 255, on_trackbar);
		cv::createTrackbar("VMAX", "Original", &Vmax, 255, on_trackbar);
        if(!init){
            cvSetTrackbarPos("HMIN", "Original", Hmin);
            cvSetTrackbarPos("HMAX", "Original", Hmax);
            cvSetTrackbarPos("SMIN", "Original", Smin);
            cvSetTrackbarPos("SMAX", "Original", Smax);
            cvSetTrackbarPos("VMIN", "Original", Vmin);
            cvSetTrackbarPos("VMAX", "Original", Vmax);
            init = true;
        }
        setMouseCallback("Original", on_mouse, &bgrImg);

        if (getRoi) 
        {
            cv::Rect rect(xmin, ymin, xmax - xmin, ymax - ymin);
            cv::Mat roi = frameWork(rect);
            cv::Mat roiHSV;
            cv::cvtColor(roi, roiHSV, CV_BGR2HSV);
            std::vector<cv::Mat> channels;
            cv::split(roiHSV, channels);
            double minVal, maxVal;
            cv::Point minPos, maxPos;
            cv::minMaxLoc(channels[0], &minVal, &maxVal, &minPos, &maxPos);
			Hmin = minVal;
			Hmax = maxVal;
			cv::minMaxLoc(channels[1], &minVal, &maxVal, &minPos, &maxPos);
			Smin = minVal;
			Smax = maxVal;
			cv::minMaxLoc(channels[2], &minVal, &maxVal, &minPos, &maxPos);
			Vmin = minVal;
			Vmax = maxVal;
			cvSetTrackbarPos("HMIN", "Original", Hmin);
			cvSetTrackbarPos("HMAX", "Original", Hmax);
			cvSetTrackbarPos("SMIN", "Original", Smin);
			cvSetTrackbarPos("SMAX", "Original", Smax);
			cvSetTrackbarPos("VMIN", "Original", Vmin);
			cvSetTrackbarPos("VMAX", "Original", Vmax);
			cv::imshow("Roi", roi);
			cv::imshow("RoiHSV", roiHSV);
			getRoi = false;
		}
        if(getPointColor){
            
            float b = frameWork.at<cv::Vec3b>(ymin, xmin)[0];
            float g = frameWork.at<cv::Vec3b>(ymin, xmin)[1];
            float r = frameWork.at<cv::Vec3b>(ymin, xmin)[2];

            cv::Mat colorBGR = cv::Mat(1, 1, CV_8UC3);
            cv::Mat colorHSV = cv::Mat(1, 1, CV_8UC3);
            colorBGR.at<cv::Vec3b>(0, 0)[0] = frameWork.at<cv::Vec3b>(ymin, xmin)[0];
            colorBGR.at<cv::Vec3b>(0, 0)[1] = frameWork.at<cv::Vec3b>(ymin, xmin)[1];
            colorBGR.at<cv::Vec3b>(0, 0)[2] = frameWork.at<cv::Vec3b>(ymin, xmin)[2];
            cv::cvtColor(colorBGR, colorHSV, CV_BGR2HSV);

            if(colorHSV.at<cv::Vec3b>(0, 0)[0] < Hmin)
                Hmin = colorHSV.at<cv::Vec3b>(0, 0)[0];
            if(colorHSV.at<cv::Vec3b>(0, 0)[0] > Hmax)
                Hmax = colorHSV.at<cv::Vec3b>(0, 0)[0];
            if(colorHSV.at<cv::Vec3b>(0, 0)[1] < Smin)
                Smin = colorHSV.at<cv::Vec3b>(0, 0)[1];
            if(colorHSV.at<cv::Vec3b>(0, 0)[1] > Smax)
                Smax = colorHSV.at<cv::Vec3b>(0, 0)[1];
            if(colorHSV.at<cv::Vec3b>(0, 0)[2] < Vmin)
                Vmin = colorHSV.at<cv::Vec3b>(0, 0)[2];
            if(colorHSV.at<cv::Vec3b>(0, 0)[2] > Vmax)
                Vmax = colorHSV.at<cv::Vec3b>(0, 0)[2];

			cvSetTrackbarPos("HMIN", "Original", Hmin);
			cvSetTrackbarPos("HMAX", "Original", Hmax);
			cvSetTrackbarPos("SMIN", "Original", Smin);
			cvSetTrackbarPos("SMAX", "Original", Smax);
			cvSetTrackbarPos("VMIN", "Original", Vmin);
			cvSetTrackbarPos("VMAX", "Original", Vmax);
			getPointColor = false;
        }
        
		cv::cvtColor(frameWork, frameHSV, CV_BGR2HSV);
		cv::inRange(frameHSV, cv::Scalar(Hmin, Smin, Vmin),cv::Scalar(Hmax, Smax, Vmax), maskRange);
		cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(1.5, 1.5));
		cv::morphologyEx(maskRange, mask, cv::MORPH_ERODE, kernel, cv::Point(-1, -1), 1);
		cv::morphologyEx(mask, mask, cv::MORPH_DILATE, kernel, cv::Point(-1, -1), 7);

		cv::Mat maskedImage;
		frameWork.copyTo(maskedImage, mask);
		// Compute the centorid mask
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::Mat canny_output;
		mask.copyTo(canny_output);
		cv::imshow("Canny", canny_output);
		cv::findContours(canny_output, contours, hierarchy, CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

		if (contours.size() > 0) 
		{
			double maxArea = -1;
			int indexMaxArea = 0;
			for (unsigned int i = 0; i < contours.size(); i++) 
			{
				float area = cv::contourArea(contours[i]);
				if (area > maxArea) 
				{
					maxArea = area;
					indexMaxArea = i;
				}
			}
			std::vector<cv::Point> contour_poly;
			cv::approxPolyDP(cv::Mat(contours[indexMaxArea]), contour_poly, 3,true);
			cv::boundingRect(contour_poly);
			cv::rectangle(maskedImage, cv::boundingRect(contour_poly).tl(),cv::boundingRect(contour_poly).br(), CV_RGB(124, 40, 30), 2, 8, 0);
			cv::Moments centroide = moments(contours[indexMaxArea], false);
			cv::Point punto(centroide.m10 / centroide.m00, centroide.m01 / centroide.m00);
			cv::circle(maskedImage, punto, 4, CV_RGB(124, 40, 30), -1, 8, 0);
			// std::cout << cv::Mat(punto) << std::endl;

		}

		imshow("Color mask", mask);
		cv::imshow("Image with mask", maskedImage);

		if(cv::waitKey(1)=='s')
		{
			fs.open(configFile, fs.APPEND);

			fs<< Huemin.str() << Hmin;
			fs<< Huemax.str() << Hmax;
			fs<< Satmin.str() << Smin;
			fs<< Satmax.str() << Smax;
			fs<< Valmin.str() << Vmin;
			fs<< Valmax.str() << Vmax;
			
			fs.release();

			std::cout << colour << " Calibration Completed..." << std::endl;

			Huemin.str(std::string());
			Huemax.str(std::string());
			Satmin.str(std::string());
			Satmax.str(std::string());
			Valmin.str(std::string());
			Valmax.str(std::string());
			
		}
		
		ros::spinOnce();
        loop.sleep();
    }
}


void callbackCalibrateV3(const std_msgs::String::ConstPtr& msg){
    std::cout << "CubesSegmentation.->Calibrate colour cubes" << std::endl;
    bool init = false;
    std::string id = msg->data;
    cv::Mat bgrImg;
    cv::Mat xyzCloud;
    cv::Mat frameWork;
    cv::Mat frameHSV;
    cv::Mat maskRange;
    cv::Mat mask;
    bool loadValues = true;
    int Hmin=0, Smin=0, Vmin=0, Hmax=0, Smax=0, Vmax=0;
    

    std::map<std::string, Data> data;
    loadValuesFromFile2(data, true);

    std::map<std::string, Data>::iterator it = data.find(id);
    if(it != data.end()){
        Hmin = it->second.hmin;
        Hmax = it->second.hmax;
        Smin = it->second.smin;
        Smax = it->second.smax;
        Vmin = it->second.vmin;
        Vmax = it->second.vmax;
    }
    else{
        Hmin = 255; Hmax = 0;
        Smin = 255; Smax = 0;
        Vmin = 255; Vmax = 0;
        loadValues = false; 
    }

    std::string configDir = ros::package::getPath("cubes_segmentation") + "/ConfigDir";
    if( !boost::filesystem::exists(configDir ) )
        boost::filesystem::create_directory(configDir); 

    std::string configFile =configDir + "/Cubes_config2.xml";
    cv::FileStorage fs;

    if(!boost::filesystem::exists(configFile))
    {
        fs.open(configFile, fs.WRITE);
        fs.release();   
    }
    
    ros::Rate loop(30);

    while(ros::ok() && cv::waitKey(1) != 'q')
    {
        GetImagesFromJustina(bgrImg,xyzCloud);
        bgrImg.copyTo(frameWork);
        blur( frameWork, frameWork, Size(4, 4) , Point(-1,-1) );

        if(!cropping && !getRoi)
            imshow("Original", bgrImg);
        else if (cropping && !getRoi) 
        {
            cv::rectangle(bgrImg, cv::Point(xmin, ymin), cv::Point(xmax, ymax),cv::Scalar(0, 255, 0), 2);
            imshow("Original", bgrImg);
        } 
        else if (!cropping && getRoi) 
        {
            cv::rectangle(bgrImg, cv::Point(xmin, ymin), cv::Point(xmax, ymax),cv::Scalar(0, 255, 0), 2);
            imshow("Original", bgrImg);
        }

        cv::createTrackbar("HMIN", "Original", &Hmin, 255, on_trackbar);
        cv::createTrackbar("HMAX", "Original", &Hmax, 255, on_trackbar);
        cv::createTrackbar("SMIN", "Original", &Smin, 255, on_trackbar);
        cv::createTrackbar("SMAX", "Original", &Smax, 255, on_trackbar);
        cv::createTrackbar("VMIN", "Original", &Vmin, 255, on_trackbar);
        cv::createTrackbar("VMAX", "Original", &Vmax, 255, on_trackbar);
        if(!init){
            cvSetTrackbarPos("HMIN", "Original", Hmin);
            cvSetTrackbarPos("HMAX", "Original", Hmax);
            cvSetTrackbarPos("SMIN", "Original", Smin);
            cvSetTrackbarPos("SMAX", "Original", Smax);
            cvSetTrackbarPos("VMIN", "Original", Vmin);
            cvSetTrackbarPos("VMAX", "Original", Vmax);
            init = true;
        }
        setMouseCallback("Original", on_mouse, &bgrImg);

        if (getRoi) 
        {
            cv::Rect rect(xmin, ymin, xmax - xmin, ymax - ymin);
            cv::Mat roi = frameWork(rect);
            cv::Mat roiHSV;
            cv::cvtColor(roi, roiHSV, CV_BGR2HSV);
            std::vector<cv::Mat> channels;
            cv::split(roiHSV, channels);
            double minVal, maxVal;
            cv::Point minPos, maxPos;
            cv::minMaxLoc(channels[0], &minVal, &maxVal, &minPos, &maxPos);
            Hmin = minVal;
            Hmax = maxVal;
            cv::minMaxLoc(channels[1], &minVal, &maxVal, &minPos, &maxPos);
            Smin = minVal;
            Smax = maxVal;
            cv::minMaxLoc(channels[2], &minVal, &maxVal, &minPos, &maxPos);
            Vmin = minVal;
            Vmax = maxVal;
            cvSetTrackbarPos("HMIN", "Original", Hmin);
            cvSetTrackbarPos("HMAX", "Original", Hmax);
            cvSetTrackbarPos("SMIN", "Original", Smin);
            cvSetTrackbarPos("SMAX", "Original", Smax);
            cvSetTrackbarPos("VMIN", "Original", Vmin);
            cvSetTrackbarPos("VMAX", "Original", Vmax);
            cv::imshow("Roi", roi);
            cv::imshow("RoiHSV", roiHSV);
            getRoi = false;
        }
        if(getPointColor){
            
            float b = frameWork.at<cv::Vec3b>(ymin, xmin)[0];
            float g = frameWork.at<cv::Vec3b>(ymin, xmin)[1];
            float r = frameWork.at<cv::Vec3b>(ymin, xmin)[2];

            cv::Mat colorBGR = cv::Mat(1, 1, CV_8UC3);
            cv::Mat colorHSV = cv::Mat(1, 1, CV_8UC3);
            colorBGR.at<cv::Vec3b>(0, 0)[0] = frameWork.at<cv::Vec3b>(ymin, xmin)[0];
            colorBGR.at<cv::Vec3b>(0, 0)[1] = frameWork.at<cv::Vec3b>(ymin, xmin)[1];
            colorBGR.at<cv::Vec3b>(0, 0)[2] = frameWork.at<cv::Vec3b>(ymin, xmin)[2];
            cv::cvtColor(colorBGR, colorHSV, CV_BGR2HSV);

            if(colorHSV.at<cv::Vec3b>(0, 0)[0] < Hmin)
                Hmin = colorHSV.at<cv::Vec3b>(0, 0)[0];
            if(colorHSV.at<cv::Vec3b>(0, 0)[0] > Hmax)
                Hmax = colorHSV.at<cv::Vec3b>(0, 0)[0];
            if(colorHSV.at<cv::Vec3b>(0, 0)[1] < Smin)
                Smin = colorHSV.at<cv::Vec3b>(0, 0)[1];
            if(colorHSV.at<cv::Vec3b>(0, 0)[1] > Smax)
                Smax = colorHSV.at<cv::Vec3b>(0, 0)[1];
            if(colorHSV.at<cv::Vec3b>(0, 0)[2] < Vmin)
                Vmin = colorHSV.at<cv::Vec3b>(0, 0)[2];
            if(colorHSV.at<cv::Vec3b>(0, 0)[2] > Vmax)
                Vmax = colorHSV.at<cv::Vec3b>(0, 0)[2];

            cvSetTrackbarPos("HMIN", "Original", Hmin);
            cvSetTrackbarPos("HMAX", "Original", Hmax);
            cvSetTrackbarPos("SMIN", "Original", Smin);
            cvSetTrackbarPos("SMAX", "Original", Smax);
            cvSetTrackbarPos("VMIN", "Original", Vmin);
            cvSetTrackbarPos("VMAX", "Original", Vmax);
            getPointColor = false;
        }
        
        cv::cvtColor(frameWork, frameHSV, CV_BGR2HSV);
        cv::inRange(frameHSV, cv::Scalar(Hmin, Smin, Vmin),cv::Scalar(Hmax, Smax, Vmax), maskRange);
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(1.5, 1.5));
        cv::morphologyEx(maskRange, mask, cv::MORPH_ERODE, kernel, cv::Point(-1, -1), 1);
        cv::morphologyEx(mask, mask, cv::MORPH_DILATE, kernel, cv::Point(-1, -1), 7);

        cv::Mat maskedImage;
        std::vector<cv::Vec2d> eigen_vecs;
        std::vector<double> eigen_val;
        frameWork.copyTo(maskedImage, mask);
        // Compute the centorid mask
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::Mat canny_output;
        mask.copyTo(canny_output);
        cv::imshow("Canny", canny_output);
        cv::findContours(canny_output, contours, hierarchy, CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

        double mArea= -1;
        if (contours.size() > 0) 
        {
            int indexMaxArea = 0;
            for (unsigned int i = 0; i < contours.size(); i++) 
            {
                float area = cv::contourArea(contours[i]);
                if (area > mArea) 
                {
                    mArea = area;
                    indexMaxArea = i;
                }
            }
            std::vector<cv::Point> contour_poly;
            cv::approxPolyDP(cv::Mat(contours[indexMaxArea]), contour_poly, 3,true);
            cv::boundingRect(contour_poly);
            cv::rectangle(maskedImage, cv::boundingRect(contour_poly).tl(),cv::boundingRect(contour_poly).br(), CV_RGB(124, 40, 30), 2, 8, 0);
            cv::Moments centroide = moments(contours[indexMaxArea], false);
            cv::Point punto(centroide.m10 / centroide.m00, centroide.m01 / centroide.m00);
            cv::circle(maskedImage, punto, 4, CV_RGB(124, 40, 30), -1, 8, 0);
            cv::Mat boundingMask = cv::Mat::zeros(mask.size(), CV_8U);
            cv::fillConvexPoly(boundingMask, &contour_poly[0], (int)contour_poly.size(), 255, 8, 0);
            cv::bitwise_and(mask, boundingMask , mask);
            getPCAAnalysis(contour_poly, maskedImage, eigen_val, eigen_vecs);
            

        }

        imshow("Color mask", mask);
        cv::imshow("Image with mask", maskedImage);

        if(cv::waitKey(1)=='s')
        {

            std::map<std::string, Data>::iterator it = data.find(id);
            if(it == data.end()){
                std::cout << "No data found" << std::endl; 
                Data tr;
                tr.hmin = Hmin;
                tr.hmax = Hmax;
                tr.smin = Smin;
                tr.smax = Smax;
                tr.vmin = Vmin;
                tr.vmax = Vmax;
                data[id] = tr;
            }
            else{
                it->second.hmin = Hmin;
                it->second.hmax = Hmax;
                it->second.smin = Smin;
                it->second.smax = Smax;
                it->second.vmin = Vmin;
                it->second.vmax = Vmax;
            }

            fs.open(configFile, fs.WRITE);
            fs << "trainning_ids" << "{";

            for (std::map<std::string, Data>::iterator it= data.begin(); it != data.end(); ++it){
                fs << it->first << "{"; 
                fs << "huemin" << it->second.hmin;
                fs << "huemax" << it->second.hmax;
                fs << "satmin" << it->second.smin;
                fs << "satmax" << it->second.smax;
                fs << "valmin" << it->second.vmin;
                fs << "valmax" << it->second.vmax;
                fs << "}";
            }
            fs << "}";

            
            fs.release();

            std::cout << id << " Calibration Completed..." << std::endl;
            
        }
        
        ros::spinOnce();
        loop.sleep();
    }

}


void callbackCalibrateCutlery2(const std_msgs::String::ConstPtr& msg)
{
	std::cout << "CubesSegmentation.->Calibrate colour cutlery" << std::endl;
    bool init = false;
    std::string id = msg->data;
    cv::Mat bgrImg;
    cv::Mat xyzCloud;
    cv::Mat frameWork;
    cv::Mat frameHSV;
    cv::Mat maskRange;
    cv::Mat mask;
    bool loadValues = true;
    int Hmin=0, Smin=0, Vmin=0, Hmax=0, Smax=0, Vmax=0;
    double minArea, maxArea;

    std::map<std::string, Data> data;
    loadValuesFromFile2(data, false);

    std::map<std::string, Data>::iterator it = data.find(id);
    if(it != data.end()){
        Hmin = it->second.hmin;
        Hmax = it->second.hmax;
        Smin = it->second.smin;
        Smax = it->second.smax;
        Vmin = it->second.vmin;
        Vmax = it->second.vmax;
        minArea = it->second.minArea;
        maxArea = it->second.maxArea;
    }
    else{
        Hmin = 255; Hmax = 0;
        Smin = 255; Smax = 0;
        Vmin = 255; Vmax = 0;
        minArea = FLT_MAX , maxArea = -FLT_MAX;
        loadValues = false; 
    }

    /*Huemin << "H_min" << colour;
    Huemax << "H_max" << colour;
    Satmin << "S_min" << colour;
    Satmax << "S_max" << colour;
    Valmin << "V_min" << colour;
    Valmax << "V_max" << colour;*/
    
    std::string configDir = ros::package::getPath("cubes_segmentation") + "/ConfigDir";
	if( !boost::filesystem::exists(configDir ) )
		boost::filesystem::create_directory(configDir); 

	std::string configFile =configDir + "/Cutlery_config2.xml";
	cv::FileStorage fs;

	if(!boost::filesystem::exists(configFile))
	{
		fs.open(configFile, fs.WRITE);
		fs.release();	
	}
	
    ros::Rate loop(30);

    while(ros::ok() && cv::waitKey(1) != 'q')
	{
    	GetImagesFromJustina(bgrImg,xyzCloud);
    	bgrImg.copyTo(frameWork);
        blur( frameWork, frameWork, Size(4, 4) , Point(-1,-1) );

    	if(!cropping && !getRoi)
			imshow("Original", bgrImg);
		else if (cropping && !getRoi) 
		{
			cv::rectangle(bgrImg, cv::Point(xmin, ymin), cv::Point(xmax, ymax),cv::Scalar(0, 255, 0), 2);
			imshow("Original", bgrImg);
		} 
		else if (!cropping && getRoi) 
		{
			cv::rectangle(bgrImg, cv::Point(xmin, ymin), cv::Point(xmax, ymax),cv::Scalar(0, 255, 0), 2);
			imshow("Original", bgrImg);
		}

		cv::createTrackbar("HMIN", "Original", &Hmin, 255, on_trackbar);
		cv::createTrackbar("HMAX", "Original", &Hmax, 255, on_trackbar);
		cv::createTrackbar("SMIN", "Original", &Smin, 255, on_trackbar);
		cv::createTrackbar("SMAX", "Original", &Smax, 255, on_trackbar);
		cv::createTrackbar("VMIN", "Original", &Vmin, 255, on_trackbar);
		cv::createTrackbar("VMAX", "Original", &Vmax, 255, on_trackbar);
        if(!init){
            cvSetTrackbarPos("HMIN", "Original", Hmin);
            cvSetTrackbarPos("HMAX", "Original", Hmax);
            cvSetTrackbarPos("SMIN", "Original", Smin);
            cvSetTrackbarPos("SMAX", "Original", Smax);
            cvSetTrackbarPos("VMIN", "Original", Vmin);
            cvSetTrackbarPos("VMAX", "Original", Vmax);
            init = true;
        }
        setMouseCallback("Original", on_mouse, &bgrImg);

        if (getRoi) 
        {
            cv::Rect rect(xmin, ymin, xmax - xmin, ymax - ymin);
            cv::Mat roi = frameWork(rect);
            cv::Mat roiHSV;
            cv::cvtColor(roi, roiHSV, CV_BGR2HSV);
            std::vector<cv::Mat> channels;
            cv::split(roiHSV, channels);
            double minVal, maxVal;
            cv::Point minPos, maxPos;
            cv::minMaxLoc(channels[0], &minVal, &maxVal, &minPos, &maxPos);
			Hmin = minVal;
			Hmax = maxVal;
			cv::minMaxLoc(channels[1], &minVal, &maxVal, &minPos, &maxPos);
			Smin = minVal;
			Smax = maxVal;
			cv::minMaxLoc(channels[2], &minVal, &maxVal, &minPos, &maxPos);
			Vmin = minVal;
			Vmax = maxVal;
			cvSetTrackbarPos("HMIN", "Original", Hmin);
			cvSetTrackbarPos("HMAX", "Original", Hmax);
			cvSetTrackbarPos("SMIN", "Original", Smin);
			cvSetTrackbarPos("SMAX", "Original", Smax);
			cvSetTrackbarPos("VMIN", "Original", Vmin);
			cvSetTrackbarPos("VMAX", "Original", Vmax);
			cv::imshow("Roi", roi);
			cv::imshow("RoiHSV", roiHSV);
			getRoi = false;
		}
        if(getPointColor){
            
            float b = frameWork.at<cv::Vec3b>(ymin, xmin)[0];
            float g = frameWork.at<cv::Vec3b>(ymin, xmin)[1];
            float r = frameWork.at<cv::Vec3b>(ymin, xmin)[2];

            cv::Mat colorBGR = cv::Mat(1, 1, CV_8UC3);
            cv::Mat colorHSV = cv::Mat(1, 1, CV_8UC3);
            colorBGR.at<cv::Vec3b>(0, 0)[0] = frameWork.at<cv::Vec3b>(ymin, xmin)[0];
            colorBGR.at<cv::Vec3b>(0, 0)[1] = frameWork.at<cv::Vec3b>(ymin, xmin)[1];
            colorBGR.at<cv::Vec3b>(0, 0)[2] = frameWork.at<cv::Vec3b>(ymin, xmin)[2];
            cv::cvtColor(colorBGR, colorHSV, CV_BGR2HSV);

            if(colorHSV.at<cv::Vec3b>(0, 0)[0] < Hmin)
                Hmin = colorHSV.at<cv::Vec3b>(0, 0)[0];
            if(colorHSV.at<cv::Vec3b>(0, 0)[0] > Hmax)
                Hmax = colorHSV.at<cv::Vec3b>(0, 0)[0];
            if(colorHSV.at<cv::Vec3b>(0, 0)[1] < Smin)
                Smin = colorHSV.at<cv::Vec3b>(0, 0)[1];
            if(colorHSV.at<cv::Vec3b>(0, 0)[1] > Smax)
                Smax = colorHSV.at<cv::Vec3b>(0, 0)[1];
            if(colorHSV.at<cv::Vec3b>(0, 0)[2] < Vmin)
                Vmin = colorHSV.at<cv::Vec3b>(0, 0)[2];
            if(colorHSV.at<cv::Vec3b>(0, 0)[2] > Vmax)
                Vmax = colorHSV.at<cv::Vec3b>(0, 0)[2];

			cvSetTrackbarPos("HMIN", "Original", Hmin);
			cvSetTrackbarPos("HMAX", "Original", Hmax);
			cvSetTrackbarPos("SMIN", "Original", Smin);
			cvSetTrackbarPos("SMAX", "Original", Smax);
			cvSetTrackbarPos("VMIN", "Original", Vmin);
			cvSetTrackbarPos("VMAX", "Original", Vmax);
			getPointColor = false;
        }
        
		cv::cvtColor(frameWork, frameHSV, CV_BGR2HSV);
		cv::inRange(frameHSV, cv::Scalar(Hmin, Smin, Vmin),cv::Scalar(Hmax, Smax, Vmax), maskRange);
		cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(1.5, 1.5));
		cv::morphologyEx(maskRange, mask, cv::MORPH_ERODE, kernel, cv::Point(-1, -1), 1);
		cv::morphologyEx(mask, mask, cv::MORPH_DILATE, kernel, cv::Point(-1, -1), 7);

		cv::Mat maskedImage;
        std::vector<cv::Vec2d> eigen_vecs;
        std::vector<double> eigen_val;
		frameWork.copyTo(maskedImage, mask);
		// Compute the centorid mask
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::Mat canny_output;
		mask.copyTo(canny_output);
		cv::imshow("Canny", canny_output);
		cv::findContours(canny_output, contours, hierarchy, CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

        double mArea= -1;
		if (contours.size() > 0) 
		{
			int indexMaxArea = 0;
			for (unsigned int i = 0; i < contours.size(); i++) 
			{
				float area = cv::contourArea(contours[i]);
				if (area > mArea) 
				{
					mArea = area;
					indexMaxArea = i;
				}
			}
			std::vector<cv::Point> contour_poly;
			cv::approxPolyDP(cv::Mat(contours[indexMaxArea]), contour_poly, 3,true);
			cv::boundingRect(contour_poly);
			cv::rectangle(maskedImage, cv::boundingRect(contour_poly).tl(),cv::boundingRect(contour_poly).br(), CV_RGB(124, 40, 30), 2, 8, 0);
			cv::Moments centroide = moments(contours[indexMaxArea], false);
			cv::Point punto(centroide.m10 / centroide.m00, centroide.m01 / centroide.m00);
			cv::circle(maskedImage, punto, 4, CV_RGB(124, 40, 30), -1, 8, 0);
            cv::Mat boundingMask = cv::Mat::zeros(mask.size(), CV_8U);
            cv::fillConvexPoly(boundingMask, &contour_poly[0], (int)contour_poly.size(), 255, 8, 0);
            cv::bitwise_and(mask, boundingMask , mask);
            getPCAAnalysis(contour_poly, maskedImage, eigen_val, eigen_vecs);
            // std::cout << cv::Mat(punto) << std::endl;

		}

		imshow("Color mask", mask);
		cv::imshow("Image with mask", maskedImage);

		if(cv::waitKey(1)=='s')
		{

            std::map<std::string, Data>::iterator it = data.find(id);
            if(it == data.end()){
                std::cout << "No data found" << std::endl; 
                Data tr;
                tr.hmin = Hmin;
                tr.hmax = Hmax;
                tr.smin = Smin;
                tr.smax = Smax;
                tr.vmin = Vmin;
                tr.vmax = Vmax;
                tr.eigen_val = eigen_val;
                tr.eigen_vec = eigen_vecs;
                tr.minArea = mArea;
                tr.maxArea = mArea;
                data[id] = tr;
            }
            else{
                it->second.hmin = Hmin;
                it->second.hmax = Hmax;
                it->second.smin = Smin;
                it->second.smax = Smax;
                it->second.vmin = Vmin;
                it->second.vmax = Vmax;
                it->second.eigen_val = eigen_val;
                it->second.eigen_vec = eigen_vecs;
                std::cout << "Previous minArea:" << it->second.minArea << std::endl;
                std::cout << "Previous maxArea:" << it->second.maxArea << std::endl;
                if(mArea > it->second.maxArea)
                    it->second.maxArea = mArea;
                if(mArea < it->second.minArea)
                    it->second.minArea = mArea;
                std::cout << "Next minArea:" << it->second.minArea << std::endl;
                std::cout << "Next maxArea:" << it->second.maxArea << std::endl;
            }

            fs.open(configFile, fs.WRITE);
            fs << "trainning_ids" << "{";

            for (std::map<std::string, Data>::iterator it= data.begin(); it != data.end(); ++it){
                fs << it->first << "{"; 
                fs << "huemin" << it->second.hmin;
                fs << "huemax" << it->second.hmax;
                fs << "satmin" << it->second.smin;
                fs << "satmax" << it->second.smax;
                fs << "valmin" << it->second.vmin;
                fs << "valmax" << it->second.vmax;
                fs << "eigen_val" << it->second.eigen_val;
                fs << "eigen_vec" << it->second.eigen_vec;
                fs << "min_area" << it->second.minArea; 
                fs << "max_area" << it->second.maxArea;
                fs << "}";
            }
            fs << "}";

            /*fs<< Huemin.str() << Hmin;
              fs<< Huemax.str() << Hmax;
              fs<< Satmin.str() << Smin;
			fs<< Satmax.str() << Smax;
			fs<< Valmin.str() << Vmin;
			fs<< Valmax.str() << Vmax;*/
			
			fs.release();

			std::cout << id << " Calibration Completed..." << std::endl;

			/*Huemin.str(std::string());
			Huemax.str(std::string());
			Satmin.str(std::string());
			Satmax.str(std::string());
			Valmin.str(std::string());
			Valmax.str(std::string());*/
			
		}
		
		ros::spinOnce();
        loop.sleep();
    }
}


void callbackCalibrateV2(const std_msgs::String::ConstPtr& msg)
{
	std::cout << "CubesSegmentation.->Calibrate colour V2" << std::endl;
    colour = msg->data;
    cv::Mat bgrImg;
    cv::Mat xyzCloud;
    cv::Mat frameWork;
    cv::Mat frameHSV;
    cv::Mat maskRange;
    cv::Mat mask;

    Huemin << "H_min" << colour;
    Huemax << "H_max" << colour;
    Satmin << "S_min" << colour;
    Satmax << "S_max" << colour;
    Valmin << "V_min" << colour;
    Valmax << "V_max" << colour;

    
    
    std::string configDir = ros::package::getPath("cubes_segmentation") + "/ConfigDir";
	if( !boost::filesystem::exists(configDir ) )
		boost::filesystem::create_directory(configDir); 

	std::string configFile =configDir + "/Cubes_config.xml";
	cv::FileStorage fs;

	if(!boost::filesystem::exists(configFile))
	{
		fs.open(configFile, fs.WRITE);
		fs.release();	
	}
	
    ros::Rate loop(30);

    while(ros::ok() && cv::waitKey(1) != 'q')
	{
    	GetImagesFromJustina(bgrImg,xyzCloud);
    	bgrImg.copyTo(frameWork);

    	if(!cropping && !getRoi)
			imshow("Original", bgrImg);
		else if (cropping && !getRoi) 
		{
			cv::rectangle(bgrImg, cv::Point(xmin, ymin), cv::Point(xmax, ymax),cv::Scalar(0, 255, 0), 2);
			imshow("Original", bgrImg);
		} 
		else if (!cropping && getRoi) 
		{
			cv::rectangle(bgrImg, cv::Point(xmin, ymin), cv::Point(xmax, ymax),cv::Scalar(0, 255, 0), 2);
			imshow("Original", bgrImg);
		}

		cv::createTrackbar("HMIN", "Original", &Hmin, 255, on_trackbar);
		cv::createTrackbar("HMAX", "Original", &Hmax, 255, on_trackbar);
		cv::createTrackbar("SMIN", "Original", &Smin, 255, on_trackbar);
		cv::createTrackbar("SMAX", "Original", &Smax, 255, on_trackbar);
		cv::createTrackbar("VMIN", "Original", &Vmin, 255, on_trackbar);
		cv::createTrackbar("VMAX", "Original", &Vmax, 255, on_trackbar);
		setMouseCallback("Original", on_mouse, &bgrImg);

		if (getRoi) 
		{
			cv::Rect rect(xmin, ymin, xmax - xmin, ymax - ymin);
			cv::Mat roi = frameWork(rect);
			cv::Mat roiHSV;
			cv::cvtColor(roi, roiHSV, CV_BGR2HSV);
			std::vector<cv::Mat> channels;
			cv::split(roiHSV, channels);
			double minVal, maxVal;
			cv::Point minPos, maxPos;
			cv::minMaxLoc(channels[0], &minVal, &maxVal, &minPos, &maxPos);
			Hmin = minVal;
			Hmax = maxVal;
			cv::minMaxLoc(channels[1], &minVal, &maxVal, &minPos, &maxPos);
			Smin = minVal;
			Smax = maxVal;
			cv::minMaxLoc(channels[2], &minVal, &maxVal, &minPos, &maxPos);
			Vmin = minVal;
			Vmax = maxVal;
			cvSetTrackbarPos("HMIN", "Original", Hmin);
			cvSetTrackbarPos("HMAX", "Original", Hmax);
			cvSetTrackbarPos("SMIN", "Original", Smin);
			cvSetTrackbarPos("SMAX", "Original", Smax);
			cvSetTrackbarPos("VMIN", "Original", Vmin);
			cvSetTrackbarPos("VMAX", "Original", Vmax);
			cv::imshow("Roi", roi);
			cv::imshow("RoiHSV", roiHSV);
			getRoi = false;
		}

		cv::cvtColor(frameWork, frameHSV, CV_BGR2HSV);
		cv::inRange(frameHSV, cv::Scalar(Hmin, Smin, Vmin),cv::Scalar(Hmax, Smax, Vmax), maskRange);
		cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(1.5, 1.5));
		cv::morphologyEx(maskRange, mask, cv::MORPH_ERODE, kernel, cv::Point(-1, -1), 1);
		cv::morphologyEx(mask, mask, cv::MORPH_DILATE, kernel, cv::Point(-1, -1), 7);

		cv::Mat maskedImage;
		frameWork.copyTo(maskedImage, mask);
		// Compute the centorid mask
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::Mat canny_output;
		mask.copyTo(canny_output);
		cv::imshow("Canny", canny_output);
		cv::findContours(canny_output, contours, hierarchy, CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

		if (contours.size() > 0) 
		{
			double maxArea = -1;
			int indexMaxArea = 0;
			for (unsigned int i = 0; i < contours.size(); i++) 
			{
				float area = cv::contourArea(contours[i]);
				if (area > maxArea) 
				{
					maxArea = area;
					indexMaxArea = i;
				}
			}
			std::vector<cv::Point> contour_poly;
			cv::approxPolyDP(cv::Mat(contours[indexMaxArea]), contour_poly, 3,true);
			cv::boundingRect(contour_poly);
			cv::rectangle(maskedImage, cv::boundingRect(contour_poly).tl(),cv::boundingRect(contour_poly).br(), CV_RGB(124, 40, 30), 2, 8, 0);
			cv::Moments centroide = moments(contours[indexMaxArea], false);
			cv::Point punto(centroide.m10 / centroide.m00, centroide.m01 / centroide.m00);
			cv::circle(maskedImage, punto, 4, CV_RGB(124, 40, 30), -1, 8, 0);
			std::cout << cv::Mat(punto) << std::endl;

		}

		imshow("Color mask", mask);
		cv::imshow("Image with mask", maskedImage);

		if(cv::waitKey(1)=='s')
		{
			fs.open(configFile, fs.APPEND);

			fs<< Huemin.str() << Hmin;
			fs<< Huemax.str() << Hmax;
			fs<< Satmin.str() << Smin;
			fs<< Satmax.str() << Smax;
			fs<< Valmin.str() << Vmin;
			fs<< Valmax.str() << Vmax;
			
			fs.release();

			std::cout << colour << " Calibration Completed..." << std::endl;

			Huemin.str(std::string());
			Huemax.str(std::string());
			Satmin.str(std::string());
			Satmax.str(std::string());
			Valmin.str(std::string());
			Valmax.str(std::string());
			
		}
		
		ros::spinOnce();
        loop.sleep();
    }
}

void callbackStartCalibrate(const std_msgs::String::ConstPtr& msg)
{
	std::cout << "CubesSegmentation.->Calibrate colour" << std::endl;
    colour = msg->data;
    cv::Mat bgrImg;
    cv::Mat xyzCloud;
    cv::Mat imageHSV;
    cv::Mat imageSegmentada;
    cv::Mat maskRange;

    Huemin << "H_min" << colour;
    Huemax << "H_max" << colour;
    Satmin << "S_min" << colour;
    Satmax << "S_max" << colour;
    Valmin << "V_min" << colour;
    Valmax << "V_max" << colour;

    
    
    std::string configDir = ros::package::getPath("cubes_segmentation") + "/ConfigDir";
	if( !boost::filesystem::exists(configDir ) )
		boost::filesystem::create_directory(configDir); 

	std::string configFile =configDir + "/Cubes_config.xml";
	cv::FileStorage fs;

	if(!boost::filesystem::exists(configFile))
	{
		fs.open(configFile, fs.WRITE);
		fs.release();	
	}
	
    ros::Rate loop(30);

	while(ros::ok() && cv::waitKey(1) != 'q')
	{
    	GetImagesFromJustina(bgrImg,xyzCloud);
    	imshow("calibrate", bgrImg);
    	createTrackbar("HMIN", "calibrate", &Hmin, 255, on_trackbar);
		createTrackbar("HMAX", "calibrate", &Hmax, 255, on_trackbar);
		createTrackbar("SMIN", "calibrate", &Smin, 255, on_trackbar);
		createTrackbar("SMAX", "calibrate", &Smax, 255, on_trackbar);
		createTrackbar("VMIN", "calibrate", &Vmin, 255, on_trackbar);
		createTrackbar("VMAX", "calibrate", &Vmax, 255, on_trackbar);


		cvtColor(bgrImg, imageHSV, CV_BGR2HSV);
		inRange(imageHSV, Scalar(Hmin, Smin, Vmin),Scalar(Hmax, Smax, Vmax), imageSegmentada);
		erode(imageSegmentada, maskRange, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(maskRange, maskRange,getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		
		imshow("Color mask", maskRange);
		cv::Mat maskedImage;
		bgrImg.copyTo(maskedImage, maskRange);
		imshow("Image with mask", maskedImage);

		if(cv::waitKey(1)=='s')
		{
			fs.open(configFile, fs.APPEND );

			fs<< Huemin.str() << Hmin;
			fs<< Huemax.str() << Hmax;
			fs<< Satmin.str() << Smin;
			fs<< Satmax.str() << Smax;
			fs<< Valmin.str() << Vmin;
			fs<< Valmax.str() << Vmax;
			
			fs.release();

			std::cout << colour << " Calibration Completed..." << std::endl;

			Huemin.str(std::string());
			Huemax.str(std::string());
			Satmin.str(std::string());
			Satmax.str(std::string());
			Valmin.str(std::string());
			Valmax.str(std::string());
			
		}

		ros::spinOnce();
        loop.sleep();
	}
}

bool setDeepthWindow()
{
	std::cout << "CubesSegmentation.->Trying to find a plane" << std::endl;

	point_cloud_manager::GetRgbd srv;
    vision_msgs::FindPlane fp;
    fp.request.name = "";

    if(!cltRgbdRobot.call(srv))
    {
        std::cout << "CubesSegmentation.->Cannot get point cloud :'(" << std::endl;
        return false;
    }

    fp.request.point_cloud = srv.response.point_cloud;

    if(!cltFindPlane.call(fp))
    {
        std::cout << "CubesSegmentation.->Cannot find a plane" << std::endl;
        return false;
    }
    std::cout << "CubesSegmentation.->Find a plane" << std::endl;

    minX = fp.response.nearestPoint.x;
    maxX = minX + 1.0;
    minY = fp.response.nearestPoint.y - 0.5;
    maxY = fp.response.nearestPoint.y + 0.5;
    minZ = fp.response.nearestPoint.z + 0.0005;
    maxZ = fp.response.nearestPoint.z + 1.0;

    std::cout << "minX: " << minX << std::endl;
    std::cout << "minY: " << minY << std::endl;
    std::cout << "minZ: " << minZ << std::endl;
    return true;
} 


bool callback_srvCubeSeg(vision_msgs::GetCubes::Request &req, vision_msgs::GetCubes::Response &resp)
{

	cv::Vec3f aux (0.0, 0.0, 0.0);
	cv::Vec3f centroid (0.0, 0.0, 0.0); 

	cv::Mat bgrImg;
    cv::Mat xyzCloud;
    cv::Mat bgrImgCopy;
    cv::Mat imageHSV;

    GetImagesFromJustina(bgrImg,xyzCloud);

    /*cv::cvtColor(bgrImg,imageHSV,CV_BGR2HSV);
    cv::Mat globalmask = cv::Mat::zeros(imageHSV.size(),CV_8U);
    cv::bitwise_not(globalmask,globalmask);*/

    bgrImg.copyTo(bgrImgCopy);
    blur(bgrImgCopy, bgrImgCopy, Size(4, 4) , Point(-1,-1) );
    cv::cvtColor(bgrImgCopy,imageHSV,CV_BGR2HSV);
    cv::Mat globalmask = cv::Mat::zeros(imageHSV.size(),CV_8U);
    cv::bitwise_not(globalmask,globalmask);

    vision_msgs::CubesSegmented cubes = req.cubes_input;
        
    //inRange(imageHSV,Scalar(0,70,50), Scalar(0,255,255),maskHSV);

    vector <cv::Point> centroidList;
    std::vector<std::vector<cv::Point> > contoursRec;
    std::vector<cv::Scalar> colors;
    geometry_msgs::Point minP, maxP;

    vision_msgs::DetectObjects srv;
    if(!cltExtObj.call(srv))
    {
        std::cout << "cubes_segmentation_node.-> Cannot extract a object above planes" << std::endl;
        return false;
    }
    sensor_msgs::ImageConstPtr objExtrMaskConsPtr( new sensor_msgs::Image( srv.response.image ) );
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(objExtrMaskConsPtr, sensor_msgs::image_encodings::TYPE_8UC1);
    cv::Mat objExtrMask = cv_ptr->image;

    bgrImg.copyTo(bgrImgCopy);
    
    std::map<std::string, Data> data;
    loadValuesFromFile2(data, true);

    for(int i = 0; i < cubes.recog_cubes.size(); i++)
    {

    	minP.x=10.0;
    	minP.y=10.0;
    	minP.z=10.0;
    	maxP.x=0.3;
    	maxP.y=0.3;
    	maxP.z=0.3;

    	cv::Mat maskHSV;
    	vision_msgs::Cube cube = cubes.recog_cubes[i];

        std::map<std::string, Data>::iterator it = data.find(cubes.recog_cubes[i].color);

        if(it == data.end())
            continue;
    	
    	//inRange(imageHSV,Scalar(Hmin, Smin, Vmin), Scalar(Hmax,Smax,Vmax),maskHSV);//color rojo
        inRange(imageHSV,Scalar(it->second.hmin, it->second.smin, it->second.vmin), Scalar(it->second.hmax, it->second.smax, it->second.vmax),maskHSV);//color rojo
    	cv::Mat maskXYZ;
		cv::inRange(xyzCloud,cv::Scalar(minX, minY,minZ),cv::Scalar(maxX,maxY,maxZ),maskXYZ);
        cv::imshow("In range image", maskXYZ);

		cv::Mat mask;
		//maskXYZ.copyTo(mask,maskHSV);
		maskHSV.copyTo(mask);
        cv::Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(1.5, 1.5));
		cv::morphologyEx(mask,mask,cv::MORPH_ERODE,kernel,cv::Point(-1,-1),1);
		cv::morphologyEx(mask,mask,cv::MORPH_DILATE,kernel,cv::Point(-1,-1),7);

        //cv::bitwise_and(mask, objExtrMask , mask);

		// Compute the centorid mask
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::Mat canny_output;
		mask.copyTo(canny_output);
		cv::findContours(canny_output, contours, hierarchy, CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
		if (contours.size() == 0){
			cube.detected_cube  = false;
			cube.cube_centroid.x = 0.0;
			cube.cube_centroid.y = 0.0;
			cube.cube_centroid.z = 0.0;
            resp.cubes_output.recog_cubes.push_back(cube);	
            continue;
        }

        double maxArea = -1;
        int indexMaxArea = 0;
        for (unsigned int contour = 0; contour < contours.size(); contour++) 
        {
            float area = cv::contourArea(contours[contour]);
            if (area > maxArea) 
            {
                maxArea = area;
                indexMaxArea = contour;
            }
        }
        std::vector<cv::Point> contour_poly;
        cv::approxPolyDP(cv::Mat(contours[indexMaxArea]), contour_poly, 3,true);
        //cv::boundingRect(contour_poly);
        cv::Mat boundingMask = cv::Mat::zeros(mask.size(), CV_8U);
        cv::fillConvexPoly(boundingMask, &contour_poly[0], (int)contour_poly.size(), 255, 8, 0);
        //cv::bitwise_and(mask, boundingMask , mask);
        boundingMask.copyTo(mask);
        //cv::imshow("Mask", mask);
        		
		cv::Point imgCentroid(0,0);
		int numPoints = 0;

        maskXYZ.copyTo(mask, mask);
        cv::bitwise_and(mask, objExtrMask, mask);

		bool firstData = false;
        for (int row = 0; row < mask.rows; ++row)
		{
			for (int col = 0; col < mask.cols; ++col)
			{
				if (mask.at<uchar>(row,col)>0)
				{
					//centroid += xyzCloud.at<cv::Vec3f>(i,j);
					aux = xyzCloud.at<cv::Vec3f>(row,col);
					centroid += aux;
					imgCentroid += cv::Point(col,row);
					++numPoints;
    
                    if(!firstData){
                        firstData = true;
                        minP.x = aux.val[0]; 
                        maxP.x = aux.val[0]; 
                        minP.y = aux.val[1]; 
                        maxP.y = aux.val[1]; 
                        minP.z = aux.val[2]; 
                        maxP.z = aux.val[2]; 
                    }
                    else{
                        if(minP.x > aux.val[0])
                            minP.x = aux.val[0];

                        if(minP.y > aux.val[1])
                            minP.y = aux.val[1];

                        if(minP.z > aux.val[2])
                            minP.z = aux.val[2];

                        if(maxP.x < aux.val[0])
                            maxP.x = aux.val[0];

                        if(maxP.y < aux.val[1])
                            maxP.y = aux.val[1];

                        if(maxP.z < aux.val[2])
                            maxP.z = aux.val[2];
                    }
                }
			}
		}

		if (numPoints == 0)
		{
			std::cout << "CubesSegmentation.->Cannot get centroid " << std::endl;
			cube.detected_cube  = false;
			cube.cube_centroid.x = 0.0;
			cube.cube_centroid.y = 0.0;
			cube.cube_centroid.z = 0.0;
		}
		else
		{
			centroid /= numPoints;
			imgCentroid /= numPoints;
			centroidList.push_back(imgCentroid);
            contoursRec.push_back(contour_poly);
			std::cout << "CubesSegmentation.->Centroid:" << centroid << std::endl;
			std::cout << "CubesSegmentation.->MinP:[" << minP << "]" << std::endl;
			std::cout << "CubesSegmentation.->MaxP:[" << maxP << "]" << std::endl;
			cube.detected_cube = true;
			cube.cube_centroid.x = centroid[0];
			cube.cube_centroid.y = centroid[1];
			cube.cube_centroid.z = centroid[2];

			cube.minPoint = minP;
			cube.maxPoint = maxP;

            tf::StampedTransform transform;
            transformListener->waitForTransform("map", "base_link", ros::Time(0), ros::Duration(10.0));
            transformListener->lookupTransform("map", "base_link", ros::Time(0), transform);

            tf::Vector3 cubePosWrtRobot((minP.x + maxP.x) / 2.0f, (minP.y + maxP.y) / 2.0f, (minP.z + maxP.z) / 2.0f);
            tf::Vector3 cubePosWrtWorld = transform * cubePosWrtRobot;

            cv::Mat colorBGR = cv::Mat(1, 1, CV_8UC3);
            cv::Mat colorHSV = cv::Mat(1, 1, CV_8UC3);
            colorHSV.at<cv::Vec3b>(0, 0)[0] = (it->second.hmin + it->second.hmax) / 2.0f;
            colorHSV.at<cv::Vec3b>(0, 0)[1] = (it->second.smin + it->second.smax) / 2.0f;
            colorHSV.at<cv::Vec3b>(0, 0)[2] = (it->second.vmin + it->second.vmax) / 2.0f;
            cv::cvtColor(colorHSV, colorBGR, CV_HSV2BGR);
            colors.push_back(cv::Scalar(colorBGR.at<cv::Vec3b>(0, 0)[0], colorBGR.at<cv::Vec3b>(0, 0)[1], colorBGR.at<cv::Vec3b>(0, 0)[2]));

            cube.colorRGB.x = colorBGR.at<cv::Vec3b>(0, 0)[2] / 255.0f;
            cube.colorRGB.y = colorBGR.at<cv::Vec3b>(0, 0)[1] / 255.0f;
            cube.colorRGB.z = colorBGR.at<cv::Vec3b>(0, 0)[0] / 255.0f;
 
            std::map<std::string, visualization_msgs::Marker>::iterator cubeIt = cubesMapMarker.find(cube.color);
            if(cubeIt == cubesMapMarker.end()){
                visualization_msgs::Marker marker;
                marker.header.frame_id = "map";
                marker.header.stamp = ros::Time();
                marker.ns = "cubes_marker";
                marker.id = i;
                marker.type = visualization_msgs::Marker::CYLINDER;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = cubePosWrtWorld.x();
                marker.pose.position.y = cubePosWrtWorld.y();
                marker.pose.position.z = cubePosWrtWorld.z();
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = fabs(minP.x - maxP.x);
                marker.scale.y = fabs(minP.y - maxP.y);
                marker.scale.z = fabs(minP.z - maxP.z);
                marker.color.a = 0.8;
                marker.color.r = colorBGR.at<cv::Vec3b>(0, 0)[2] / 255.0f;
                marker.color.g = colorBGR.at<cv::Vec3b>(0, 0)[1] / 255.0f;
                marker.color.b = colorBGR.at<cv::Vec3b>(0, 0)[0] / 255.0f;
                cubesMapMarker[cube.color] = marker; 
            }
            else{
                visualization_msgs::Marker marker = cubeIt->second;
                marker.pose.position.x = cubePosWrtWorld.x();
                marker.pose.position.y = cubePosWrtWorld.y();
                marker.pose.position.z = cubePosWrtWorld.z();
                marker.scale.x = fabs(minP.x - maxP.x);
                marker.scale.y = fabs(minP.y - maxP.y);
                marker.scale.z = fabs(minP.z - maxP.z);
                cubesMapMarker[cube.color] = marker; 
            }
		}

		cv::bitwise_not(boundingMask,boundingMask);
        cv::bitwise_and(boundingMask,globalmask,globalmask);
		//imshow("mask", mask);
		
		resp.cubes_output.recog_cubes.push_back(cube);	
		//mask.copyTo(globalmask, globalmask);
    }
    cv::bitwise_not(globalmask,globalmask);
	//imshow("globalmask", globalmask);
    cv::Mat maskedImage;
	bgrImg.copyTo(maskedImage,globalmask);
	for(int i=0; i<centroidList.size(); i++)
	{
		cv::circle(maskedImage, centroidList[i],5, colors[i], -1);
        cv::rectangle(maskedImage, cv::boundingRect(contoursRec[i]).tl(), cv::boundingRect(contoursRec[i]).br(), colors[i], 2, 8, 0);
	}
	imshow("global",maskedImage);
    imshow("Original image", bgrImgCopy);
    return true;
}

bool callback_srvCutlerySeg(vision_msgs::GetCubes::Request &req, vision_msgs::GetCubes::Response &resp)
{
	cv::Vec3f aux (0.0, 0.0, 0.0);
	cv::Vec3f centroid (0.0, 0.0, 0.0); 

	cv::Mat bgrImg;
    cv::Mat xyzCloud;
    cv::Mat bgrImgCopy;
    cv::Mat imageHSV;

    GetImagesFromJustina(bgrImg,xyzCloud);

    bgrImg.copyTo(bgrImgCopy);
    blur(bgrImgCopy, bgrImgCopy, Size(4, 4) , Point(-1,-1) );
    cv::cvtColor(bgrImgCopy,imageHSV,CV_BGR2HSV);
    cv::Mat globalmask = cv::Mat::zeros(imageHSV.size(),CV_8U);
    cv::bitwise_not(globalmask,globalmask);

    vision_msgs::CubesSegmented cubes = req.cubes_input;
        
    //inRange(imageHSV,Scalar(0,70,50), Scalar(0,255,255),maskHSV);

    vector <cv::Point> centroidList;
    std::vector<std::vector<cv::Point> > contoursRec;
    std::vector<cv::Scalar> colors;
    geometry_msgs::Point minP, maxP;

    vision_msgs::DetectObjects srv;
    if(!cltExtCut.call(srv))
    {
        std::cout << "cutlery_segmentation_node.-> Cannot extract a object with planes" << std::endl;
        return false;
    }
    sensor_msgs::ImageConstPtr objExtrMaskConsPtr( new sensor_msgs::Image( srv.response.image ) );
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(objExtrMaskConsPtr, sensor_msgs::image_encodings::TYPE_8UC1);
    cv::Mat objExtrMask = cv_ptr->image;

    bgrImg.copyTo(bgrImgCopy);
    
    std::map<std::string, Data> data;
    loadValuesFromFile2(data, false);

    for(int i = 0; i < cubes.recog_cubes.size(); i++)
    {

    	minP.x=10.0;
    	minP.y=10.0;
    	minP.z=10.0;
    	maxP.x=0.3;
    	maxP.y=0.3;
    	maxP.z=0.3;

    	cv::Mat maskHSV;
    	vision_msgs::Cube cube = cubes.recog_cubes[i];

        std::map<std::string, Data>::iterator it = data.find(cubes.recog_cubes[i].color);

        if(it == data.end())
            continue;

    	inRange(imageHSV,Scalar(it->second.hmin, it->second.smin, it->second.vmin), Scalar(it->second.hmax, it->second.smax, it->second.vmax),maskHSV);//color rojo
    	cv::Mat maskXYZ;
		cv::inRange(xyzCloud,cv::Scalar(minX, minY,minZ),cv::Scalar(maxX,maxY,maxZ),maskXYZ);
        cv::imshow("In range image", maskXYZ);

		cv::Mat mask;
		//maskXYZ.copyTo(mask,maskHSV);
        maskHSV.copyTo(mask);
		cv::Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(1.5, 1.5));
		cv::morphologyEx(mask,mask,cv::MORPH_ERODE,kernel,cv::Point(-1,-1),1);
		cv::morphologyEx(mask,mask,cv::MORPH_DILATE,kernel,cv::Point(-1,-1),7);

        // cv::bitwise_and(mask, objExtrMask , mask);

		// Compute the centorid mask
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::Mat canny_output;
		mask.copyTo(canny_output);
		cv::findContours(canny_output, contours, hierarchy, CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
		if (contours.size() == 0){
			cube.detected_cube  = false;
			cube.cube_centroid.x = 0.0;
			cube.cube_centroid.y = 0.0;
			cube.cube_centroid.z = 0.0;
            resp.cubes_output.recog_cubes.push_back(cube);	
            continue;
        }

        double maxArea = -1;
        int indexMaxArea = 0;
        for (unsigned int contour = 0; contour < contours.size(); contour++) 
        {
            float area = cv::contourArea(contours[contour]);
            if (area > maxArea) 
            {
                maxArea = area;
                indexMaxArea = contour;
            }
        }
        std::vector<cv::Point> contour_poly;
        cv::approxPolyDP(cv::Mat(contours[indexMaxArea]), contour_poly, 3,true);
        cv::Mat boundingMask = cv::Mat::zeros(mask.size(), CV_8U);
        cv::fillConvexPoly(boundingMask, &contour_poly[0], (int)contour_poly.size(), 255, 8, 0);
        // cv::fillPoly(boundingMask, contour_poly.data(), (int)contour_poly.size(), 1, cv::Scalar(255, 255, 255), 8);
        // cv::bitwise_or(mask, boundingMask , mask);
        boundingMask.copyTo(mask);

		cv::Point imgCentroid(0,0);
		int numPoints = 0;

        maskXYZ.copyTo(mask, mask);
        cv::bitwise_and(mask, objExtrMask, mask);
		bool firstData = false;
        for (int row = 0; row < mask.rows; ++row)
		{
			for (int col = 0; col < mask.cols; ++col)
			{
				if (mask.at<uchar>(row,col)>0)
				{
					//centroid += xyzCloud.at<cv::Vec3f>(i,j);
					aux = xyzCloud.at<cv::Vec3f>(row,col);
					centroid += aux;
					imgCentroid += cv::Point(col,row);
					++numPoints;
    
                    if(!firstData){
                        firstData = true;
                        minP.x = aux.val[0]; 
                        maxP.x = aux.val[0]; 
                        minP.y = aux.val[1]; 
                        maxP.y = aux.val[1]; 
                        minP.z = aux.val[2]; 
                        maxP.z = aux.val[2]; 
                    }
                    else{
                        if(minP.x > aux.val[0])
                            minP.x = aux.val[0];

                        if(minP.y > aux.val[1])
                            minP.y = aux.val[1];

                        if(minP.z > aux.val[2])
                            minP.z = aux.val[2];

                        if(maxP.x < aux.val[0])
                            maxP.x = aux.val[0];

                        if(maxP.y < aux.val[1])
                            maxP.y = aux.val[1];

                        if(maxP.z < aux.val[2])
                            maxP.z = aux.val[2];
                    }
                }
			}
		}

        bool isGlass = comparePCA2D(contour_poly, maxArea, it->second);

		if (!isGlass && numPoints <= 200 || numPoints == 0)
		{
			std::cout << "CutlerySegmentation.->Cannot get centroid " << std::endl;
			cube.detected_cube  = false;
			cube.cube_centroid.x = 0.0;
			cube.cube_centroid.y = 0.0;
			cube.cube_centroid.z = 0.0;
		}
		else
		{
			centroid /= numPoints;
			imgCentroid /= numPoints;
			centroidList.push_back(imgCentroid);
            contoursRec.push_back(contour_poly);
			/*std::cout << "CutlerySegmentation.->Centroid:" << centroid << std::endl;
			std::cout << "CutlerySegmentation.->MinP:[" << minP << "]" << std::endl;
			std::cout << "CutlerySegmentation.->MaxP:[" << maxP << "]" << std::endl;*/
			cube.detected_cube = true;
			cube.cube_centroid.x = centroid[0];
			cube.cube_centroid.y = centroid[1];
			cube.cube_centroid.z = centroid[2];

			cube.minPoint = minP;
			cube.maxPoint = maxP;

            cv::Mat colorBGR = cv::Mat(1, 1, CV_8UC3);
            cv::Mat colorHSV = cv::Mat(1, 1, CV_8UC3);
            colorHSV.at<cv::Vec3b>(0, 0)[0] = (it->second.hmin + it->second.hmax) / 2.0f;
            colorHSV.at<cv::Vec3b>(0, 0)[1] = (it->second.smin + it->second.smax) / 2.0f;
            colorHSV.at<cv::Vec3b>(0, 0)[2] = (it->second.vmin + it->second.vmax) / 2.0f;
            cv::cvtColor(colorHSV, colorBGR, CV_HSV2BGR);
            colors.push_back(cv::Scalar(colorBGR.at<cv::Vec3b>(0, 0)[0], colorBGR.at<cv::Vec3b>(0, 0)[1], colorBGR.at<cv::Vec3b>(0, 0)[2]));

            std::cout << "Color.->" << it->first << std::endl;
            float roll, pitch, yaw;
            float rate = 1.0;
            TYPE_CULTLERY typeCutlery;
            // comparePCA2(contours[indexMaxArea], bgrImg, typeCutlery);
            cv::bitwise_not(boundingMask, boundingMask);
            cv::bitwise_and(boundingMask, globalmask, globalmask);
            if(isGlass && numPoints <= 200){
                roll = 0.0;
                pitch = 0.0;
                yaw = 1.5708;
                typeCutlery = GLASS;
            }
            else
                comparePCA(mask, xyzCloud, typeCutlery, roll, pitch, yaw);
            cube.roll = roll;
            cube.pitch = pitch;
            cube.yaw = yaw;
			cv::boundingRect(contour_poly);
			cv::rectangle(bgrImgCopy, cv::boundingRect(contour_poly).tl(),cv::boundingRect(contour_poly).br(), CV_RGB(colorBGR.at<cv::Vec3b>(0, 0)[2], colorBGR.at<cv::Vec3b>(0, 0)[1], colorBGR.at<cv::Vec3b>(0, 0)[0]), 2, 8, 0);
            std::stringstream ss;
            ss << it->first;
            switch(typeCutlery){
                case DISH:
                    ss << "_dish";
                    rate = 1.0;
                    break;
                case CUTLERY:
                    ss << "_cutlery";
                    rate= 0.8;
                    break;
                case GLASS:
                    ss << "_glass";
                    rate=0.4;
                    break;
                case BOWL:
                    ss << "_bowl";
                    rate=0.6;
                    break;
                default:
                    break;
            }
            cv::putText(bgrImgCopy, ss.str(), cv::Point(cv::boundingRect(contour_poly).tl().x, cv::boundingRect(contour_poly).br().y + 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,255) );

            cube.type_object = typeCutlery;

            cube.priority = (priorityFlag) ? (cube.cube_centroid.x * rate) : (cube.cube_centroid.x * 1.0); 
		}
		
		resp.cubes_output.recog_cubes.push_back(cube);	
    }
    cv::bitwise_not(globalmask,globalmask);
	//imshow("globalmask", globalmask);
    cv::Mat maskedImage;
	bgrImg.copyTo(maskedImage,globalmask);
	for(int i=0; i<centroidList.size(); i++){
		cv::circle(maskedImage, centroidList[i],5, colors[i], -1);
        cv::rectangle(maskedImage, cv::boundingRect(contoursRec[i]).tl(), cv::boundingRect(contoursRec[i]).br(), colors[i], 2, 8, 0);
	}
	imshow("global",maskedImage);
	imshow("Original image", bgrImgCopy);
    return true;
}

bool callback_srvCutlerySeg2(vision_msgs::GetCubes::Request &req, vision_msgs::GetCubes::Response &resp)
{

	cv::Vec3f aux (0.0, 0.0, 0.0);
	cv::Vec3f centroid (0.0, 0.0, 0.0); 

	cv::Mat bgrImg;
    cv::Mat xyzCloud;
    cv::Mat bgrImgCopy;
    cv::Mat imageHSV;

    GetImagesFromJustina(bgrImg,xyzCloud);

    bgrImg.copyTo(bgrImgCopy);
    blur(bgrImgCopy, bgrImgCopy, Size(4, 4) , Point(-1,-1) );
    cv::cvtColor(bgrImgCopy,imageHSV,CV_BGR2HSV);
    cv::Mat globalmask = cv::Mat::zeros(imageHSV.size(),CV_8U);
    cv::bitwise_not(globalmask,globalmask);
    cv::imshow("Global Mask init", globalmask);
    
    cv::Mat generalMask = cv::Mat::ones(imageHSV.size(), CV_8UC1);

    vision_msgs::CubesSegmented cubes = req.cubes_input;
        
    //inRange(imageHSV,Scalar(0,70,50), Scalar(0,255,255),maskHSV);

    vector <cv::Point> centroidList;
    std::vector<std::vector<cv::Point> > contoursRec;
    std::vector<cv::Scalar> colors;
    geometry_msgs::Point minP, maxP;

    vision_msgs::DetectObjects srv;
    if(!cltExtCut.call(srv))
    {
        std::cout << "cutlery_segmentation_node.-> Cannot extract a object with planes" << std::endl;
        return false;
    }
    sensor_msgs::ImageConstPtr objExtrMaskConsPtr( new sensor_msgs::Image( srv.response.image ) );
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(objExtrMaskConsPtr, sensor_msgs::image_encodings::TYPE_8UC1);
    cv::Mat objExtrMask = cv_ptr->image;

    std::map<std::string, Data> data;
    loadValuesFromFile2(data, false);
   
    int match = 0;
    for (std::map<std::string, Data>::iterator it= data.begin(); it != data.end(); ++it)
    {
        std::stringstream ss;
    	minP.x=10.0;
    	minP.y=10.0;
    	minP.z=10.0;
    	maxP.x=0.3;
    	maxP.y=0.3;
    	maxP.z=0.3;

    	cv::Mat maskHSV;
    	vision_msgs::Cube cube;
        cube.color = it->first;

        cv::inRange(imageHSV,Scalar(it->second.hmin, it->second.smin, it->second.vmin), Scalar(it->second.hmax,it->second.smax,it->second.vmax),maskHSV);
    	cv::Mat maskXYZ;
		//cv::inRange(xyzCloud,cv::Scalar(minX, minY,minZ),cv::Scalar(maxX,maxY,maxZ),maskXYZ);
        //cv::imshow("In range image", maskXYZ);

        cv::Mat mask = cv::Mat::zeros(imageHSV.size(), CV_8UC1);
		//maskXYZ.copyTo(mask,maskHSV);
		maskHSV.copyTo(mask);
        //mask.copyTo(mask, generalMask);
		cv::Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(1.5, 1.5));
		cv::morphologyEx(mask, mask, cv::MORPH_ERODE,kernel, cv::Point(-1,-1),1);
		cv::morphologyEx(mask, mask, cv::MORPH_DILATE,kernel, cv::Point(-1,-1),7);
        cv::bitwise_and(mask, globalmask, mask);

        //cv::bitwise_and(mask, objExtrMask , mask);

		// Compute the centorid mask
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::Mat canny_output;
		mask.copyTo(canny_output);
		cv::findContours(canny_output, contours, hierarchy, CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
		if (contours.size() == 0){
			cube.detected_cube  = false;
			cube.cube_centroid.x = 0.0;
			cube.cube_centroid.y = 0.0;
			cube.cube_centroid.z = 0.0;
            resp.cubes_output.recog_cubes.push_back(cube);	
            continue;
        }

        double maxArea = -1;
        int indexMaxArea = 0;
        for (unsigned int contour = 0; contour < contours.size(); contour++){
            std::vector<cv::Point> contour_poly;
            cv::approxPolyDP(cv::Mat(contours[contour]), contour_poly, 3,true);
            cv::Mat boundingMask = cv::Mat::zeros(mask.size(), CV_8U);
            cv::fillConvexPoly(boundingMask, &contour_poly[0], (int)contour_poly.size(), 255, 8, 0);
            cv::bitwise_or(mask, boundingMask , mask);
            
            float area = cv::contourArea(contours[contour]);

            if(!comparePCA2D(contour_poly, area, bgrImg, it->second, 0.75))
                continue;

			cv::boundingRect(contour_poly);
			cv::rectangle(bgrImg, cv::boundingRect(contour_poly).tl(),cv::boundingRect(contour_poly).br(), CV_RGB(124, 40, 30), 2, 8, 0);
            cv::putText(bgrImg, it->first, cv::Point(cv::boundingRect(contour_poly).tl().x, cv::boundingRect(contour_poly).br().y + 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,255) );
			//cv::Moments centroide = moments(contours[indexMaxArea], false);
			//cv::Point punto(centroide.m10 / centroide.m00, centroide.m01 / centroide.m00);
			//cv::circle(maskedImage, punto, 4, CV_RGB(124, 40, 30), -1, 8, 0);

            cv::Point imgCentroid(0,0);
            int numPoints = 0;
            bool firstData = false;
            for (int row = 0; row < mask.rows; ++row){
                for (int col = 0; col < mask.cols; ++col){
                    if (mask.at<uchar>(row,col) > 0){
                        aux = xyzCloud.at<cv::Vec3f>(row,col);
                        centroid += aux;
                        imgCentroid += cv::Point(col,row);
                        ++numPoints;

                        if(!firstData){
                            firstData = true;
                            minP.x = aux.val[0]; 
                            maxP.x = aux.val[0]; 
                            minP.y = aux.val[1]; 
                            maxP.y = aux.val[1]; 
                            minP.z = aux.val[2]; 
                            maxP.z = aux.val[2]; 
                        }
                        else{
                            if(minP.x > aux.val[0])
                                minP.x = aux.val[0];

                            if(minP.y > aux.val[1])
                                minP.y = aux.val[1];

                            if(minP.z > aux.val[2])
                                minP.z = aux.val[2];

                            if(maxP.x < aux.val[0])
                                maxP.x = aux.val[0];

                            if(maxP.y < aux.val[1])
                                maxP.y = aux.val[1];

                            if(maxP.z < aux.val[2])
                                maxP.z = aux.val[2];
                        }
                    }
                }
            }
            ss.str("");
            ss << "Mask HSV " << it->first;
            cv::imshow(ss.str(), maskHSV);
            ss.str("");
            ss << "Mask " << it->first << " " << contour << std::endl;
            cv::imshow(ss.str(), mask);
            cv::bitwise_not(mask, mask);
            cv::bitwise_and(globalmask, mask, globalmask);

            if (area > maxArea) 
            {
                maxArea = area;
                indexMaxArea = contour;
            }
        }

        //cv::rectangle(bgrImg, cv::boundingRect(contour_poly).tl(),cv::boundingRect(contour_poly).br(), CV_RGB(124, 40, 30), 2, 8, 0);
        /*cv::Moments centroide = moments(contours[indexMaxArea], false);
          cv::Point punto(centroide.m10 / centroide.m00, centroide.m01 / centroide.m00);
          cv::circle(maskedImage, punto, 4, CV_RGB(124, 40, 30), -1, 8, 0);*/



		/*if (numPoints == 0)
		{
			std::cout << "CutlerySegmentation.->Cannot get centroid " << std::endl;
			cube.detected_cube  = false;
			cube.cube_centroid.x = 0.0;
			cube.cube_centroid.y = 0.0;
			cube.cube_centroid.z = 0.0;
		}
		else
		{
			centroid /= numPoints;
			imgCentroid /= numPoints;
			centroidList.push_back(imgCentroid);
            //contoursRec.push_back(contour_poly);
			std::cout << "CutlerySegmentation.->Centroid:" << centroid << std::endl;
			std::cout << "CutlerySegmentation.->MinP:[" << minP << "]" << std::endl;
			std::cout << "CutlerySegmentation.->MaxP:[" << maxP << "]" << std::endl;
			cube.detected_cube = true;
			cube.cube_centroid.x = centroid[0];
			cube.cube_centroid.y = centroid[1];
			cube.cube_centroid.z = centroid[2];

			cube.minPoint = minP;
			cube.maxPoint = maxP;

            //float roll, pitch, yaw;
            //pcaAnalysis(mask, xyzCloud, roll, pitch, yaw);
            //cube.roll = roll;
            //cube.pitch = pitch;
            //cube.yaw = yaw;

            //cube.type_object = (roll==0.0 && pitch==0 && yaw==0.0) ? 1 : 0;
            //if(roll==0.0 && pitch==0 && yaw==0.0)
            //	cube.type_object = 0;
            //else
            //	cube.type_object = 1;

		}*/

		//cv::bitwise_not(mask,mask);
		
		resp.cubes_output.recog_cubes.push_back(cube);	
		//mask.copyTo(globalmask, globalmask);
    }
    //cv::bitwise_not(globalmask,globalmask);
    //cv::Mat maskedImage;
	//bgrImg.copyTo(maskedImage,globalmask);
	//for(int i=0; i<centroidList.size(); i++)
	//{
	//	cv::circle(maskedImage, centroidList[i],5, colors[i], -1);
    //   cv::rectangle(maskedImage, cv::boundingRect(contoursRec[i]).tl(), cv::boundingRect(contoursRec[i]).br(), colors[i], 2, 8, 0);
	//}
	imshow("Cutlery Reco", bgrImg);
	imshow("General Mask", globalmask);
    return true;
}

int main(int argc, char** argv)
{
	
    std::cout << "Initializing Cubes Segmentation by Hugo..." << std::endl;
    ros::init(argc, argv, "cubes_segmentation");
    ros::NodeHandle n;
    node = &n;
    
    srvCubesSeg = n.advertiseService("/vision/cubes_segmentation/cubes_seg", callback_srvCubeSeg);
    srvCutlerySeg = n.advertiseService("/vision/cubes_segmentation/cutlery_seg", callback_srvCutlerySeg);
    cltRgbdRobot = n.serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot");
    cltFindPlane = n.serviceClient<vision_msgs::FindPlane>("/vision/geometry_finder/findPlane");
    cltExtObj = n.serviceClient<vision_msgs::DetectObjects>("/vision/obj_reco/ext_objects_above_planes");
    cltExtCut = n.serviceClient<vision_msgs::DetectObjects>("/vision/obj_reco/ext_objects_with_planes");
    ros::Subscriber subStartCalib = n.subscribe("/vision/cubes_segmentation/start_calib", 1, callbackStartCalibrate);
    ros::Subscriber subCalibV2 = n.subscribe("/vision/cubes_segmentation/calibv2", 1, callbackCalibrateV3);
    ros::Subscriber subCalibCutlery = n.subscribe("/vision/cubes_segmentation/calibCutlery", 1, callbackCalibrateCutlery2);
    ros::Publisher pubCubesMarker = n.advertise<visualization_msgs::MarkerArray>("/vision/cubes_segmentation/cubes_markers", 1);

    ros::Rate loop(30);
    
    std::cout << "CubesSegmentation.->Running..." << std::endl;
    
    transformListener = new tf::TransformListener();
    
    while(ros::ok() && cv::waitKey(1) != 'q')
    {
        /*cubesMarker.markers.clear();
        for(std::map<std::string, visualization_msgs::Marker>::iterator it = cubesMapMarker.begin(); it != cubesMapMarker.end(); it++)
            cubesMarker.markers.push_back(it->second);
        pubCubesMarker.publish(cubesMarker);*/
        ros::spinOnce();
        loop.sleep();
    }
    
    delete transformListener;

    cv::destroyAllWindows();   
}
