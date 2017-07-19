#include "ros/ros.h"

#include "std_msgs/Bool.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud2.h"

#include "justina_tools/JustinaTools.h"

ros::Subscriber subPointCloud;
ros::Publisher pub_nearestDetect;
ros::NodeHandle * nh_ptr;
geometry_msgs::Point32 refPoint;
bool enableDetect = false;
bool detected = false;

bool initThreshold = false;
//This is the for hardcode for the optimal PCL in bounding box
//int threshhold = 5000;
int threshold = 0;

// NEAREST_DETECT
bool enaNearestDetect = false; 
bool debug = true; 
cv::Scalar frontLeftBotBBPoint = cv::Scalar(0.25, -0.5, 0.7);
cv::Scalar backRigthTopBBPoint = cv::Scalar(0.4,  0.5, 1.3); 
int yLimit=350;		//BGR Limit

typedef struct BoundingBox {
    float w, h, l;
    cv::Point3f center;
} BoundingBox;

geometry_msgs::Point32 NeaerestDetect(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    cv::Mat imaBGR; 
    cv::Mat imaXYZ; 
    JustinaTools::PointCloud2Msg_ToCvMat(msg, imaBGR, imaXYZ);

    geometry_msgs::Point32 edgePoint; 

    // Segmenting by BBox 
    cv::Mat validMask; 
    cv::inRange(imaXYZ, frontLeftBotBBPoint, backRigthTopBBPoint, validMask);  

    cv::Mat validXYZ;
    imaXYZ.copyTo( validXYZ, validMask);
    int pclCount = countNonZero( validMask );

    cv::Mat bgrCanny;
    cv::RNG rng(12345);
    std::vector<std::vector<cv::Point> > contours;

    // Contours
    findContours( validMask, contours, cv::RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    // Draw contours
    cv::Mat drawing = cv::Mat::zeros( validMask.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ )
    {
        cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        drawContours( drawing, contours, i, color, 2 );
    }

    // Contours analysis
    float num_pixel = 0;
    float euc, etmp;
    std::vector<float> xps,yps;
    cv::Point THEPOINT;
    cv::Mat centroid = cv::Mat::zeros( validMask.size(), CV_8UC3 );
    cv::line(centroid, cv::Point(0, yLimit), cv::Point(centroid.cols, yLimit), cv::Scalar( 0, 0, 255));
    for(unsigned int i=0;i<contours.size();i++)
    {
        cv::Point p2c;
        bool st1=false, st2=false;
        cv::Moments m = moments(contours[i], false);
        // Centroid
        cv::Point p(m.m10/m.m00, m.m01/m.m00);
        if(p.y > yLimit) //Too close to robot (img start from 0, far from robot), ideally with point cloud distances, but not yet implemented
        {
            circle(centroid, p, 5, cv::Scalar( 0, 0, 255), -1);
            continue;
        }
        // Area
        if(m.m00<4000){ // Low pixel density
            circle(centroid, p, 5, cv::Scalar( 0, 0, 255), -1);
            continue;
        }

        cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        drawContours( centroid, contours, i, color, 2 );
        circle(centroid, p, 5, cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255)), -1);
        // XYZ analisys, the nearest point to the robot its stored to further comparison
        etmp=9999999.9;
        for(unsigned int j=0;j<contours[i].size();j++){
            cv::Point3f poi=imaXYZ.at<cv::Point3f>( contours[i][j] );
            euc = sqrt(poi.x*poi.x + poi.y*poi.y);
            if(fabs(euc) < etmp){
                etmp=fabs(euc);
                p2c = contours[i][j];
            }
        }

        circle( centroid, p2c, 5, cv::Scalar( 255,0,0 ), -1 ); 

        xps.push_back(p2c.x);//Vector of maximum points (one per contour i)
        yps.push_back(p2c.y);
    }


    //Maximum points comparison
    euc=0;etmp=9.999999;
    cv::Point p2c;
    for(unsigned int i=0;i<xps.size();i++)
    {
        cv::Point3f poi=imaXYZ.at<cv::Point3f>( xps[i] , yps[i] );
        euc = sqrt(poi.x*poi.x + poi.y*poi.y);
        if(fabs(euc) < etmp){
            etmp=fabs(euc);
            THEPOINT.x = xps[i];
            THEPOINT.y = yps[i];
        }
    }
    circle(centroid, THEPOINT, 5, cv::Scalar(255,255,255), 3);

    // debug
    if( debug ) 
    {
        imshow( "centroids", centroid );
        imshow( "Contours", drawing );
        cv::Mat bgrWithMask;
        imaBGR.copyTo( bgrWithMask, validMask ); 
        cv::imshow( "bgrWithMask", bgrWithMask );
        //std::cout << "Numbers of pts in BBox: " << pclCount << std::endl; 
    }

    cv::Point3f poi=imaXYZ.at<cv::Point3f>( THEPOINT );
    edgePoint.x=poi.x;
    edgePoint.y=poi.y;
    edgePoint.z=poi.z;
    pub_nearestDetect.publish(edgePoint);
    return edgePoint; 
}

void callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {

    if( enaNearestDetect )
        NeaerestDetect( msg ); 

    if(enableDetect){

        cv::Mat imaBGR;
        cv::Mat imaPCL;
        int pclCount = 0;

        cv::Point3f handRobotPosition;
        handRobotPosition.x = refPoint.x;
        handRobotPosition.y = refPoint.y;
        handRobotPosition.z = refPoint.z;

        BoundingBox bb;
        bb.center = handRobotPosition;
        bb.w = 0.15;
        bb.h = 0.3;
        bb.l = 0.45;

        JustinaTools::PointCloud2Msg_ToCvMat(msg, imaBGR, imaPCL);
        for (int i = 0; i < imaPCL.rows; i++) {
            for (int j = 0; j < imaPCL.cols; j++) {
                cv::Point3f point = imaPCL.at<cv::Point3f>(i, j);
                if (point.x >= bb.center.x 
                        && point.x <= bb.center.x + bb.l / 2
                        && point.y >= bb.center.y - bb.w / 2
                        && point.y <= bb.center.y + bb.w / 2
                        && point.z >= bb.center.z - bb.h / 2
                        && point.z <= bb.center.z + bb.h / 2) {
                    pclCount++;
                } else {
                    imaBGR.at<cv::Vec3b>(i, j) = cv::Vec3b(0.0, 0.0, 0.0);
                }
            }
        }

        if(!initThreshold && pclCount > 300){
            initThreshold = true;
            threshold = pclCount;
            std::cout << "HandDetect.->threshhold:" << threshold << std::endl; 
            return;
        }

        cv::imshow("Hand Detect", imaBGR);
        //std::cout << "HandDetect.->Number of pcl in BB:" << pclCount << std::endl;

        if (pclCount > 1.25 * threshold) {
            detected = true;
            std::cout << "HandDetect.->The Bounding box is fill" << std::endl;
        }
        else
            detected = false;
    }
}

void callbackStartFrontRecog(const geometry_msgs::Point32::ConstPtr& msg) {
    std::cout << "HandDetect.->Starting Hand Detect in BB..."
        << std::endl;
    subPointCloud = nh_ptr->subscribe(
            "/hardware/point_cloud_man/rgbd_wrt_robot", 1, callbackPointCloud);
    refPoint.x = msg->x;
    refPoint.y = msg->y;
    refPoint.z = msg->z;
    enableDetect = true;
    detected = false;
    initThreshold = false;
    threshold = 0;
}

void callbackStartNearestRecog(const std_msgs::Empty::ConstPtr& msg) {
    std::cout << "HandDetect.->Starting Hand Detect in BB..."
        << std::endl;
    subPointCloud = nh_ptr->subscribe(
            "/hardware/point_cloud_man/rgbd_wrt_robot", 1, callbackPointCloud);
    enaNearestDetect = true;
}

void callbackStopFrontRecog(const std_msgs::Empty::ConstPtr& msg) {
    std::cout << "HandDetect.->Stoping Hand Detect in BB..."
        << std::endl;
    subPointCloud.shutdown();
    cv::destroyAllWindows();
    refPoint.x = 0.0;
    refPoint.y = 0.0;
    refPoint.z = 0.0;
    enableDetect = false;
    detected = false;
    initThreshold = true;
    threshold = 0;
}

void callbackStopNearestRecog(const std_msgs::Empty::ConstPtr& msg) {
    std::cout << "HandDetect.->Stoping Hand Detect in BB..."
        << std::endl;
    subPointCloud.shutdown();
    cv::destroyAllWindows();
    enaNearestDetect = false;
}

int main(int argc, char ** argv) {
    std::cout << "INITIALIZING NODE hand_detect_in_BB"
        << std::endl;
    ros::init(argc, argv, "hand_detect_in_BB_node");

    ros::NodeHandle n;
    nh_ptr = &n;

    ros::Subscriber subStartFrontRecog = n.subscribe("/vision/hand_detect_in_bb/start_hand_front_recog", 1, callbackStartFrontRecog);
    ros::Subscriber subStopFrontRecog = n.subscribe("/vision/hand_detect_in_bb/stop_hand_front_recog", 1, callbackStopFrontRecog);
    ros::Subscriber subStartNearestRecog = n.subscribe("/vision/hand_detect_in_bb/start_hand_nearest_recog", 1, callbackStartNearestRecog);
    ros::Subscriber subStopNearestRecog = n.subscribe("/vision/hand_detect_in_bb/stop_hand_nearest_recog", 1, callbackStopNearestRecog);
    ros::Publisher pubHandInFront = n.advertise<std_msgs::Bool>("/vision/hand_detect_in_bb/hand_in_front", 1);
    pub_nearestDetect = n.advertise< geometry_msgs::Point32 >( "vision/hand_detect_in_BB/hand_nearest_detect", 1);   

    /*subPointCloud = n.subscribe("/hardware/point_cloud_man/rgbd_wrt_robot", 1,
      callbackPointCloud);*/

    ros::Rate rate(15);

    int key = 0;
    while (ros::ok() && key != 27) {

        if(enableDetect){
            std_msgs::Bool msg;
            msg.data = detected;
            pubHandInFront.publish(msg);
        }

        rate.sleep();
        ros::spinOnce();
        key = cvWaitKey(10);
    }

    return true;
}
