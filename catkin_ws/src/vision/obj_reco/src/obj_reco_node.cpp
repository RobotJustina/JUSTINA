#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_conversions/pcl_conversions.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "vision_msgs/VisionObject.h"
#include "vision_msgs/RecognizeObjects.h"
#include "vision_msgs/DetectObjects.h"
#include "vision_msgs/TrainObject.h"
#include "vision_msgs/VisionObjectList.h"
#include "vision_msgs/FindLines.h"
#include "vision_msgs/FindPlane.h"

#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaRepresentation.h"

#include "ObjExtractor.hpp"
#include "DetectedObject.hpp"
#include "ObjRecognizer.hpp"

cv::VideoCapture kinect;
cv::Mat lastImaBGR;
cv::Mat lastImaPCL;

ObjRecognizer objReco;


std::string execMsg = " >> RECO_OBJ_NODE : Executing... "; 
bool debugMode = false;
bool useCVKinect = false;

bool enableDetectWindow = false;
bool enableRecognizeTopic = false;
std::string dirToSaveFiles   = "";
std::string data_base_folder = "";

ros::Publisher pubRecognizedObjects;
ros::Publisher pubRvizMarkers; 

ros::Subscriber subPointCloud;
ros::Subscriber subEnableDetectWindow;
ros::Subscriber subEnableRecognizeTopic;

ros::ServiceServer srvDetectObjs;
ros::ServiceServer srvDetectAllObjs;
ros::ServiceServer srvTrainObject;
ros::ServiceServer srvFindLines;
ros::ServiceServer srvFindPlane;
ros::ServiceServer srvFindFreePlane;

ros::ServiceClient cltRgbdRobot;

void callback_subPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg);
void callback_subEnableDetectWindow(const std_msgs::Bool::ConstPtr& msg);
void callback_subEnableRecognizeTopic(const std_msgs::Bool::ConstPtr& msg);
bool callback_srvDetectObjects(vision_msgs::DetectObjects::Request &req, vision_msgs::DetectObjects::Response &resp);
bool callback_srvDetectAllObjects(vision_msgs::DetectObjects::Request &req, vision_msgs::DetectObjects::Response &resp);
bool callback_srvTrainObject(vision_msgs::TrainObject::Request &req, vision_msgs::TrainObject::Response &resp);
bool callback_srvFindLines(vision_msgs::FindLines::Request &req, vision_msgs::FindLines::Response &resp);
bool callback_srvFindPlane(vision_msgs::FindPlane::Request &req, vision_msgs::FindPlane::Response &resp);
bool callback_srvFindFreePlane(vision_msgs::FindPlane::Request &req, vision_msgs::FindPlane::Response &resp);

bool GetImagesFromJustina( cv::Mat& imaBGR, cv::Mat& imaPCL); 
void GetParams(int argc, char** argv);
void DrawObjects(std::vector< vision_msgs::VisionObject >& objList); 
void DrawObjects(std::vector<DetectedObject> detObjList); 

int main(int argc, char** argv)
{ 
    std::cout << "INITIALIZING OBJECT RECOGNIZER BY MR. YISUS" << std::endl;
    GetParams(argc, argv);

    // Initializing ROS node
    ros::init(argc, argv, "obj_reco_node");
    ros::NodeHandle n;

    //subPointCloud = n.subscribe("/hardware/point_cloud_man/rgbd_wrt_robot", 1, callback_subPointCloud);
    subEnableDetectWindow   = n.subscribe("/vision/obj_reco/enableDetectWindow", 1, callback_subEnableDetectWindow);
    subEnableRecognizeTopic = n.subscribe("/vision/obj_reco/enableRecognizeTopic", 1, callback_subEnableRecognizeTopic);

    pubRecognizedObjects = n.advertise<vision_msgs::VisionObjectList>("/vision/obj_reco/recognizedObjectes",1);
    pubRvizMarkers = n.advertise< visualization_msgs::MarkerArray >("/hri/visualization_marker_array", 10); 

    srvDetectObjs    = n.advertiseService("/vision/obj_reco/det_objs", callback_srvDetectObjects);
    srvDetectAllObjs = n.advertiseService("/vision/obj_reco/det_all_objs", callback_srvDetectAllObjects);
    srvTrainObject   = n.advertiseService("/vision/obj_reco/trainObject", callback_srvTrainObject);

    srvFindLines = n.advertiseService("/vision/line_finder/find_lines_ransac", callback_srvFindLines);
    srvFindPlane = n.advertiseService("/vision/geometry_finder/findPlane", callback_srvFindPlane);
    srvFindFreePlane = n.advertiseService("/vision/geometry_finder/vacantPlane", callback_srvFindFreePlane);

    cltRgbdRobot = n.serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot");

    ros::Rate loop(10);

    // Getting Objects to train
    objReco = ObjRecognizer(18);
    objReco.LoadTrainingDir(data_base_folder);

    JustinaRepresentation::setNodeHandle(&n);

    // Principal loop
    char keyStroke = 0;
    while(ros::ok())
    {
        // ROS
        ros::spinOnce();
        loop.sleep();

        if( cv::waitKey(5) == 'q' )
            break;
    }
    cv::destroyAllWindows();
    return 0;
}

void GetParams(int argc, char** argv)
{
    for( int i=0; i<argc; i++)
    {
        std::string params( argv[i] );

        if( params == "-d" )
        {
            debugMode = true;
            std::cout << "-> DebugMode ON" << std::endl;
        }
        else if( params == "-f" )
        {
            dirToSaveFiles = argv[i+1];
            std::cout << "-> DirToSaveFiles: " << dirToSaveFiles << std::endl;
        }
        else if( params == "--db")
        {
            data_base_folder = argv[++i];
            std::cout << "obj_reco_node.->Training folder: " << data_base_folder << std::endl;
        }
    }
}

bool callback_srvTrainObject(vision_msgs::TrainObject::Request &req, vision_msgs::TrainObject::Response &resp)
{
    if( req.name == "" )
    {
        std::cout << "WARNING !: objects must have a name to be trained" << std::cout;
        return false;
    }

    point_cloud_manager::GetRgbd srv;
    if(!cltRgbdRobot.call(srv))
    {
        std::cout << "ObjDetector.->Cannot get point cloud" << std::endl;
        return false;
    } 
    cv::Mat imaBGR;
    cv::Mat imaPCL;
    JustinaTools::PointCloud2Msg_ToCvMat(srv.response.point_cloud, imaBGR, imaPCL);
    //cv::Mat imaBGR = lastImaBGR.clone();
    //cv::Mat imaPCL = lastImaPCL.clone();

    ObjExtractor::DebugMode = debugMode;
    std::vector<DetectedObject> detObjList = ObjExtractor::GetObjectsInHorizontalPlanes(imaPCL);

    if( detObjList.size() > 0 )
        objReco.TrainObject( detObjList[0], imaBGR, req.name );

    std::cout << "Training Success" << req.name << std::endl;
    return true;
}

void callback_subPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    cv::Mat bgrImage;
    cv::Mat xyzCloud;
    JustinaTools::PointCloud2Msg_ToCvMat(msg, bgrImage, xyzCloud);

    lastImaBGR = bgrImage.clone();
    lastImaPCL = xyzCloud.clone();

    /* //Debug */
    //ObjExtractor::DebugMode = true;
    //ObjExtractor::GetLine( lastImaPCL );

    //cv::imshow( "bgrIma", bgrImage );
    //cv::imshow( "xyzCloud", xyzCloud );

    /*return ; */

    if( enableDetectWindow || enableRecognizeTopic )
    {
        ObjExtractor::DebugMode = debugMode;
        std::vector<DetectedObject> detObjList = ObjExtractor::GetObjectsInHorizontalPlanes(xyzCloud);

        vision_msgs::VisionObjectList objList;
        for( int i=0; i<detObjList.size(); i++)
        {
            if( i == 0 )
                cv::rectangle( bgrImage, detObjList[i].boundBox, cv::Scalar(255,0,0), 2);
            else
                cv::rectangle( bgrImage, detObjList[i].boundBox, cv::Scalar(0,0,255), 2);

            if( enableRecognizeTopic )
            {
                std::string objName = objReco.RecognizeObject( detObjList[i], lastImaBGR );
                vision_msgs::VisionObject obj;

                obj.id = objName;
                obj.pose.position.x = detObjList[i].centroid.x;
                obj.pose.position.y = detObjList[i].centroid.y;
                obj.pose.position.z = detObjList[i].centroid.z;

                objList.ObjectList.push_back(obj);
            }
        }

        if( enableRecognizeTopic )
            pubRecognizedObjects.publish( objList );

        if( enableDetectWindow )
            cv::imshow("Detected Objects", bgrImage);
    }
}

bool callback_srvDetectObjects(vision_msgs::DetectObjects::Request &req, vision_msgs::DetectObjects::Response &resp)
{
    std::cout << execMsg  << "srvDetectObjects" << std::endl;  

    cv::Mat imaBGR;
    cv::Mat imaPCL;
    if( !GetImagesFromJustina( imaBGR, imaPCL) )
        return false; 

    ObjExtractor::DebugMode = debugMode;
    std::vector<DetectedObject> detObjList = ObjExtractor::GetObjectsInHorizontalPlanes(imaPCL);
    DrawObjects( detObjList ); 

    cv::Mat imaToShow = imaBGR.clone();
    for( int i=0; i<detObjList.size(); i++)
    {
        std::string objName = objReco.RecognizeObject( detObjList[i], imaBGR );
        std::string objTag;

        vision_msgs::VisionObject obj;

        if(objName.compare("") != 0){     
            std::stringstream ss;
            std::string result;
            bool querySuccess = JustinaRepresentation::selectCategoryObjectByName(objName, result, 0);
            ss << objName;
            if(querySuccess)
            {
                ss << "_" << result;
                std::cout << "ObjDetector.->The object name with category:" << ss.str() << std::endl;
                obj.category = result;
            }
            objTag = ss.str();
        }
        cv::rectangle(imaToShow, detObjList[i].boundBox, cv::Scalar(0,0,255) );
        cv::putText(imaToShow, objTag, detObjList[i].boundBox.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,255) );

        if( objName == "" )
            continue;

        if( dirToSaveFiles != "" && req.saveFiles)
        {
            cv::Mat imaToSave = imaBGR.clone();
            cv::rectangle(imaToSave, detObjList[i].boundBox, cv::Scalar(0,0,255) );
            cv::putText(imaToSave, objTag, detObjList[i].boundBox.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,255) );
            cv::imwrite( dirToSaveFiles + objName + ".png", imaToSave);
        }


        obj.id = objName;
        obj.pose.position.x = detObjList[i].centroid.x;
        obj.pose.position.y = detObjList[i].centroid.y;
        obj.pose.position.z = detObjList[i].centroid.z;

        resp.recog_objects.push_back(obj);
    }

    //Code bubble_sort by euclidean distance for objects
    /*
    // Code for printing the list of objects
    std::cout << "objs_detect after Sorting...." << std::endl;
    for(int i = 0; i < resp.recog_objects.size(); i++)
    {
    std::cout << "obj_" << i << ":  " << resp.recog_objects[i].id << std::endl;
    std::cout << "pose: " << resp.recog_objects[i].pose.position << std::endl;
    }
     */
    for(int i=1; i < resp.recog_objects.size(); i++)
    {
        for(int j=0; j < resp.recog_objects.size() - i; j++)
        {
            float euclideanDist [] = {0.0, 0.0};
            float objx[] = {resp.recog_objects[j].pose.position.x, resp.recog_objects[j+1].pose.position.x};
            float objy[] = {resp.recog_objects[j].pose.position.y, resp.recog_objects[j+1].pose.position.y};
            float objz[] = {resp.recog_objects[j].pose.position.z, resp.recog_objects[j+1].pose.position.z};

            euclideanDist[0] = sqrt(objx[0]*objx[0] + objy[0]*objy[0] + objz[0]*objz[0]);
            euclideanDist[1] = sqrt(objx[1]*objx[1] + objy[1]*objy[1] + objz[1]*objz[1]);

            //if(resp.recog_objects[j].pose.position.x > resp.recog_objects[j+1].pose.position.x)
            if(euclideanDist[0] > euclideanDist[1])
            {
                vision_msgs::VisionObject aux;
                aux = resp.recog_objects[j];
                resp.recog_objects[j] = resp.recog_objects[j+1];
                resp.recog_objects[j+1] = aux;
            }
        }
    }

    /*
       std::cout << "objs_detect before Sorting...." << std::endl;
       for(int i = 0; i < resp.recog_objects.size(); i++)
       {
       std::cout << "obj_" << i << ":  " << resp.recog_objects[i].id << std::endl;
       std::cout << "pose: " << resp.recog_objects[i].pose.position << std::endl;
       }
     */
    cv::imshow( "Recognized Objects", imaToShow );
    return true;
}

bool callback_srvDetectAllObjects(vision_msgs::DetectObjects::Request &req, vision_msgs::DetectObjects::Response &resp)
{
    std::cout << execMsg  << "srvDetectiALLObjects" << std::endl;  

    cv::Mat imaBGR;
    cv::Mat imaPCL;
    if( !GetImagesFromJustina( imaBGR, imaPCL) )
        return false; 

    ObjExtractor::DebugMode = debugMode;
    std::vector<DetectedObject> detObjList = ObjExtractor::GetObjectsInHorizontalPlanes(imaPCL);

    cv::Mat imaToShow = imaBGR.clone();
    int indexObjUnknown = 0;
    for( int i=0; i<detObjList.size(); i++)
    {
        vision_msgs::VisionObject obj;
        std::string objTag;
        std::string objName = objReco.RecognizeObject( detObjList[i], imaBGR );

        if(objName.compare("") != 0){     
            std::stringstream ss;
            std::string result;
            bool querySuccess = JustinaRepresentation::selectCategoryObjectByName(objName, result, 0);
            ss << objName;
            if(querySuccess)
            {
                ss << "_" << result;
                std::cout << "ObjDetector.->The object name with category:" << ss.str() << std::endl;
                obj.category = result;
            }
            objTag = ss.str();
        }

        if( objName == "" ){
            std::stringstream ss;
            ss << "unknown" << indexObjUnknown++;
            objName = ss.str();
        }

        cv::rectangle(imaToShow, detObjList[i].boundBox, cv::Scalar(0,0,255) );
        cv::putText(imaToShow, objTag, detObjList[i].boundBox.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,255) );

        if( dirToSaveFiles != "" && req.saveFiles)
        {
            std::stringstream ss;
            ss << dirToSaveFiles << objName << ".png";
            std::cout << "JustinaVision.->save file object name:" << ss.str() << std::endl;
            cv::Mat imaToSave = imaBGR.clone();
            cv::rectangle(imaToSave, detObjList[i].boundBox, cv::Scalar(0,0,255) );
            cv::putText(imaToSave, objName, detObjList[i].boundBox.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,255) );
            cv::imwrite( ss.str(), imaToSave);
        }

        obj.id = objName;
        obj.pose.position.x = detObjList[i].centroid.x;
        obj.pose.position.y = detObjList[i].centroid.y;
        obj.pose.position.z = detObjList[i].centroid.z;

        resp.recog_objects.push_back(obj);
    }

    //Code bubble_sort by euclidean distance for objects
    /*
    // Code for printing the list of objects
    std::cout << "objs_detect after Sorting...." << std::endl;
    for(int i = 0; i < resp.recog_objects.size(); i++)
    {
    std::cout << "obj_" << i << ":  " << resp.recog_objects[i].id << std::endl;
    std::cout << "pose: " << resp.recog_objects[i].pose.position << std::endl;
    }
     */
    for(int i=1; i < resp.recog_objects.size(); i++)
    {
        for(int j=0; j < resp.recog_objects.size() - i; j++)
        {
            float euclideanDist [] = {0.0, 0.0};
            float objx[] = {resp.recog_objects[j].pose.position.x, resp.recog_objects[j+1].pose.position.x};
            float objy[] = {resp.recog_objects[j].pose.position.y, resp.recog_objects[j+1].pose.position.y};
            float objz[] = {resp.recog_objects[j].pose.position.z, resp.recog_objects[j+1].pose.position.z};

            euclideanDist[0] = sqrt(objx[0]*objx[0] + objy[0]*objy[0] + objz[0]*objz[0]);
            euclideanDist[1] = sqrt(objx[1]*objx[1] + objy[1]*objy[1] + objz[1]*objz[1]);

            //if(resp.recog_objects[j].pose.position.x > resp.recog_objects[j+1].pose.position.x)
            if(euclideanDist[0] > euclideanDist[1])
            {
                vision_msgs::VisionObject aux;
                aux = resp.recog_objects[j];
                resp.recog_objects[j] = resp.recog_objects[j+1];
                resp.recog_objects[j+1] = aux;
            }
        }
    }

    /*
/       std::cout << "objs_detect before Sorting...." << std::endl;
       for(int i = 0; i < resp.recog_objects.size(); i++)
       {
       std::cout << "obj_" << i << ":  " << resp.recog_objects[i].id << std::endl;
       std::cout << "pose: " << resp.recog_objects[i].pose.position << std::endl;
       }
     */
    cv::imshow( "Recognized Objects", imaToShow );
    return true;
}

bool callback_srvRecognizeObjects(vision_msgs::RecognizeObjects::Request &req, vision_msgs::RecognizeObjects::Response &resp)
{ 
    std::cout << " >>> WARNING !!! Service not implemented, use det_objs instead" << std::cout;
    return false;

    boost::shared_ptr<sensor_msgs::PointCloud2 const> msg;
    msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/hardware/point_cloud_man/rgbd_wrt_robot", ros::Duration(1.0) ) ;
    if( msg == NULL )
    {
        std::cout << "det_objs TIMEOUT" << std::endl;
        return false;
    }

    sensor_msgs::PointCloud2 pc2 = * msg;

    //std::vector<std::pair< double, std::string> > recog_objects;
    //cv::Mat bgrImage;
    //cv::Mat pointCloud;

    //Transform from PointCloud2 (ros msg) to cv::Mat format

    vision_msgs::VisionObject obj1;
    obj1.id= "Milk";
    resp.recog_objects.push_back(obj1);

    vision_msgs::VisionObject obj2;
    obj2.id= "Frutastica";
    resp.recog_objects.push_back(obj2);

    //std::vector< DetectedObject > detObj = objExt.ExtractObjectsHorizantalPlanes(bgrImage, pointCloud, detectedObj);
    std::cout << "HW" << std::endl;
    return true;
}

void callback_subEnableDetectWindow(const std_msgs::Bool::ConstPtr& msg)
{
    enableDetectWindow = msg->data;

    if( !enableDetectWindow )
        cv::destroyAllWindows();
}

void callback_subEnableRecognizeTopic(const std_msgs::Bool::ConstPtr& msg)
{
    enableRecognizeTopic = msg->data;
}

bool callback_srvFindLines(vision_msgs::FindLines::Request &req, vision_msgs::FindLines::Response &resp)
{
    std::cout << "EXECUTING srvFindLines (Yisus Version)" << std::endl;
    point_cloud_manager::GetRgbd srv;
    if(!cltRgbdRobot.call(srv))
    {
        std::cout << "ObjDetector.->Cannot get point cloud" << std::endl;
        return false;
    }
    cv::Mat bgrImg;
    cv::Mat xyzCloud;
    JustinaTools::PointCloud2Msg_ToCvMat(srv.response.point_cloud, bgrImg, xyzCloud);
    //cv::Mat bgrImg = lastImaBGR.clone();
    //cv::Mat xyzCloud = lastImaPCL.clone();

    ObjExtractor::DebugMode = debugMode;
    cv::Vec4i pointsLine = ObjExtractor::GetLine( xyzCloud );
    if( pointsLine == cv::Vec4i(0,0,0,0) )
    {
        std::cout << "Line not Detected" << std::endl;
        return false;
    }

    cv::Point3f iniLine = xyzCloud.at<cv::Vec3f>( cv::Point(pointsLine[0], pointsLine[1]) );
    cv::Point3f endLine = xyzCloud.at<cv::Vec3f>( cv::Point(pointsLine[2], pointsLine[3]) );

    geometry_msgs::Point p1;
    p1.x = iniLine.x;
    p1.y = iniLine.y;
    p1.z = iniLine.z;

    geometry_msgs::Point p2;
    p2.x = endLine.x;
    p2.y = endLine.y;
    p2.z = endLine.z;

    resp.lines.push_back(p1);
    resp.lines.push_back(p2);

    cv::line(bgrImg, cv::Point(pointsLine[0], pointsLine[1]), cv::Point(pointsLine[2], pointsLine[3]), cv::Scalar(0, 255, 0), 3, 8 );
    cv::imshow("Find Line", bgrImg );

    std::cout << "Line found:" << std::endl;
    std::cout << "  p1=" << iniLine << std::endl;
    std::cout << "  p2=" << endLine << std::endl;

    return true;
}

bool callback_srvFindPlane(vision_msgs::FindPlane::Request &req, vision_msgs::FindPlane::Response &resp)
{
    std::cout << "EXECUTING srvFindPlane " << std::endl;

    //cv::Mat imaBGR = lastImaBGR.clone();
    //cv::Mat imaPCL = lastImaPCL.clone();
    point_cloud_manager::GetRgbd srv;
    if(!cltRgbdRobot.call(srv))
    {
        std::cout << "ObjDetector.->Cannot get point cloud" << std::endl;
        return false;
    }
    cv::Mat imaBGR;
    cv::Mat imaPCL;
    JustinaTools::PointCloud2Msg_ToCvMat(srv.response.point_cloud, imaBGR, imaPCL);

    std::vector<PlanarSegment>  horizontalPlanes = ObjExtractor::GetHorizontalPlanes(imaPCL);

    if( horizontalPlanes.size() < 1 )
    {
        std::cout << "Planes not Detected" << std::endl;
        return false;
    }

    for( int i=0; i<(int)horizontalPlanes.size(); i++)
    {
        std::vector< cv::Point2i > indexes = horizontalPlanes[i].Get_Indexes();
        cv::Vec3b color = cv::Vec3b( rand()%255, rand()%255, rand()%255 );
        for( int j=0; j<(int)indexes.size(); j++)
        {
            imaBGR.at< cv::Vec3b >( indexes[j] ) = color;
        }
    }

    std::cout << "Planes detected !!" << std::endl;
    cv::imshow("FindPlane", imaBGR);
    cv::waitKey(10);
    return true;
}

bool callback_srvFindFreePlane(vision_msgs::FindPlane::Request &req, vision_msgs::FindPlane::Response &resp)
{
    std::cout << "EXECUTING srv Find Free Plane " << std::endl;

    int inliers;
    int minInliers;
    float x_min;
    float z_plane;
    float y_rnd;

    float x_minBox;
    float x_maxBox;
    float y_minBox;
    float y_maxBox;
    float z_minBox;
    float z_maxBox;

    float w_box;
    float h_box;


    cv::Mat imaBGR;
    cv::Mat imaPCL;
    cv::Point3f p;

    point_cloud_manager::GetRgbd srv;

    inliers = 0;
    x_min = 100.0;
    z_plane = 0.0;

    minInliers = 3000;
    h_box = 0.04;
    w_box = 0.27;

    if(!cltRgbdRobot.call(srv))
    {
        std::cout << "ObjDetector.->Cannot get point cloud" << std::endl;
        return false;
    }
    JustinaTools::PointCloud2Msg_ToCvMat(srv.response.point_cloud, imaBGR, imaPCL);

    std::vector<PlanarSegment>  horizontalPlanes = ObjExtractor::GetHorizontalPlanes(imaPCL);

    if( horizontalPlanes.size() < 1 )
    {
        std::cout << "Planes not Detected" << std::endl;
        return false;
    }

    for( int i=0; i<(int)horizontalPlanes.size(); i++)
    {
        std::vector< cv::Point2i > indexes = horizontalPlanes[i].Get_Indexes();
        //Get z_prom of eache plane
        for( int j=0; j<(int)indexes.size(); j++)
        {
            p = imaPCL.at< cv::Point3f >( indexes[j] );
            if(p.x < x_min && p.x > 0.15)
                x_min = p.x;
            z_plane += p.z;
        }
        z_plane /= (int)indexes.size();
        //std::cout << "z_plane[" << i << "]:  " << z_plane << std::endl;
        //std::cout << "x_min[" << i << "]:  " << x_min << std::endl;

        x_minBox = x_min + 0.10;
        x_maxBox = x_minBox + h_box;
        z_minBox = z_plane - 0.03;
        z_maxBox = z_plane + 0.03;

        //Try to find free place on plane
        for (float att = 0; att < 41; att++)
        {
            inliers = 0;
            y_rnd = (-0.02*att) + 0.4;
            y_minBox = y_rnd - (w_box/2);
            y_maxBox = y_rnd + (w_box/2);

            for( int j=0; j<(int)indexes.size(); j++)
            {
                p = imaPCL.at< cv::Point3f >( indexes[j] );
                if(p.x > x_minBox && p.x < x_maxBox &&
                        p.y > y_minBox && p.y < y_maxBox &&
                        p.z > z_minBox && p.z < z_maxBox)
                {
                    inliers++;
                }
            }

            //std::cout << "inliers: " << inliers << std::endl;
            //std::cout << "" << std::endl;

            if (inliers > minInliers)
            {
                geometry_msgs::Point p1;
                std_msgs::Int32 bestInliers;
                bestInliers.data = inliers;
                p1.x = (x_min+x_maxBox)/2 ;
                p1.y = y_rnd;
                p1.z = z_plane + 0.10;
                cv::Vec3b color = cv::Vec3b( rand()%255, rand()%255, rand()%255 );
                std::cout << "Find_freePlane.-> free_spacePlane:  [" << p1.x << ", " << p1.y << ", " << p1.z << "]" << std::endl;
                for( int j=0; j<(int)indexes.size(); j++)
                {
                    p = imaPCL.at< cv::Point3f >( indexes[j] );
                    if(p.x > x_minBox && p.x < x_maxBox &&
                            p.y > y_minBox && p.y < y_maxBox &&
                            p.z > z_minBox && p.z < z_maxBox)
                    {
                        imaBGR.at< cv::Vec3b >( indexes[j] ) = color;
                    }
                }
                resp.centroidFreeSpace.push_back(p1);
                resp.inliers.push_back(bestInliers);
            }

        }

        if(resp.centroidFreeSpace.size() > 0)
        {
            std::cout << "Planes detected:  " << resp.centroidFreeSpace.size() << std::endl;
        }
        else
        {
            std::cout << "I can´t find free space on plane:  " <<  std::endl;
            return false;
        }

    }
    cv::imshow("FindPlane", imaBGR);
    cv::waitKey(10);
    return true;
}

bool GetImagesFromJustina( cv::Mat& imaBGR, cv::Mat& imaPCL)
{
    point_cloud_manager::GetRgbd srv;
    if(!cltRgbdRobot.call(srv))
    {
        std::cout << "ObjDetector.->Cannot get point cloud" << std::endl;
        return false;
    }
    JustinaTools::PointCloud2Msg_ToCvMat(srv.response.point_cloud, imaBGR, imaPCL);
    return true; 
}

void DrawObjects(std::vector<DetectedObject> detObjList)
{
    std::cout << "Dsdasdasd RAAAAAAAAAAAAAAAAWING" << std::endl; 

    std::string robotFrameID = "/base_link";
    std::string markersNS = "objects_Markers";   
    
    float markCentroidScale = .0353535;
    float duration = 60;

    visualization_msgs::MarkerArray markersList;     
    for(size_t i=0; i<detObjList.size(); i++)
    {
        visualization_msgs::Marker markCentroid;
        markCentroid.header.frame_id = robotFrameID; 
        markCentroid.header.stamp = ros::Time::now(); 
        markCentroid.ns = markersNS; 
        markCentroid.id = i;   
        markCentroid.type = visualization_msgs::Marker::SPHERE; 
        markCentroid.action = visualization_msgs::Marker::ADD;
        markCentroid.pose.position.x = detObjList[i].centroid.x;
        markCentroid.pose.position.y = detObjList[i].centroid.y;
        markCentroid.pose.position.z = detObjList[i].centroid.z;
        markCentroid.scale.x = markCentroidScale;
        markCentroid.scale.y = markCentroidScale;
        markCentroid.scale.z = markCentroidScale;
        markCentroid.color.r = 1.0; 
        markCentroid.color.g = 0.0; 
        markCentroid.color.b = 0.0;
        markCentroid.color.a = 1.0; 
        markCentroid.lifetime = ros::Duration(duration, duration); 

        markersList.markers.push_back( markCentroid );         
        std::cout << "Drawing obj " << i  << std::endl; 
    }
    pubRvizMarkers.publish( markersList ); 
}
