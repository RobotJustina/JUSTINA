#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/tracking.hpp>

#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"

#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "vision_msgs/VisionObject.h"
#include "vision_msgs/RecognizeObjects.h"
#include "vision_msgs/DetectObjects.h"
#include "vision_msgs/TrainObject.h"
#include "vision_msgs/VisionObjectList.h"
#include "vision_msgs/FindLines.h"
#include "vision_msgs/FindPlane.h"
#include "vision_msgs/DetectGripper.h"

#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaRepresentation.h"

#include "ObjExtractor.hpp"
#include "DetectedObject.hpp"
#include "ObjRecognizer.hpp"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

#include "vision_msgs/image_srv.h"

// Includes for YOLO object recog
#include "vision_msgs/CheckForObjectsAction.h"
#include "actionlib/client/simple_action_client.h"

cv::VideoCapture kinect;
cv::Mat lastImaBGR;
cv::Mat lastImaPCL;
bool recivedResponse;

ObjRecognizer objReco;

std::string execMsg = " >> RECO_OBJ_NODE : Executing... "; 
bool debugMode = false;
bool useCVKinect = false;

bool enaDetectByHeigth = false; 
bool enaDetectByPlane = false; 
bool enableDetectWindow = false;
bool enableRecognizeTopic = false;
bool enableGripperPose = false;
bool enableDetectObjectYOLO = false;
bool showImgYOLO = false;

std::string dirToSaveFiles   = "";
std::string data_base_folder = "";

ros::NodeHandle* node; 

ros::Publisher pubRecognizedObjects;
ros::Publisher pubRvizMarkers; 
ros::Publisher pubGripperPose;

ros::Subscriber subEnableDetectWindow;
ros::Subscriber subEnableRecognizeTopic;
ros::Subscriber sub_enaDetectByPlane; 
ros::Subscriber sub_enaDetectByHeight;
ros::Subscriber sub_pointCloudRobot;
ros::Subscriber subTrainGripper;
ros::Subscriber subStartGripperPose;

ros::ServiceServer srvDetectObjs;
ros::ServiceServer srvDetectAllObjs;
ros::ServiceServer srvTrainObject;
ros::ServiceServer srvFindLines;
ros::ServiceServer srvFindPlane;
ros::ServiceServer srvFindTable;
ros::ServiceServer srvFindFreePlane;
ros::ServiceServer srv_trainByHeight;
ros::ServiceServer srvDetectGripper;
ros::ServiceServer srvExtractObjectAbovePlanes;
ros::ServiceServer srvEXtractObjectWithPlanes;

//YOLO object recog
ros::ServiceServer srvDetectObjsYOLO;
ros::Subscriber subEnableDetectObjsYOLO;
ros::Subscriber subDetectObjsYOLO;
ros::Publisher pubImgYOLO;
ros::Publisher pubDetectObjsYOLO;
ros::Subscriber subImgDetectObjsYOLO;
//Action client for YOLO object recog
actionlib::SimpleActionClient<vision_msgs::CheckForObjectsAction> * actCltForObjects;

//test
ros::ServiceServer srvVotObjs;
ros::ServiceServer srvTFObjects;
ros::ServiceClient cltTFObjects;

ros::ServiceClient cltRgbdRobot;
            
void callback_subEnableDetectWindow(const std_msgs::Bool::ConstPtr& msg);
void callback_subEnableRecognizeTopic(const std_msgs::Bool::ConstPtr& msg);
void callback_subStartGripperPosition(const std_msgs::Bool::ConstPtr& msg);
bool callback_srvDetectObjects(vision_msgs::DetectObjects::Request &req, vision_msgs::DetectObjects::Response &resp);
bool callback_srvDetectAllObjects(vision_msgs::DetectObjects::Request &req, vision_msgs::DetectObjects::Response &resp);
bool callback_srvDetectGripper(vision_msgs::DetectGripper::Request &req, vision_msgs::DetectGripper::Response &resp);
bool callback_srvTrainObject(vision_msgs::TrainObject::Request &req, vision_msgs::TrainObject::Response &resp);
bool callback_srvFindLines(vision_msgs::FindLines::Request &req, vision_msgs::FindLines::Response &resp);
bool callback_srvFindPlane(vision_msgs::FindPlane::Request &req, vision_msgs::FindPlane::Response &resp);
bool callback_srvFindTable(vision_msgs::FindPlane::Request &req, vision_msgs::FindPlane::Response &resp);
bool callback_srvFindFreePlane(vision_msgs::FindPlane::Request &req, vision_msgs::FindPlane::Response &resp);

//test
bool callback_srvVotationObjects(vision_msgs::DetectObjects::Request &req, vision_msgs::DetectObjects::Response &resp);
bool callback_srvTFObjectDetect(vision_msgs::DetectObjects::Request &req, vision_msgs::DetectObjects::Response &resp);
bool callback_srvExtractObjectsAbovePlanes(vision_msgs::DetectObjects::Request &req, vision_msgs::DetectObjects::Response &resp);
bool callback_srvExtractObjectsWithPlanes(vision_msgs::DetectObjects::Request &req, vision_msgs::DetectObjects::Response &resp);

bool cb_srvTrainByHeigth(vision_msgs::TrainObject::Request &req, vision_msgs::TrainObject::Response &resp);
void call_pointCloudRobot(const sensor_msgs::PointCloud2::ConstPtr& msg);

//YOLO object recog
void callback_subEnableDetectObjsYOLO(const std_msgs::Bool::ConstPtr& msg);
void callback_subDetectObjsYOLO(const vision_msgs::BoundingBoxes::ConstPtr& msg);
void callback_subImgDetectObjsYOLO(const sensor_msgs::Image::ConstPtr &msg);
bool callback_srvDetectObjectsYOLO(vision_msgs::DetectObjects::Request &req, vision_msgs::DetectObjects::Response &resp);

bool GetImagesFromJustina( cv::Mat& imaBGR, cv::Mat& imaPCL); 
void GetParams(int argc, char** argv);
void DrawObjects(std::vector< vision_msgs::VisionObject >& objList); 
void DrawObjects(std::vector<DetectedObject> detObjList); 

void cb_sub_pointCloudRobot(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    std::string winName = ""; 

    cv::Mat imaRGB; 
    cv::Mat imaXYZ; 
    if( !GetImagesFromJustina(imaRGB, imaXYZ) )
        return;   

    winName = "Detect by Heigth"; 
    if( enaDetectByHeigth )
    {
        DetectedObject dObj =  ObjExtractor::GetObjectInBox( imaRGB, imaXYZ );   
        cv::Mat imaToShow; 
        if( dObj.image.data != 0 )
            imaToShow = dObj.GetImageWithMask();    
        else
            imaToShow =  cv::Mat::zeros(100,100,CV_8UC1);
        cv::imshow(winName, imaToShow);  
    }
    else
    {
       try{ cv::destroyWindow(winName); }catch(...){}
    }

    winName = "Detect by Plane";
    if( enaDetectByPlane )
    {
        std::vector<DetectedObject> detObjList = ObjExtractor::GetObjectsInHorizontalPlanes( imaXYZ ); 
        std::sort(detObjList.begin(), detObjList.end(), DetectedObject::CompareByEuclidean ); 
        
        cv::Mat imaToShow = imaRGB.clone(); 
        if( detObjList.size() > 0 )
        {
            for( size_t i=0; i<detObjList.size(); i++)
                cv::rectangle(imaToShow, detObjList[i].boundBox, cv::Scalar(255,0,0) );

            cv::rectangle(imaToShow, detObjList[0].boundBox, cv::Scalar(0,255,0) );
        }
        cv::imshow(winName, imaToShow); 
    }
    else
    {
       try{ cv::destroyWindow(winName); }catch(...){}
    } 

    if( !(enaDetectByHeigth || enaDetectByPlane) )
        sub_pointCloudRobot.shutdown(); 
}     

void cb_sub_enaDetectByHeigth(const std_msgs::Bool::ConstPtr& msg)
{
    if( enaDetectByHeigth = msg->data )
        sub_pointCloudRobot = node -> subscribe("/hardware/point_cloud_man/rgbd_wrt_robot", 1, cb_sub_pointCloudRobot);         
}

void cb_sub_enaDetectByPlane(const std_msgs::Bool::ConstPtr& msg)
{
    if( enaDetectByPlane = msg->data )
        sub_pointCloudRobot = node -> subscribe("/hardware/point_cloud_man/rgbd_wrt_robot", 1, cb_sub_pointCloudRobot);         
}

bool cb_srvTrainByHeigth(vision_msgs::TrainObject::Request &req, vision_msgs::TrainObject::Response &resp)
{ 
    if( req.name == "" )
    {
        std::cout << "WARNING !: objects must have a name to be trained" << std::cout;
        return false;
    }

    cv::Mat imaRGB; 
    cv::Mat imaXYZ; 
    if( !GetImagesFromJustina(imaRGB, imaXYZ) )
        return false;   

    DetectedObject dObj =  ObjExtractor::GetObjectInBox( imaRGB, imaXYZ );  
    if( dObj.image.data != 0 )
        cv::imshow( "imaData", dObj.image ) ;  

    objReco.TrainObject( dObj, imaRGB, req.name);  

    return true; 
 }


void cb_sub_trainGripper(const std_msgs::Empty::ConstPtr msg)
{
    cv::Mat imaBGR, imaPCL;
    std::cout << execMsg << "Train gripper" << std::endl;
    if (!GetImagesFromJustina(imaBGR,imaPCL)){
        std::cout <<"Error GetImagesFromJustina" << std::endl;
        return;
    }
    if(ObjExtractor::TrainGripper(imaBGR))
        std::cout <<"Train successfully" << std::endl;

}



// MAIN
int main(int argc, char** argv)
{   
    std::cout << "INITIALIZING OBJECT RECOGNIZER BY MR. YISUS" << std::endl;
    
    GetParams(argc, argv);

    // Initializing ROS node
    ros::init(argc, argv, "obj_reco_node");
    ros::NodeHandle n;
    node = &n; 

    subEnableDetectWindow   = n.subscribe("/vision/obj_reco/enableDetectWindow",1       , callback_subEnableDetectWindow);
    subEnableRecognizeTopic = n.subscribe("/vision/obj_reco/enableRecognizeTopic",1     , callback_subEnableRecognizeTopic);
    sub_enaDetectByHeight   = n.subscribe("/vision/obj_reco/enable_detect_byHeigth",1   , cb_sub_enaDetectByHeigth);
    sub_enaDetectByPlane    = n.subscribe("/vision/obj_reco/enable_detect_byPlane",1    , cb_sub_enaDetectByPlane); 
    subTrainGripper         = n.subscribe("/vision/obj_reco/train_gripper",1            , cb_sub_trainGripper);
    subStartGripperPose     = n.subscribe("/vision/obj_reco/start_gripper_position", 1, callback_subStartGripperPosition);

    pubRecognizedObjects    = n.advertise<vision_msgs::VisionObjectList>("/vision/obj_reco/recognizedObjectes",1);
    pubRvizMarkers          = n.advertise< visualization_msgs::MarkerArray >("/hri/visualization_marker_array", 10); 
    pubGripperPose           = n.advertise<geometry_msgs::Point>("/vision/obj_reco/gripper_position", 1);

    srvDetectObjs           = n.advertiseService("/vision/obj_reco/det_objs"        , callback_srvDetectObjects);
    srvDetectAllObjs        = n.advertiseService("/vision/obj_reco/det_all_objs"    , callback_srvDetectAllObjects);
    srvTrainObject          = n.advertiseService("/vision/obj_reco/trainObject"     , callback_srvTrainObject);
    srv_trainByHeight       = n.advertiseService("/vision/obj_reco/train_byHeight"  , cb_srvTrainByHeigth);
    srvDetectGripper        = n.advertiseService("/vision/obj_reco/gripper"         , callback_srvDetectGripper);
    srvExtractObjectAbovePlanes        = n.advertiseService("/vision/obj_reco/ext_objects_above_planes"      , callback_srvExtractObjectsAbovePlanes);
    srvEXtractObjectWithPlanes = n.advertiseService("/vision/obj_reco/ext_objects_with_planes", callback_srvExtractObjectsWithPlanes);

    srvFindLines            = n.advertiseService("/vision/line_finder/find_lines_ransac"    , callback_srvFindLines);
    srvFindPlane            = n.advertiseService("/vision/geometry_finder/findPlane"        , callback_srvFindPlane);
    srvFindTable            = n.advertiseService("/vision/geometry_finder/findTable"        , callback_srvFindTable);
    srvFindFreePlane        = n.advertiseService("/vision/geometry_finder/vacantPlane"      , callback_srvFindFreePlane);

    //test
    srvVotObjs		    = n.advertiseService("/vision/obj_reco/vot_objs"		    , callback_srvVotationObjects);
    srvTFObjects = n.advertiseService("/vision/obj_reco/tf_object", callback_srvTFObjectDetect);
    cltTFObjects = n.serviceClient<vision_msgs::image_srv>("/tensor_flow/image"); 


    cltRgbdRobot = n.serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot");
    
    //YOLO object recog
    subEnableDetectObjsYOLO = n.subscribe("/vision/obj_reco/enable_det_objs_YOLO", 1, callback_subEnableDetectObjsYOLO);
    subDetectObjsYOLO       = n.subscribe("/vision/darknet_ros/bounding_boxes", 1, callback_subDetectObjsYOLO);
    subImgDetectObjsYOLO    = n.subscribe("/vision/darknet_ros/detection_image", 1, callback_subImgDetectObjsYOLO);
    srvDetectObjsYOLO       = n.advertiseService("/vision/obj_reco/det_objs_YOLO", callback_srvDetectObjectsYOLO);
    pubImgYOLO              = n.advertise<sensor_msgs::Image>("/usb_cam/image_raw", 1);
    pubDetectObjsYOLO       = n.advertise<vision_msgs::VisionObjectList>("/vision/obj_reco/get_det_objs_YOLO", 1);
    //Action client for YOLO object recog
    actCltForObjects = new actionlib::SimpleActionClient<vision_msgs::CheckForObjectsAction>("/vision/darknet_ros/check_for_objects", true);

    ros::Rate loop(30);

    // Getting Objects to train
    objReco = ObjRecognizer(18);
    if( data_base_folder == "" )
        data_base_folder = ros::package::getPath("obj_reco") + std::string("/TrainingDir");  

    objReco.TrainingDir = data_base_folder; 
    objReco.LoadTrainingDir();
    ObjExtractor::LoadValueGripper();  
    JustinaRepresentation::setNodeHandle(&n); 


    // Principal loop
    char keyStroke = 0;
    while(ros::ok())
    {
        if(enableGripperPose){
            //std::cout << "Calculating the gripper position." << std::endl;
            cv::Mat imaBGR;
            cv::Mat imaPCL;
            //while(cv::waitKey(1)!='q'){
            if (!GetImagesFromJustina(imaBGR,imaPCL)){
                //std::cout << "Can not get images from Justina." << std::endl;
            }
            else{
                cv::Vec3f centroid = ObjExtractor::GetGrippers(imaBGR,imaPCL);
                geometry_msgs::Point msg_point;
                msg_point.x = centroid[0];
                msg_point.y = centroid[1];
                msg_point.z = centroid[2];
                pubGripperPose.publish(msg_point);
            }
        }

        if(enableDetectObjectYOLO && !recivedResponse){
            //std::cout << "Calculating the gripper position." << std::endl;
            //while(cv::waitKey(1)!='q'){
            if (!GetImagesFromJustina(lastImaBGR,lastImaPCL)){
                std::cout << "Can not get images from Justina." << std::endl;
            }
            else{
                recivedResponse = true;
                sensor_msgs::Image container;
                cv_bridge::CvImage cvi_mat;
                cvi_mat.encoding = sensor_msgs::image_encodings::BGR8;
                cvi_mat.image = lastImaBGR;
                cvi_mat.toImageMsg(container);
                pubImgYOLO.publish(container);
            }
        }

        // ROS
        ros::spinOnce();
        loop.sleep();

        if( cv::waitKey(1) == 'q' )
            break;
    }
    cv::destroyAllWindows();
    delete actCltForObjects;
    return 1;
} 

bool callback_srvDetectGripper(vision_msgs::DetectGripper::Request &req, vision_msgs::DetectGripper::Response &resp)
{
    std::cout << execMsg << "srvDetectGripper" << std::endl;
    cv::Mat imaBGR;
    cv::Mat imaPCL;
    //while(cv::waitKey(1)!='q'){
    if (!GetImagesFromJustina(imaBGR,imaPCL))
        return false;
    
    cv::Vec3f centroid = ObjExtractor::GetGrippers(imaBGR,imaPCL);
    if (centroid == cv::Vec3f(0.0,0.0,0.0) )
        return false;

    resp.gripper_position.x  = centroid[0];
    resp.gripper_position.y  = centroid[1];
    resp.gripper_position.z  = centroid[2];
    //}
    return true;
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
    std::sort(detObjList.begin(), detObjList.end(), DetectedObject::CompareByEuclidean ); 

    if( detObjList.size() > 0 )
        objReco.TrainObject( detObjList[0], imaBGR, req.name );

    std::cout << "Training Success" << req.name << std::endl;
    return true;
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
            objTag = ss.str();
        }

        cv::rectangle(imaToShow, detObjList[i].boundBox, cv::Scalar(0,0,255) );
        cv::putText(imaToShow, objTag, detObjList[i].boundBox.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,255) );

        if( dirToSaveFiles != "" && req.saveFiles)
        {
            std::stringstream ss;
            ss << dirToSaveFiles << objTag << ".png";
            std::cout << "JustinaVision.->save file object name:" << ss.str() << std::endl;
            cv::Mat imaToSave = imaBGR.clone();
            cv::rectangle(imaToSave, detObjList[i].boundBox, cv::Scalar(0,0,255) );
            cv::putText(imaToSave, objTag, detObjList[i].boundBox.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,255) );
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

            //if(euclideanDist[0] > euclideanDist[1])
            if(resp.recog_objects[j].pose.position.x > resp.recog_objects[j+1].pose.position.x)
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

void callback_subStartGripperPosition(const std_msgs::Bool::ConstPtr& msg){
    if(msg->data){
        std::cout << "Try to start gripper position." << std::endl;
        enableGripperPose = true;     
    }
    else{
        std::cout << "Try to stop gripper position." << std::endl;
        enableGripperPose = false;     
    }
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

bool callback_srvFindTable(vision_msgs::FindPlane::Request &req, vision_msgs::FindPlane::Response &resp)
{
    std::cout << "obj_reco_node.-> Find table ... " << std::endl;

    cv::Mat imaBGR;
    cv::Mat imaPCL;

    float normMin = 999999.0;
    float norm = 0.0;

    float z_plane;


    point_cloud_manager::GetRgbd srv;
    if(!cltRgbdRobot.call(srv))
    {
        std::cout << "ObjDetector.->Cannot get point cloud" << std::endl;
        return false;
    }
    JustinaTools::PointCloud2Msg_ToCvMat(srv.response.point_cloud, imaBGR, imaPCL);

    std::vector<PlanarSegment> tablePlane;

    tablePlane = ObjExtractor::GetHorizontalPlanes(imaPCL);

    if(tablePlane.size() > 1 || tablePlane.size() == 0)
        return false;

    cv::Point3f p;
    cv::Point3f pMin;
    std::vector<cv::Point2i> indexes = tablePlane[0].Get_Indexes();
    cv::Point2i minIndex;
    //Get z_prom of eache plane

    for( int j=0; j<(int)indexes.size(); j++)
    {
        p = imaPCL.at< cv::Point3f >( indexes[j] );
        //norm = cv::norm(p);
        norm = sqrt(p.x*p.x + p.y*p.y );
        if (norm < normMin)
        {
            normMin = norm;
            minIndex = indexes[j];
            pMin = p;
        }

        z_plane += p.z;
    }
    z_plane /= (int)indexes.size();
    
    std::cout << "Find plane ---------" << std::endl;
    std::cout << "zPlane:  " << z_plane << std::endl;
    std::cout << "xMin:  " << pMin.x << std::endl;
    std::cout << "yMin:  " << pMin.y << std::endl;
    std::cout << "normMin:  " << normMin << std::endl << std::endl;

    if(z_plane > 0.80 || z_plane < 0.50)
        return false;
    
    resp.nearestPoint.x = pMin.x;
    resp.nearestPoint.y = pMin.y;    
    resp.nearestPoint.z = pMin.z;
    
    cv::Point2i p_min(minIndex.x - 10, minIndex.y - 10);
    cv::Point2i p_max(minIndex.x + 10, minIndex.y + 10);
    rectangle(imaBGR, p_min, p_max, cv::Scalar(255, 0, 0), 2, 8, 0);

    cv::imshow("findTable", imaBGR);
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
    
    float markCentroidScale = .035;
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
        markCentroid.color.a = 0.70; 
        markCentroid.lifetime = ros::Duration(duration, duration); 

        markersList.markers.push_back( markCentroid );         
        std::cout << "Drawing obj " << i  << std::endl; 
    }
    pubRvizMarkers.publish( markersList ); 
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
            std::cout << "\nobj_reco_node.-> EXTERN Training folder: " << data_base_folder << std::endl;
        }
    }
}


bool callback_srvVotationObjects(vision_msgs::DetectObjects::Request &req, vision_msgs::DetectObjects::Response &resp)
{
	std::cout << execMsg  << "srvDetectObjects" << std::endl; 
	std::vector<std::string> cloudName; 
	std::vector<cv::Rect> boundBoxList;
	cv::Mat imaBGR;
	cv::Mat imaPCL;
	if( !GetImagesFromJustina( imaBGR, imaPCL) )
		return false; 
	ObjExtractor::DebugMode = debugMode;
	std::vector<DetectedObject> detObjList = ObjExtractor::GetObjectsInHorizontalPlanes(imaPCL);
	//DrawObjects( detObjList ); 

	if(detObjList.size() == 0)
		return false;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
	cv::Mat imaToShow = imaBGR.clone();
	vision_msgs::VisionObject obj;
	for(int k = 0; k<req.iterations; k++){
		if( !GetImagesFromJustina( imaBGR, imaPCL) )
			return false; 
		std::vector<DetectedObject> detObjList = ObjExtractor::GetObjectsInHorizontalPlanes(imaPCL);
		//DrawObjects( detObjList ); 

		if(detObjList.size() == 0)
			return false;
		for( int i=0; i<detObjList.size(); i++)
		{
			std::string objName = objReco.RecognizeObject( detObjList[i], imaBGR );
			std::string objTag;
			
			obj.id = objName;
			obj.pose.position.x = detObjList[i].centroid.x;
			obj.pose.position.y = detObjList[i].centroid.y;
			obj.pose.position.z = detObjList[i].centroid.z;
				std::cout << "object:  " << obj.id 
				<< " pose: "<< obj.pose.position.x << ", "
				<< obj.pose.position.y << ", "
				<< obj.pose.position.z << std::endl;
			
			
			cloud->push_back(pcl::PointXYZ(detObjList[i].centroid.x, detObjList[i].centroid.y, detObjList[i].centroid.z));
			//(obj.pose.position.x, obj.pose.position.y, obj.pose.position.z));
			cloudName.push_back(objName);
			boundBoxList.push_back(detObjList[i].boundBox);

		}
	}

	std::cout << "point cloud before filtering  has: " << cloud->points.size() << " data points" << std::endl;

	// Create the filtering object: dowsample the datest  using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	vg.setInputCloud (cloud);
	vg.setLeafSize (0.01f, 0.01f, 0.01f);
	vg.filter (*cloud_filtered);
	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl;




 	pcl::PCDWriter writer;

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.1); // 2cm
	ec.setMinClusterSize (1);
	ec.setMaxClusterSize (req.iterations);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
	ec.extract (cluster_indices);

	std::cout << "size of indices: " << cluster_indices.size() << std::endl;

	int j = 0;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		std::map<std::string, int> objMap;
		std::pair<std::map<std::string, int>::iterator, bool> ret;
		 for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
		 	cloud_cluster->points.push_back (cloud->points[*pit]); //*
			if(cloudName.at(*pit) == "")
				cloudName.at(*pit) = "unknown";
			std::cout << "Indice: " << *pit << " Name: " << cloudName.at(*pit) << std::endl;
			ret = objMap.insert(std::pair<std::string,int>(cloudName.at(*pit), 1));
			if (ret.second == false){
				//std::cout << "element " << cloudName.at(*pit) << " alrady exist" << std::endl;
				//std::cout << "with a value of " << ret.first->second << std::endl;
				ret.first->second++;
			}
		 }
		 cloud_cluster->width = cloud_cluster->points.size ();
		 cloud_cluster->height = 1;
		 cloud_cluster->is_dense = true;
		

		 std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
		 std::stringstream ss;
		 ss << "cloud_cluster_" << j << ".pcd";
		 writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
		
		std::map<std::string, int>::iterator tit = objMap.begin();
		std::pair<std::string, int> tag(tit->first, tit->second);
		for(tit = objMap.begin(); tit != objMap.end(); ++tit){
			if(tag.second < tit->second){
				tag.first = tit->first;
				tag.second = tit->second;
			}
		}
	
		std::vector<int>::const_iterator index = it->indices.begin();
        	cv::rectangle(imaToShow, boundBoxList.at(*index), cv::Scalar(0,0,255) );
		//cv::putText(imaToShow, tag.first, boundBoxList.at(*index).tl(), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,255) );
			obj.id = tag.first;
			obj.pose.position.x = cloud->points[*index].x;
			obj.pose.position.y = cloud->points[*index].y;
			obj.pose.position.z = cloud->points[*index].z;
			obj.confidence = (float)tag.second/req.iterations;
				std::cout << "object:  " << obj.id 
				<< " pose: "<< obj.pose.position.x << ", "
				<< obj.pose.position.y << ", "
				<< obj.pose.position.z << std::endl;
			std::cout << "boundBox: " << boundBoxList.at(*index).x << ", "
				  << boundBoxList.at(*index).y << ", "
				  << boundBoxList.at(*index).width << ", "
				  << boundBoxList.at(*index).height << std::endl;
			//resp.recog_objects.push_back(obj);
		ss.str("");
		ss << tag.first << " " << obj.confidence;
		cv::putText(imaToShow, ss.str(), boundBoxList.at(*index).tl(), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,255) );
		obj.x = boundBoxList.at(*index).x;
		obj.y = boundBoxList.at(*index).y;
		obj.width = boundBoxList.at(*index).width;
		obj.height = boundBoxList.at(*index).height;
			resp.recog_objects.push_back(obj);
		ss.str("");
		
		 j++;
	  }

	sensor_msgs::Image container;
	cv_bridge::CvImage cvi_mat;
	std::cout << "out_mat.type() = " << imaBGR.type() << std::endl;
	cvi_mat.encoding = sensor_msgs::image_encodings::BGR8;
	cvi_mat.image = imaBGR;
	cvi_mat.toImageMsg(container);
	resp.image = container;	
	//resp.recog_objects.push_back(obj);

	cv::imshow( "Recognized Objects", imaToShow );
	return true;
}

bool callback_srvTFObjectDetect(vision_msgs::DetectObjects::Request &req, vision_msgs::DetectObjects::Response &resp)
{
	std::cout << execMsg  << "srvDetectObjects" << std::endl; 
	std::vector<std::string> cloudName; 
	std::vector<cv::Rect> boundBoxList;
	cv::Mat imaBGR;
	cv::Mat imaPCL;
	if( !GetImagesFromJustina( imaBGR, imaPCL) )
		return false; 
	ObjExtractor::DebugMode = debugMode;
	std::vector<DetectedObject> detObjList = ObjExtractor::GetObjectsInHorizontalPlanes(imaPCL);
	//DrawObjects( detObjList ); 

	if(detObjList.size() == 0)
		return false;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
	cv::Mat imaToShow = imaBGR.clone();
	vision_msgs::VisionObject obj;
	for(int k = 0; k<req.iterations; k++){
		if( !GetImagesFromJustina( imaBGR, imaPCL) )
			return false; 
		std::vector<DetectedObject> detObjList = ObjExtractor::GetObjectsInHorizontalPlanes(imaPCL);
		//DrawObjects( detObjList ); 

		if(detObjList.size() == 0)
			return false;
		for( int i=0; i<detObjList.size(); i++)
		{
			std::string objName = objReco.RecognizeObject( detObjList[i], imaBGR );
			std::string objTag;
			
			obj.id = objName;
			obj.pose.position.x = detObjList[i].centroid.x;
			obj.pose.position.y = detObjList[i].centroid.y;
			obj.pose.position.z = detObjList[i].centroid.z;
				std::cout << "object:  " << obj.id 
				<< " pose: "<< obj.pose.position.x << ", "
				<< obj.pose.position.y << ", "
				<< obj.pose.position.z << std::endl;
			
			
			cloud->push_back(pcl::PointXYZ(detObjList[i].centroid.x, detObjList[i].centroid.y, detObjList[i].centroid.z));
			//(obj.pose.position.x, obj.pose.position.y, obj.pose.position.z));
			cloudName.push_back(objName);
			boundBoxList.push_back(detObjList[i].boundBox);

		}
	}

	std::cout << "point cloud before filtering  has: " << cloud->points.size() << " data points" << std::endl;

	// Create the filtering object: dowsample the datest  using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	vg.setInputCloud (cloud);
	vg.setLeafSize (0.01f, 0.01f, 0.01f);
	vg.filter (*cloud_filtered);
	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl;




 	pcl::PCDWriter writer;

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.1); // 2cm
	ec.setMinClusterSize (1);
	ec.setMaxClusterSize (req.iterations);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
	ec.extract (cluster_indices);

	std::cout << "size of indices: " << cluster_indices.size() << std::endl;

	int j = 0;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		std::map<std::string, int> objMap;
		std::pair<std::map<std::string, int>::iterator, bool> ret;
		 for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
		 	cloud_cluster->points.push_back (cloud->points[*pit]); //*
			if(cloudName.at(*pit) == "")
				cloudName.at(*pit) = "unknown";
			std::cout << "Indice: " << *pit << " Name: " << cloudName.at(*pit) << std::endl;
			ret = objMap.insert(std::pair<std::string,int>(cloudName.at(*pit), 1));
			if (ret.second == false){
				//std::cout << "element " << cloudName.at(*pit) << " alrady exist" << std::endl;
				//std::cout << "with a value of " << ret.first->second << std::endl;
				ret.first->second++;
			}
		 }
		 cloud_cluster->width = cloud_cluster->points.size ();
		 cloud_cluster->height = 1;
		 cloud_cluster->is_dense = true;
		

		 std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
		 std::stringstream ss;
		 ss << "cloud_cluster_" << j << ".pcd";
		 writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
		
		std::map<std::string, int>::iterator tit = objMap.begin();
		std::pair<std::string, int> tag(tit->first, tit->second);
		for(tit = objMap.begin(); tit != objMap.end(); ++tit){
			if(tag.second < tit->second){
				tag.first = tit->first;
				tag.second = tit->second;
			}
		}
 
		std::vector<int>::const_iterator index = it->indices.begin();
            cv::rectangle(imaToShow, boundBoxList.at(*index), cv::Scalar(0,0,255) );
		//cv::putText(imaToShow, tag.first, boundBoxList.at(*index).tl(), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,255) );
			obj.id = tag.first;
			obj.pose.position.x = cloud->points[*index].x;
			obj.pose.position.y = cloud->points[*index].y;
			obj.pose.position.z = cloud->points[*index].z;
			obj.confidence = (float)tag.second/req.iterations;
				std::cout << "object:  " << obj.id 
				<< " pose: "<< obj.pose.position.x << ", "
				<< obj.pose.position.y << ", "
				<< obj.pose.position.z << std::endl;
			std::cout << "boundBox: " << boundBoxList.at(*index).x << ", "
				  << boundBoxList.at(*index).y << ", "
				  << boundBoxList.at(*index).width << ", "
				  << boundBoxList.at(*index).height << std::endl;
			//resp.recog_objects.push_back(obj);
            
            cv::Rect roi(boundBoxList.at(*index).x, boundBoxList.at(*index).y, boundBoxList.at(*index).width, boundBoxList.at(*index).height);
            ///se manda llamar a la funcion de tensor flow para confirmar la etiqueta del objeto
            sensor_msgs::Image tf_container;
            cv_bridge::CvImage tf_image;
            tf_image.encoding = sensor_msgs::image_encodings::BGR8;
            tf_image.image = imaToShow(roi);
	        //cv::imshow( "Object", tf_image.image);
            tf_image.toImageMsg(tf_container);
            vision_msgs::image_srv tf_srv;
            tf_srv.request.image = tf_container;
            std::string label_response;
            if(cltTFObjects.call(tf_srv)){
                label_response = tf_srv.response.id;
                std::cout << "TF_LABEL: " << label_response << std::endl;
                obj.id = label_response;
            }
            else{
                ROS_INFO("Failed to call service tensor_flow");
                return -1;
            }


		ss.str("");
		ss << tag.first << " " << obj.confidence;
		cv::putText(imaToShow, ss.str(), boundBoxList.at(*index).tl(), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,255) );
		obj.x = boundBoxList.at(*index).x;
		obj.y = boundBoxList.at(*index).y;
		obj.width = boundBoxList.at(*index).width;
		obj.height = boundBoxList.at(*index).height;
			resp.recog_objects.push_back(obj);
		ss.str("");
		
		 j++;
	  }

	sensor_msgs::Image container;
	cv_bridge::CvImage cvi_mat;
	std::cout << "out_mat.type() = " << imaBGR.type() << std::endl;
	cvi_mat.encoding = sensor_msgs::image_encodings::BGR8;
	cvi_mat.image = imaBGR;
	cvi_mat.toImageMsg(container);
	resp.image = container;	
	//resp.recog_objects.push_back(obj);

	cv::imshow( "Recognized Objects", imaToShow );
	return true;
}

bool callback_srvExtractObjectsAbovePlanes(vision_msgs::DetectObjects::Request &req, vision_msgs::DetectObjects::Response &resp){
    std::cout << "obj_recog_node.-> Executing srvExtract Object Above planes " << std::endl;

    point_cloud_manager::GetRgbd srv;
    if(!cltRgbdRobot.call(srv))
    {
        std::cout << "obj_recog_node.-> Cannot get point cloud" << std::endl;
        return false;
    }
    cv::Mat imaBGR;
    cv::Mat imaPCL;
    JustinaTools::PointCloud2Msg_ToCvMat(srv.response.point_cloud, imaBGR, imaPCL);

    cv::Mat objectsExtracted;
    bool isExtractor = ObjExtractor::extractObjectsFromHorizontalPlanes(imaPCL, objectsExtracted);
    if(!isExtractor)
        return false;
    cv::imshow("Object Extracted", objectsExtracted);
	
    sensor_msgs::Image container;
	cv_bridge::CvImage cvi_mat;
	cvi_mat.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
	cvi_mat.image = objectsExtracted;
	cvi_mat.toImageMsg(container);
	resp.image = container;	

    return true;
}

bool callback_srvExtractObjectsWithPlanes(vision_msgs::DetectObjects::Request &req, vision_msgs::DetectObjects::Response &resp){
    std::cout << "obj_recog_node.-> Executing srvExtract Object Above planes " << std::endl;

    point_cloud_manager::GetRgbd srv;
    if(!cltRgbdRobot.call(srv))
    {
        std::cout << "obj_recog_node.-> Cannot get point cloud" << std::endl;
        return false;
    }
    cv::Mat imaBGR;
    cv::Mat imaPCL;
    JustinaTools::PointCloud2Msg_ToCvMat(srv.response.point_cloud, imaBGR, imaPCL);

    cv::Mat objectsExtracted;
    bool isExtractor;
    try{
        isExtractor = ObjExtractor::extractObjectsIncludingPlanes(imaPCL, objectsExtracted);
    }
    catch(std::exception &e){
        std::cout << e.what() << '\n';
    }
    if(!isExtractor)
        return false;
    cv::Mat imaMasked = cv::Mat::zeros(imaBGR.size(), imaBGR.type());
    imaBGR.copyTo(imaMasked, objectsExtracted);
    cv::imshow("Object Extracted Mask", objectsExtracted);
    cv::imshow("Object Extracted", imaMasked);
    
    sensor_msgs::Image container;
    cv_bridge::CvImage cvi_mat;
    cvi_mat.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    cvi_mat.image = objectsExtracted;
    cvi_mat.toImageMsg(container);
    resp.image = container; 

    return true;
}

//Callback that call a actionlib YOLO darknet node
bool callback_srvDetectObjectsYOLO(vision_msgs::DetectObjects::Request &req, vision_msgs::DetectObjects::Response &resp){
    if( !GetImagesFromJustina(lastImaBGR, lastImaPCL) )
        return false;

    sensor_msgs::Image container;
	cv_bridge::CvImage cvi_mat;
	cvi_mat.encoding = sensor_msgs::image_encodings::BGR8;
	cvi_mat.image = lastImaBGR;
	cvi_mat.toImageMsg(container);

    bool bussy = actCltForObjects->waitForServer(ros::Duration(0.1));
    if(!bussy){
        std::cout << "obj_recog_node.-> Error calling the actionlib server to detect YOLO objects server unavailable." << std::endl;
        return false;
    }
    vision_msgs::CheckForObjectsGoal goal;
    goal.image = container;
    actCltForObjects->sendGoal(goal);
    actCltForObjects->waitForResult(ros::Duration(2.0));
   
    if (actCltForObjects->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        vision_msgs::CheckForObjectsResult result = *actCltForObjects->getResult();
        std::vector<vision_msgs::BoundingBox> boundingBoxes = result.bounding_boxes.bounding_boxes;
        std::vector<vision_msgs::BoundingBox>::iterator boundingBoxesIterator;

        for(boundingBoxesIterator = boundingBoxes.begin(); boundingBoxesIterator != boundingBoxes.end(); boundingBoxesIterator++){
            vision_msgs::VisionObject yoloObject;
            yoloObject.id = boundingBoxesIterator->Class;
            yoloObject.confidence = boundingBoxesIterator->probability;
            yoloObject.x = boundingBoxesIterator->xmin;
            yoloObject.y = boundingBoxesIterator->ymin;
            yoloObject.width = fabs(boundingBoxesIterator->xmax - boundingBoxesIterator->xmin);
            yoloObject.height = fabs(boundingBoxesIterator->ymax - boundingBoxesIterator->ymin);
            cv::Rect roi3D;
            roi3D.x = boundingBoxesIterator->xmin;
            roi3D.y = boundingBoxesIterator->ymin;
            roi3D.width = cvRound(fabs(boundingBoxesIterator->xmax - boundingBoxesIterator->xmin));
            roi3D.height = cvRound(fabs(boundingBoxesIterator->ymax - boundingBoxesIterator->ymin));
            cv::Mat objectxyz = lastImaPCL(roi3D).clone();
            cv::Vec3f object3Dcenter = objectxyz.at<cv::Vec3f>(objectxyz.rows * 0.5, objectxyz.cols * 0.5);
            yoloObject.pose.position.x = object3Dcenter[0];
            yoloObject.pose.position.y = object3Dcenter[1];
            yoloObject.pose.position.z = object3Dcenter[2];
            resp.recog_objects.push_back(yoloObject);
        }

    }
    else{
        std::cout << "obj_recog_node.-> Error calling the actionlib server to detect YOLO objects." << std::endl;
        return false;
    }
    showImgYOLO = true;
    cv::namedWindow("YOLO V3");
    cvMoveWindow("YOLO V3", 0, 0);

    return true;
}


void callback_subEnableDetectObjsYOLO(const std_msgs::Bool::ConstPtr& msg){
    std::cout << "obj_reco_node.->Enable detect obst_detec YOLO" << std::endl;
    recivedResponse = !msg->data;
    enableDetectObjectYOLO = msg->data;
    showImgYOLO = msg->data;
    if(msg->data){
        cv::namedWindow("YOLO V3");
        cvMoveWindow("YOLO V3", 0, 0);
    }else
        cv::destroyWindow("YOLO V3");
}


void callback_subDetectObjsYOLO(const vision_msgs::BoundingBoxes::ConstPtr& msg){
    std::cout << "obj_reco_node.->Reciving obst_detec YOLO" << std::endl;
    vision_msgs::VisionObjectList objectsYOLO;
    recivedResponse = false;
    std::vector<vision_msgs::BoundingBox> boundingBoxes = msg->bounding_boxes;
    std::vector<vision_msgs::BoundingBox>::iterator boundingBoxesIterator;

    for(boundingBoxesIterator = boundingBoxes.begin(); boundingBoxesIterator != boundingBoxes.end(); boundingBoxesIterator++){
        vision_msgs::VisionObject yoloObject;
        yoloObject.id = boundingBoxesIterator->Class;
        yoloObject.confidence = boundingBoxesIterator->probability;
        yoloObject.x = boundingBoxesIterator->xmin;
        yoloObject.y = boundingBoxesIterator->ymin;
        yoloObject.width = fabs(boundingBoxesIterator->xmax - boundingBoxesIterator->xmin);
        yoloObject.height = fabs(boundingBoxesIterator->ymax - boundingBoxesIterator->ymin);
        cv::Rect roi3D;
        roi3D.x = boundingBoxesIterator->xmin;
        roi3D.y = boundingBoxesIterator->ymin;
        roi3D.width = cvRound(fabs(boundingBoxesIterator->xmax - boundingBoxesIterator->xmin));
        roi3D.height = cvRound(fabs(boundingBoxesIterator->ymax - boundingBoxesIterator->ymin));
        cv::Mat objectxyz = lastImaPCL(roi3D).clone();
        cv::Vec3f object3Dcenter = objectxyz.at<cv::Vec3f>(objectxyz.rows * 0.5, objectxyz.cols * 0.5);
        yoloObject.pose.position.x = object3Dcenter[0];
        yoloObject.pose.position.y = object3Dcenter[1];
        yoloObject.pose.position.z = object3Dcenter[2];
        objectsYOLO.ObjectList.push_back(yoloObject);
    }
    pubDetectObjsYOLO.publish(objectsYOLO);
}

void callback_subImgDetectObjsYOLO(const sensor_msgs::Image::ConstPtr &msg)
{
    if(showImgYOLO){
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::imshow("YOLO V3", cv_ptr->image);
    }
}
