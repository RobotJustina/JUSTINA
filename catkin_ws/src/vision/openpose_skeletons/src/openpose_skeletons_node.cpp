#include <string>
#include <fstream>
#include <streambuf>
#include <algorithm>

#include <set>
#include <tuple>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <vision_msgs/Skeletons.h>
#include <vision_msgs/OpenPoseRecognize.h>

#include <tinyxml.h>

#include <gflags/gflags.h>

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <justina_tools/JustinaTools.h>

#define OP_SKEL_NOSE 0
#define OP_SKEL_NECK 1
#define OP_SKEL_RIGHT_SHOULDER 2
#define OP_SKEL_RIGHT_ELBOW 3
#define OP_SKEL_RIGHT_WRIST 4
#define OP_SKEL_LEFT_SHOULDER 5
#define OP_SKEL_LEFT_ELBOW 6
#define OP_SKEL_LEFT_WRIST 7
#define OP_SKEL_HIP 8
#define OP_SKEL_RIGHT_HIP 9
#define OP_SKEL_RIGHT_KNEE 10
#define OP_SKEL_RIGHT_ANKLE 11
#define OP_SKEL_LEFT_HIP 12
#define OP_SKEL_LEFT_KNEE 13
#define OP_SKEL_LEFT_ANKLE 14

//Node config
DEFINE_bool(debug_mode, true, "The debug mode");
DEFINE_string(rgbd_camera_topic, "/hardware/point_cloud_man/rgbd_wrt_robot", "The rgbd input camera topic.");
DEFINE_string(openpose_topic, "/usb_cam/image_raw", "The openpose topic recognizer.");
DEFINE_string(openpose_service, "openpose/recognize", "The openpose service recognizer");
// Config links
DEFINE_string(file_links_config, "", "Path of the config links.");
DEFINE_double(min_score_pose, 0.15, "Min score pose to detect a keypoint");

ros::NodeHandle * nh_ptr;
ros::Subscriber * subPointCloud_ptr;
ros::Publisher pub3DKeyPointsMarker;
std::vector<std::tuple<int, int, float> > links;
ros::Time lastTimeFrame;
ros::Publisher pubSkeletons;
ros::Publisher pubSkeletons2D;
ros::ServiceClient cltOpenPose;

bool shortPersonImg(const std::map<int, std::vector<float> > &lperson, const std::map<int, std::vector<float> > &rperson){ 
    int lindexMin, rindexMin;
    float minJointX = FLT_MAX;
    for(std::map<int, std::vector<float>>::const_iterator it = lperson.begin(); it != lperson.end(); it++){
        if(it->second[2] >= FLAGS_min_score_pose && it->second[0] < minJointX){
            minJointX = it->second[0];
            lindexMin = it->first;
        }
    }
    minJointX = FLT_MAX;
    for(std::map<int, std::vector<float>>::const_iterator it = rperson.begin(); it != rperson.end(); it++){
        if(it->second[2] >= FLAGS_min_score_pose && it->second[0] < minJointX){
            minJointX = it->second[0];
            rindexMin = it->first;
        }
    }
    std::map<int, std::vector<float>>::const_iterator lit = lperson.find(lindexMin);
    std::map<int, std::vector<float>>::const_iterator rit = rperson.find(rindexMin);
    if(lit != lperson.end() && rit != rperson.end())
        return lperson.find(lindexMin)->second[0] < rperson.find(rindexMin)->second[0];
    else if(lit == lperson.end() && rit != rperson.end())
        return true;
    else if(lit != lperson.end() && rit == rperson.end())
        return false;
    else
        return true;
}

bool initLinksRestrictions(std::string fileXML){
    TiXmlDocument docXML(fileXML);
    bool isLoad = docXML.LoadFile();
    if(!isLoad){
        std::cout << "OpenPose.->Can't not open the xml file config of the openpose node." << std::endl;
        return false;
    }

    TiXmlElement * pElm;
    pElm = docXML.FirstChildElement("open_pose_links");
    if(pElm != nullptr){
        for(TiXmlElement * child = pElm->FirstChildElement(); child; child = child->NextSiblingElement()){
            /*std::cout << child->Attribute("joint_start") << std::endl;
            std::cout << child->Attribute("joint_end") << std::endl;
            std::cout << child->Attribute("restriction") << std::endl;*/
            if(child != nullptr){
                std::tuple<int, int, float> link;
                std::get<0>(link) = atoi(child->Attribute("joint_start"));
                std::get<1>(link) = atoi(child->Attribute("joint_end"));
                std::get<2>(link) = atof(child->Attribute("restriction"));
                links.push_back(link);
            }
        }
    }
    return true;
}

void getKeyPointsFromOpenPose(cv::Mat bgrImg, std::vector<std::map<int, std::vector<float> > > &keyPoints)
{
    vision_msgs::OpenPoseRecognize srv;
    cv_bridge::CvImage cvi_mat;
    sensor_msgs::Image inputImage;
    cvi_mat.encoding = sensor_msgs::image_encodings::BGR8;
    cvi_mat.image = bgrImg;
    cvi_mat.toImageMsg(inputImage);
    srv.request.input_image = inputImage;
    if(!cltOpenPose.call(srv))
    {
        std::cout << "Error invoking in openpose service." << std::endl;
        return; 
    }
    keyPoints.clear();
    for(int i = 0 ; i < srv.response.recognitions.size(); i++)
    {
        std::map<int, std::vector<float> > personKeyPoint;
        for(int j = 0; j < srv.response.recognitions[i].keyPoints.size(); j++)
        {
            std::vector<float> keyPoint;
            keyPoint.push_back(srv.response.recognitions[i].keyPoints[j].x);
            keyPoint.push_back(srv.response.recognitions[i].keyPoints[j].y);
            keyPoint.push_back(srv.response.recognitions[i].keyPoints[j].score);
            personKeyPoint[j] = keyPoint;
        }
        keyPoints.push_back(personKeyPoint);
    }
}

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    cv::Mat bgrImg;
    cv::Mat xyzCloud;

    ros::Time currTimeFrame = ros::Time::now(); 
    JustinaTools::PointCloud2Msg_ToCvMat(msg, bgrImg, xyzCloud);
    cv::Mat mask = cv::Mat::zeros(bgrImg.size(), bgrImg.type());
    cv::Mat maskAllJoints = cv::Mat::zeros(bgrImg.size(), bgrImg.type());
    for (int y = 0; y < xyzCloud.rows; y++)
        for (int x = 0; x < xyzCloud.cols; x++) {
            cv::Point3f point = xyzCloud.at<cv::Point3f>(y, x);
            if (point.x != 0 && point.y != 0 && point.z != 0) 
                mask.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255);
            else
                mask.at<cv::Vec3b>(y, x) = cv::Vec3b(0.0, 0.0, 0.0);
    }
    
    mask.copyTo(maskAllJoints);
    bgrImg.copyTo(bgrImg, mask);
       
    std::vector<std::map<int, std::vector<float> > > keyPoints;
    getKeyPointsFromOpenPose(bgrImg, keyPoints);
    
    std::sort(keyPoints.begin(), keyPoints.end(), shortPersonImg);
    visualization_msgs::MarkerArray markerArray;
    vision_msgs::Skeletons skeletons;
    vision_msgs::Skeletons skeletons2D;
    for(int i = 0; i < keyPoints.size(); i++){
        vision_msgs::Skeleton skeleton;
        vision_msgs::Skeleton skeleton2D;
        std::stringstream ss;
        ss << "person_" << i << "_joints";
        //std::cout << "Person:" << i << ", color:" << ((float)(keyPoints.size() - i)) / ((float) keyPoints.size()) << std::endl;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/base_link";
        marker.header.stamp = ros::Time::now();
        marker.ns = ss.str();
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.id = i;
        marker.lifetime = ros::Duration(currTimeFrame.toSec() - lastTimeFrame.toSec());
        marker.action = visualization_msgs::Marker::ADD;
        marker.color.r = ((float)(keyPoints.size() - i)) / ((float) keyPoints.size());
        marker.color.g = 0.7;//((float)(keyPoints.size() - i)) / ((float) keyPoints.size()) * 0.5;
        marker.color.b = 0.2;//((float)(keyPoints.size() - i)) / ((float) keyPoints.size()) * 0.5;
        marker.color.a = 1.0f;
        marker.scale.x = 0.04;
        marker.scale.y = 0.04;
        marker.scale.z = 0.04;
        skeleton.user_id = i;
        skeleton2D.user_id = i;
    
        std::set<int> keyPointInserted;
        std::set<int> blackList;
        for(int l = 0; l < links.size(); l++){
            int index1 = std::get<0>(links[l]);
            int index2 = std::get<1>(links[l]);
            std::vector<float> k1 = keyPoints[i].find(index1)->second;
            std::vector<float> k2 = keyPoints[i].find(index2)->second;
            int x1 = round(k1[0]);
            int y1 = round(k1[1]);
            float score1 = k1[2];
            int x2 = round(k2[0]);
            int y2 = round(k2[1]);
            float score2 = k2[2];
            /*int x1 = k1[0];
            int y1 = k1[1];
            float score1 = k1[2];
            int x2 = k2[0];
            int y2 = k2[1];
            float score2 = k2[2];*/
            if(score1 >= FLAGS_min_score_pose && score2 >= FLAGS_min_score_pose){
                if(mask.at<cv::Vec3b>(y1, x1).val[0] != 0 && mask.at<cv::Vec3b>(y2, x2).val[0] != 0){
                    cv::Point3f centroid1 = xyzCloud.at<cv::Point3f>(y1, x1);
                    cv::Point3f centroid2 = xyzCloud.at<cv::Point3f>(y2, x2);
                    float dis = cv::norm(centroid1 - centroid2);
                    if(dis < std::get<2>(links[l]) && blackList.find(index1) == blackList.end() && blackList.find(index2) == blackList.end()){
                        if(keyPointInserted.find(index1) == keyPointInserted.end()){
                            cv::circle(mask, cv::Point(x1, y1), 3.0, cv::Scalar(0, 255 / 2, 255), 3.0);
                            keyPointInserted.insert(index1);
                            geometry_msgs::Point msg_point;
                            msg_point.x = centroid1.x;
                            msg_point.y = centroid1.y;
                            msg_point.z = centroid1.z;
                            marker.points.push_back(msg_point);
                        }
                        if(keyPointInserted.find(index2) == keyPointInserted.end()){
                            cv::circle(mask, cv::Point(x2, y2), 3.0, cv::Scalar(0, 255 / 2, 255), 3.0);
                            keyPointInserted.insert(index2);
                            geometry_msgs::Point msg_point;
                            msg_point.x = centroid2.x;
                            msg_point.y = centroid2.y;
                            msg_point.z = centroid2.z;
                            marker.points.push_back(msg_point);
                        }
                    }
                    else{
                        blackList.insert(index2);
                    }
                }
            }
            else{
                if(mask.at<cv::Vec3b>(y1, x1).val[0] == 0)
                    blackList.insert(index1);
                if(mask.at<cv::Vec3b>(y2, x2).val[0] == 0)
                    blackList.insert(index2);
            }
        }

        for(int l = 0; l < links.size(); l++){
            int index1 = std::get<0>(links[l]);
            int index2 = std::get<1>(links[l]);
            std::vector<float> k1 = keyPoints[i].find(index1)->second;
            std::vector<float> k2 = keyPoints[i].find(index2)->second;
            int x1 = round(k1[0]);
            int y1 = round(k1[1]);
            float score1 = k1[2];
            int x2 = round(k2[0]);
            int y2 = round(k2[1]);
            float score2 = k2[2];
            /*int x1 = k1[0];
            int y1 = k1[1];
            float score1 = k1[2];
            int x2 = k2[0];
            int y2 = k2[1];
            float score2 = k2[2];*/
            if(keyPointInserted.find(index1) != keyPointInserted.end() && keyPointInserted.find(index2) != keyPointInserted.end()){
                std::stringstream ss;

                tf::Vector3 c1(xyzCloud.at<cv::Point3f>(y1, x1).x, xyzCloud.at<cv::Point3f>(y1, x1).y, xyzCloud.at<cv::Point3f>(y1, x1).z); 
                tf::Vector3 c2(xyzCloud.at<cv::Point3f>(y2, x2).x, xyzCloud.at<cv::Point3f>(y2, x2).y, xyzCloud.at<cv::Point3f>(y2, x2).z); 

                tf::Vector3 z(0, 0, 1);
                tf::Vector3 p = c1 - c2;
                tf::Vector3 t = z.cross(p);
                float angle = acos(z.dot(p) / p.length());

                tf::Quaternion qe(t, angle);

                ss << "person_" << i << "_link_" << index1 << "_" << index2;
                //ss << "person_" << i << "_links";
                visualization_msgs::Marker link_marker;
                link_marker.header.frame_id = "/base_link";
                link_marker.header.stamp = ros::Time::now();
                link_marker.ns = ss.str();
                link_marker.type = visualization_msgs::Marker::CYLINDER;
                link_marker.id = i;
                link_marker.lifetime = ros::Duration(currTimeFrame.toSec() - lastTimeFrame.toSec());
                link_marker.action = visualization_msgs::Marker::ADD;
                link_marker.color.r = 0.2;//((float)(keyPoints.size() - i)) / ((float) keyPoints.size());
                link_marker.color.g = 0.8;//((float)(keyPoints.size() - i)) / ((float) keyPoints.size());
                link_marker.color.b = ((float)(keyPoints.size() - i)) / ((float) keyPoints.size());
                link_marker.color.a = 1.0f;
                link_marker.scale.x = 0.025;
                link_marker.scale.y = 0.025;
                link_marker.scale.z = p.length() - 0.04;
                link_marker.pose.position.x = (c1.x() + c2.x()) / 2;
                link_marker.pose.position.y = (c1.y() + c2.y()) / 2;
                link_marker.pose.position.z = (c1.z() + c2.z()) / 2;
                link_marker.pose.orientation.x = qe.getX();
                link_marker.pose.orientation.y = qe.getY();
                link_marker.pose.orientation.z = qe.getZ();
                link_marker.pose.orientation.w = qe.getW();
                markerArray.markers.push_back(link_marker);
            } 
        }
        
        for(std::map<int, std::vector<float> >::iterator it = keyPoints[i].begin(); it != keyPoints[i].end(); ++it){
            int x = round(it->second[0]);
            int y = round(it->second[1]);
            float score = it->second[2];
            cv::circle(maskAllJoints, cv::Point(x, y), 3.0, cv::Scalar(0, 255 / 2, 255), 3.0);
            if(score >= FLAGS_min_score_pose && blackList.find(OP_SKEL_NECK) == blackList.end()){
                vision_msgs::SkeletonJoint joint;
                joint.position.x = x;
                joint.position.y = y;
                joint.position.z = 0.0;
                switch(it->first){
                    case OP_SKEL_NECK:
                        joint.name_joint.data = "neck";
                        skeleton2D.joints.push_back(joint);
                        break;
                    case OP_SKEL_NOSE:
                        joint.name_joint.data = "nose";
                        skeleton2D.joints.push_back(joint);
                        break;
                    case OP_SKEL_RIGHT_SHOULDER:
                        joint.name_joint.data = "right_shoulder";
                        skeleton2D.joints.push_back(joint);
                        break;
                    case OP_SKEL_RIGHT_ELBOW:
                        joint.name_joint.data = "right_elbow";
                        skeleton2D.joints.push_back(joint);
                        break;
                    case OP_SKEL_RIGHT_WRIST:
                        joint.name_joint.data = "right_wrist";
                        skeleton2D.joints.push_back(joint);
                        break;
                    case OP_SKEL_LEFT_SHOULDER:
                        joint.name_joint.data = "left_shoulder";
                        skeleton2D.joints.push_back(joint);
                        break;
                    case OP_SKEL_LEFT_ELBOW:
                        joint.name_joint.data = "left_elbow";
                        skeleton2D.joints.push_back(joint);
                        break;
                    case OP_SKEL_LEFT_WRIST:
                        joint.name_joint.data = "left_wrist";
                        skeleton2D.joints.push_back(joint);
                        break;
                    case OP_SKEL_HIP:
                        joint.name_joint.data = "hip";
                        skeleton2D.joints.push_back(joint);
                        break;
                    case OP_SKEL_RIGHT_HIP:
                        joint.name_joint.data = "right_hip";
                        skeleton2D.joints.push_back(joint);
                        break;
                    case OP_SKEL_RIGHT_KNEE:
                        joint.name_joint.data = "right_knee";
                        skeleton2D.joints.push_back(joint);
                        break;
                    case OP_SKEL_RIGHT_ANKLE:
                        joint.name_joint.data = "right_ankle";
                        skeleton2D.joints.push_back(joint);
                        break;
                    case OP_SKEL_LEFT_HIP:
                        joint.name_joint.data = "left_hip";
                        skeleton2D.joints.push_back(joint);
                        break;
                    case OP_SKEL_LEFT_KNEE:
                        joint.name_joint.data = "left_knee";
                        skeleton2D.joints.push_back(joint);
                        break;
                    case OP_SKEL_LEFT_ANKLE:
                        joint.name_joint.data = "left_ankle";
                        skeleton2D.joints.push_back(joint);
                        break;
                    default:
                        //std::cout << "OpenPose.->" << "No valid joint id:" << it->first << std::endl;
                        break;
                }
            }
        }
        
        for(std::set<int>::iterator it = keyPointInserted.begin(); it != keyPointInserted.end(); it++){
            std::vector<float> k1 = keyPoints[i].find(*it)->second;
            int x1 = round(k1[0]);
            int y1 = round(k1[1]);
            vision_msgs::SkeletonJoint joint;
            joint.position.x = xyzCloud.at<cv::Point3f>(y1, x1).x;
            joint.position.y = xyzCloud.at<cv::Point3f>(y1, x1).y;
            joint.position.z = xyzCloud.at<cv::Point3f>(y1, x1).z;
            switch(*it){
                case OP_SKEL_NECK:
                    joint.name_joint.data = "neck";
                    break;
                case OP_SKEL_NOSE:
                    joint.name_joint.data = "nose";
                    break;
                case OP_SKEL_RIGHT_SHOULDER:
                    joint.name_joint.data = "right_shoulder";
                    break;
                case OP_SKEL_RIGHT_ELBOW:
                    joint.name_joint.data = "right_elbow";
                    break;
                case OP_SKEL_RIGHT_WRIST:
                    joint.name_joint.data = "right_wrist";
                    break;
                case OP_SKEL_LEFT_SHOULDER:
                    joint.name_joint.data = "left_shoulder";
                    break;
                case OP_SKEL_LEFT_ELBOW:
                    joint.name_joint.data = "left_elbow";
                    break;
                case OP_SKEL_LEFT_WRIST:
                    joint.name_joint.data = "left_wrist";
                    break;
                case OP_SKEL_HIP:
                    joint.name_joint.data = "hip";
                    break;
                case OP_SKEL_RIGHT_HIP:
                    joint.name_joint.data = "right_hip";
                    break;
                case OP_SKEL_RIGHT_KNEE:
                    joint.name_joint.data = "right_knee";
                    break;
                case OP_SKEL_RIGHT_ANKLE:
                    joint.name_joint.data = "right_ankle";
                    break;
                case OP_SKEL_LEFT_HIP:
                    joint.name_joint.data = "left_hip";
                    break;
                case OP_SKEL_LEFT_KNEE:
                    joint.name_joint.data = "left_knee";
                    break;
                case OP_SKEL_LEFT_ANKLE:
                    joint.name_joint.data = "left_ankle";
                    break;
                default:
                    std::cout << "OpenPose.->" << "No valid joint id." << std::endl;
                    break;
            }
            skeleton.joints.push_back(joint);
        }
            
        geometry_msgs::Point32 refPointMsg;
        std::vector<float> k = keyPoints[i].find(OP_SKEL_NECK)->second;
        int x1 = round(k[0]);
        int y1 = round(k[1]);
        refPointMsg.x = xyzCloud.at<cv::Point3f>(y1, x1).x;
        refPointMsg.y = xyzCloud.at<cv::Point3f>(y1, x1).y;
        refPointMsg.z = xyzCloud.at<cv::Point3f>(y1, x1).z;
        skeleton.ref_point = refPointMsg;
        skeleton2D.ref_point = refPointMsg;
        
        if(skeleton.joints.size() > 0)
            skeletons.skeletons.push_back(skeleton);
        if(skeleton2D.joints.size() > 0)
            skeletons2D.skeletons.push_back(skeleton2D);
        markerArray.markers.push_back(marker);
    }
    lastTimeFrame = currTimeFrame;

    pub3DKeyPointsMarker.publish(markerArray);
    pubSkeletons.publish(skeletons);
    pubSkeletons2D.publish(skeletons2D);

    if(FLAGS_debug_mode)
    {
        cv::imshow("Mask", mask);
        cv::imshow("Mask all joints", maskAllJoints);
    }

}

void resultImageCallback(const sensor_msgs::ImageConstPtr &image)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception &e)
    {
        ROS_ERROR("openpose_skeletons_node.->cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat imaBGR = cv_ptr->image;
    cv::imshow("Openpose result", imaBGR);
}

void enableEstimatePoseCallback(const std_msgs::Bool::ConstPtr& enable){
    if(enable->data){
        std::cout << "OpenPoseNode.->Enable Pose estimator" << std::endl;
        subPointCloud_ptr = new ros::Subscriber(nh_ptr->subscribe(FLAGS_rgbd_camera_topic, 1, pointCloudCallback));
        lastTimeFrame = ros::Time::now();
    }
    else{
        std::cout << "OpenPoseNode.->Disable Pose estimator" << std::endl;
        subPointCloud_ptr->shutdown();
        cv::destroyAllWindows();
    }
}

int main(int argc, char ** argv){

    ros::init(argc, argv, "openpose_skeleton_node");
    std::cout << "openpose_skeletons_node.->Initializing the openpose skeleton node by Rey" << std::endl;
    ros::NodeHandle nh;
    nh_ptr = &nh;
    ros::Rate rate(30);
    
    if(ros::param::has("~debug_mode"))
        ros::param::get("~debug_mode", FLAGS_debug_mode);
    if(ros::param::has("~rgbd_camera_topic"))
        ros::param::get("~rgbd_camera_topic", FLAGS_rgbd_camera_topic);
    if(ros::param::has("~openpose_topic"))
        ros::param::get("~openpose_topic", FLAGS_openpose_topic);
    if(ros::param::has("~openpose_service"))
        ros::param::get("~openpose_service", FLAGS_openpose_service);
    if(ros::param::has("~file_links_config"))
        ros::param::get("~file_links_config", FLAGS_file_links_config);
    bool initLinks = initLinksRestrictions(FLAGS_file_links_config);
    if(!initLinks)
        return -1;

    std::cout << "openpose_skeletons_node.->The node will be initializing with the next parameters" << std::endl;
    std::cout << "openpose_skeletons_node.->Debug mode:" << FLAGS_debug_mode << std::endl;
    std::cout << "openpose_skeletons_node.->rgbd camera topic:" << FLAGS_rgbd_camera_topic << std::endl;

    //ros::Subscriber * subPointCloud = nh_ptr->subscribe("/hardware/point_cloud_man/rgbd_wrt_robot", 1, pointCloudCallback);
    ros::Subscriber subEnableEstimatePose = nh.subscribe("/vision/openpose_skeletons/enable_estimate_pose", 1, enableEstimatePoseCallback);
    ros::Subscriber subResultImage = nh.subscribe("/vision/openpose/result_image", 1, resultImageCallback);
    cltOpenPose = nh.serviceClient<vision_msgs::OpenPoseRecognize>((std::string) FLAGS_openpose_service);
    pub3DKeyPointsMarker = nh.advertise<visualization_msgs::MarkerArray>("/vision/openpose_skeletons/skeleton_marker_key_points", 1);
    pubSkeletons = nh.advertise<vision_msgs::Skeletons>("/vision/openpose_skeletons/skeleton_recog", 1);
    pubSkeletons2D = nh.advertise<vision_msgs::Skeletons>("/vision/openpose_skletons/skeleton_recog_2D", 1);

    while(ros::ok()){
        cv::waitKey(1);
        rate.sleep();
        ros::spinOnce();
    }

    delete subPointCloud_ptr;

    return 1;

}

