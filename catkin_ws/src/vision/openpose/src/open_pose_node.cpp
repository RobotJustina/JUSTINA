#include <set>
#include <tuple>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <openpose/OpenPose.hpp>

#include <Eigen/Dense>

#include <justina_tools/JustinaTools.h>

//Node config
DEFINE_bool(debug_mode, true, "The debug mode");
DEFINE_int32(logging_level, 3, "The logging level. Integer in the range [0, 255]. 0 will output any log() message, while 255 will not output any. Current OpenPose library messages are in the range 0-4: 1 for low priority messages and 4 for important ones.");
DEFINE_string(rgbd_camera_topic, "/hardware/point_cloud_man/rgbd_wrt_robot", "The rgbd input camera topic.");
DEFINE_string(result_pose_topic, "", "The result pose topic.");
// OpenPose
DEFINE_string(model_folder, "/opt/openpose/models/",      "Folder path (absolute or relative) where the models (pose, face, ...) are located.");
DEFINE_string(model_pose, "COCO", "Model to be used (e.g. COCO, MPI, MPI_4_layers).");
DEFINE_string(net_resolution, "640x480", "Multiples of 16. If it is increased, the accuracy potentially increases. If it is decreased, the speed increases. For maximum speed-accuracy balance, it should keep the closest aspect ratio possible to the images or videos to be processed. E.g. the default `656x368` is optimal for 16:9 videos, e.g. full HD (1980x1080) and HD (1280x720) videos.");
DEFINE_string(resolution, "640x480", "The image resolution (display and output). Use \"-1x-1\" to force the program to use the default images resolution.");
DEFINE_int32(num_gpu_start, 0, "GPU device start number.");
DEFINE_double(scale_gap, 0.3, "Scale gap between scales. No effect unless scale_number > 1. Initial scale is always 1. If you want to change the initial scale, you actually want to multiply the `net_resolution` by your desired initial scale.");
DEFINE_int32(scale_number, 1, "Number of scales to average.");
// OpenPose Rendering
DEFINE_bool(disable_blending, false, "If blending is enabled, it will merge the results with the original frame. If disabled, it will only display the results.");
DEFINE_double(render_threshold, 0.05, "Only estimated keypoints whose score confidences are higher than this threshold will be rendered. Generally, a high threshold (> 0.5) will only render very clear body parts;  while small thresholds (~0.1) will also output guessed and occluded keypoints, but also  more false positives (i.e. wrong detections).");
DEFINE_double(alpha_pose, 0.6, "Blending factor (range 0-1) for the body part rendering. 1 will show it completely, 0 will hide it. Only valid for GPU rendering.");
DEFINE_double(min_score_pose, 0.5, "Min score pose to detect a jeypoint.");
DEFINE_int32(nearest_pixel, 0, "Max pixel to find a depth point");

OpenPose * openPoseEstimator_ptr;
ros::NodeHandle * nh_ptr;
ros::Subscriber * subPointCloud_ptr;
ros::Publisher pub3DKeyPointsMarker;
ros::Time lastTimeFrame;

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){

    cv::Mat bgrImg;
    cv::Mat xyzCloud;

    ros::Time currTimeFrame = ros::Time::now(); 
    JustinaTools::PointCloud2Msg_ToCvMat(msg, bgrImg, xyzCloud);
    cv::Mat mask = cv::Mat::zeros(bgrImg.size(), bgrImg.type());
    cv::Mat maskAllJoints = cv::Mat::zeros(bgrImg.size(), bgrImg.type());
    //std::cout << "Size of the bgrImg Cols:" << bgrImg.cols << ", rows:" << bgrImg.rows << std::endl; 
    //std::cout << "Size of the pcl Cols:" << xyzCloud.cols << ", rows:" << xyzCloud.rows << std::endl; 
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
    cv::Mat opResult;
    std::vector<std::map<int, std::vector<float> > > keyPoints;
    openPoseEstimator_ptr->framePoseEstimation(bgrImg, opResult, keyPoints);
    visualization_msgs::MarkerArray markerArray;
    for(int i = 0; i < keyPoints.size(); i++){
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
    
        std::vector<std::tuple<int, int, float> > links;
        std::tuple<int, int, float> link;
        std::get<0>(link) = 1;
        std::get<1>(link) = 0;
        std::get<2>(link) = 0.3;
        links.push_back(link);
        std::get<0>(link) = 1;
        std::get<1>(link) = 2;
        std::get<2>(link) = 0.4;
        links.push_back(link);
        std::get<0>(link) = 2;
        std::get<1>(link) = 3;
        std::get<2>(link) = 0.5;
        links.push_back(link);
        std::get<0>(link) = 3;
        std::get<1>(link) = 4;
        std::get<2>(link) = 0.5;
        links.push_back(link);
        std::get<0>(link) = 1;
        std::get<1>(link) = 5;
        std::get<2>(link) = 0.4;
        links.push_back(link);
        std::get<0>(link) = 5;
        std::get<1>(link) = 6;
        std::get<2>(link) = 0.4;
        links.push_back(link);
        std::get<0>(link) = 6;
        std::get<1>(link) = 7;
        std::get<2>(link) = 0.4;
        links.push_back(link);
        std::get<0>(link) = 1;
        std::get<1>(link) = 8;
        std::get<2>(link) = 1.2;
        links.push_back(link);
        std::get<0>(link) = 8;
        std::get<1>(link) = 9;
        std::get<2>(link) = 0.7;
        links.push_back(link);
        std::get<0>(link) = 9;
        std::get<1>(link) = 10;
        std::get<2>(link) = 0.7;
        links.push_back(link);
        std::get<0>(link) = 1;
        std::get<1>(link) = 11;
        std::get<2>(link) = 1.2;
        links.push_back(link);
        std::get<0>(link) = 11;
        std::get<1>(link) = 12;
        std::get<2>(link) = 0.7;
        links.push_back(link);
        std::get<0>(link) = 12;
        std::get<1>(link) = 13;
        std::get<2>(link) = 0.7;
        links.push_back(link);

        std::set<int> keyPointInserted;
        std::set<int> blackList;
        for(int l = 0; l < links.size(); l++){
            //float disTwoKeyPoint = 0.0;
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
            if(score1 > FLAGS_min_score_pose && score2 > FLAGS_min_score_pose){
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
                        //blackList.insert(index1);
                        blackList.insert(index2);
                    }
                }
            }
            else{
                if(mask.at<uchar>(x1, y1) == 0)
                    blackList.insert(index1);
                if(mask.at<uchar>(x2, y2) == 0)
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
                /*cv::Point3f centroid1 = xyzCloud.at<cv::Point3f>(k1[1], k1[0]);
                cv::Point3f centroid2 = xyzCloud.at<cv::Point3f>(k2[1], k2[0]);*/

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
            if(score > FLAGS_min_score_pose){
                cv::circle(maskAllJoints, cv::Point(x, y), 3.0, cv::Scalar(0, 255 / 2, 255), 3.0);
            }
        }
        markerArray.markers.push_back(marker);
    }
    lastTimeFrame = currTimeFrame;

    pub3DKeyPointsMarker.publish(markerArray);

    if(FLAGS_debug_mode){
        cv::imshow("Mask", mask);
        cv::imshow("Mask all joints", maskAllJoints);
    }

    cv::imshow("Openpose estimation", opResult);
    
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

    ros::init(argc, argv, "open_pose_node");
    std::cout << "open_pose_node.->Initializing the openpose node by Rey" << std::endl;
    ros::NodeHandle nh;
    nh_ptr = &nh;
    ros::Rate rate(30);
    
    if(ros::param::has("~debug_mode"))
        ros::param::get("~debug_mode", FLAGS_debug_mode);
    if(ros::param::has("~model_folder"))
        ros::param::get("~model_folder", FLAGS_model_folder);
    if(ros::param::has("~model_pose"))
        ros::param::get("~model_pose", FLAGS_model_pose);
    if(ros::param::has("~net_resolution"))
        ros::param::get("~net_resolution", FLAGS_net_resolution);
    if(ros::param::has("~resolution"))
        ros::param::get("~resolution", FLAGS_resolution);
    if(ros::param::has("~num_gpu_start"))
        ros::param::get("~num_gpu_start", FLAGS_num_gpu_start);
    if(ros::param::has("~scale_gap"))
        ros::param::get("~scale_gap", FLAGS_scale_gap);
    if(ros::param::has("~scale_number"))
        ros::param::get("~scale_number", FLAGS_scale_number);
    if(ros::param::has("~render_threshold"))
        ros::param::get("~render_threshold", FLAGS_render_threshold);
    if(ros::param::has("~rgbd_camera_topic"))
        ros::param::get("~rgbd_camera_topic", FLAGS_rgbd_camera_topic);
    if(ros::param::has("~result_pose_topic"))
        ros::param::get("~result_pose_topic", FLAGS_result_pose_topic);

    std::cout << "open_pose_node.->The node will be initializing with the next parameters" << std::endl;
    std::cout << "open_pose_node.->Debug mode:" << FLAGS_debug_mode << std::endl;
    std::cout << "open_pose_node.->Model folder:" << FLAGS_model_folder << std::endl;
    std::cout << "open_pose_node.->Model pose:" << FLAGS_model_pose << std::endl;
    std::cout << "open_pose_node.->Net resolution:" << FLAGS_net_resolution << std::endl;
    std::cout << "open_pose_node.->Resolution:" << FLAGS_resolution << std::endl;
    std::cout << "open_pose_node.->Num gpu start:" << FLAGS_num_gpu_start << std::endl;
    std::cout << "open_pose_node.->Scale gap:" << FLAGS_scale_gap << std::endl;
    std::cout << "open_pose_node.->Scale number:" << FLAGS_scale_number << std::endl;
    std::cout << "open_pose_node.->Render threshold:" << FLAGS_render_threshold << std::endl;
    std::cout << "open_pose_node.->rgbd camera topic:" << FLAGS_rgbd_camera_topic << std::endl;
    std::cout << "open_pose_node.->Result pose topic:" << FLAGS_result_pose_topic << std::endl;

    op::log("OpenPose ROS Node", op::Priority::High);
    std::cout << "OpenPose->loggin_level_flag:" << FLAGS_logging_level << std::endl; 
    op::check(0 <= FLAGS_logging_level && FLAGS_logging_level <= 255, "Wrong logging_level value.", __LINE__, __FUNCTION__, __FILE__);

    //ros::Subscriber * subPointCloud = nh_ptr->subscribe("/hardware/point_cloud_man/rgbd_wrt_robot", 1, pointCloudCallback);
    ros::Subscriber subEnableEstimatePose = nh.subscribe("/vision/openpose/enable_estimate_pose", 1, enableEstimatePoseCallback);
    pub3DKeyPointsMarker = nh.advertise<visualization_msgs::MarkerArray>("/vision/openpose/persons_marker_key_points", 1);

    std::string modelFoler = (std::string) FLAGS_model_folder;
    op::PoseModel poseModel =  op::flagsToPoseModel(FLAGS_model_pose);
    op::Point<int> netResolution = op::flagsToPoint(FLAGS_net_resolution);
    op::Point<int> outputSize = op::flagsToPoint(FLAGS_resolution);
    int numGpuStart = (int) FLAGS_num_gpu_start;
    float scaleGap = (float) FLAGS_scale_gap;
    float scaleNumber = (float) FLAGS_scale_number;
    bool disableBlending = (bool) FLAGS_disable_blending;
    float renderThreshold = (float) FLAGS_render_threshold;
    float alphaPose = (float) FLAGS_alpha_pose;

    openPoseEstimator_ptr = new OpenPose();
    openPoseEstimator_ptr->initOpenPose(modelFoler, poseModel, netResolution, outputSize, numGpuStart, scaleGap, scaleNumber, disableBlending, renderThreshold, alphaPose);

    while(ros::ok()){

        cv::waitKey(1);

        rate.sleep();
        ros::spinOnce();
    }

    delete subPointCloud_ptr;

    return 1;

}

