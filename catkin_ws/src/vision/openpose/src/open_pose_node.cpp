#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>

#include <openpose/OpenPose.hpp>

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

OpenPose * openPoseEstimator_ptr;
ros::NodeHandle * nh_ptr;
ros::Subscriber * subPointCloud_ptr;

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){

    cv::Mat bgrImg;
    cv::Mat xyzCloud;

    JustinaTools::PointCloud2Msg_ToCvMat(msg, bgrImg, xyzCloud);
    cv::Mat mask = cv::Mat::zeros(bgrImg.size(), bgrImg.type());
    cv::Mat inputImageOp = cv::Mat::zeros(bgrImg.size(), bgrImg.type());
    for (int i = 0; i < bgrImg.rows; i++)
        for (int j = 0; j < bgrImg.cols; j++) {
            cv::Point3f point = xyzCloud.at<cv::Point3f>(i, j);
            if (point.x != 0 && point.y != 0 && point.z != 0) 
                mask.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
            else
                mask.at<cv::Vec3b>(i, j) = cv::Vec3b(0.0, 0.0, 0.0);
    }
    bgrImg.copyTo(inputImageOp, mask);

    cv::Mat opRec = openPoseEstimator_ptr->framePoseEstimation(inputImageOp);

    if(FLAGS_debug_mode){
        cv::imshow("Mask", mask);
        cv::imshow("Input image OP", inputImageOp);
    }

    cv::imshow("Openpose estimation", opRec);

}

void enableEstimatePoseCallback(const std_msgs::Bool::ConstPtr& enable){
    if(enable->data){
        std::cout << "OpenPoseNode.->Enable Pose estimator" << std::endl;
        subPointCloud_ptr = new ros::Subscriber(nh_ptr->subscribe(FLAGS_rgbd_camera_topic, 1, pointCloudCallback));
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

