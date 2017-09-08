#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>

#include <openpose/OpenPose.hpp>

#include <justina_tools/JustinaTools.h>

DEFINE_int32(logging_level, 3, "The logging level. Integer in the range [0, 255]. 0 will output any log() message, while 255 will not output any. Current OpenPose library messages are in the range 0-4: 1 for low priority messages and 4 for important ones.");
// Camera Topic
DEFINE_string(camera_topic, "/camera/image_raw", "Image topic that OpenPose will process.");
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
    cv::imshow("Mask", mask);
    cv::imshow("Input image OP", inputImageOp);

    /*cv::Mat maskGRAY, maskBinary;
    cv::Mat maskBGR = cv::Mat::zeros(bgrImg.size(), CV_32FC3);
    cv::cvtColor(xyzCloud, maskGRAY, CV_BGR2GRAY);
    cv::threshold(maskGRAY, maskBinary, 0, 255, 0);
    cv::Mat b, g, r = bgrImg.split();
    cv::Mat mask
    bgrImg.copyTo(maskBGR, cv::Mat(bgrImg.size(), bgrImg.type(), maskBinary.data));
    //bgrImg.copyTo(maskBGR, maskBinary);
    cv::imshow("maskGRAY", maskGRAY);
    cv::imshow("maskBinary", maskBinary);
    cv::imshow("maskBGR", maskBGR);*/

    /*cv::Mat mask;
    xyzCloud.copyTo(mask, bgrImg);
    cv::Mat maskedImage;
    bgrImg.copyTo(maskedImage, mask);*/

    cv::Mat opRec = openPoseEstimator_ptr->framePoseEstimation(inputImageOp);

    cv::imshow("Openpose estimation", opRec);

}

void enableEstimatePoseCallback(const std_msgs::Bool::ConstPtr& enable){
    if(enable->data){
        std::cout << "OpenPoseNode.->Enable Pose estimator" << std::endl;
        subPointCloud_ptr = new ros::Subscriber(nh_ptr->subscribe("/hardware/point_cloud_man/rgbd_wrt_robot", 1, pointCloudCallback));
    }
    else{
        std::cout << "OpenPoseNode.->Disable Pose estimator" << std::endl;
        subPointCloud_ptr->shutdown();
        cv::destroyAllWindows();
    }
}

int main(int argc, char ** argv){

    ros::init(argc, argv, "open_pose_node");
    std::cout << "Init the openpose node by Rey" << std::endl;
    ros::NodeHandle nh;
    nh_ptr = &nh;
    ros::Rate rate(30);

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

