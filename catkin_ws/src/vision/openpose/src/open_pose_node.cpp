#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <openpose/OpenPose.hpp>

#include <justina_tools/JustinaTools.h>

OpenPose openPoseEstimator;

void subPointCloudRGB(const sensor_msgs::PointCloud2::ConstPtr& msg){
    
    cv::Mat bgrImg;
    cv::Mat xyzCloud;

    JustinaTools::PointCloud2Msg_ToCvMat(msg, bgrImg, xyzCloud);

    cv::Mat opRec = openPoseEstimator.framePoseEstimation(bgrImg);

    cv::imshow("Openpose estimation", opRec);

}

int main(int argc, char ** argv){
    
    ros::init(argc, argv, "open_pose_node");
    std::cout << "Init the openpose node by Rey" << std::endl;
    ros::NodeHandle nh;
    ros::Rate rate(30);

    openPoseEstimator.initOpenPose();

    while(ros::ok()){

        cv::waitKey(1);

        rate.sleep();
        ros::spinOnce();
    }

    return 1;

}

