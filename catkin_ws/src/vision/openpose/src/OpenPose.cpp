#include "openpose/OpenPose.hpp"

OpenPose::OpenPose(){
}
        
OpenPose::~OpenPose(){
    delete cvMatToOpInput;
    delete cvMatToOpOutput;
    delete poseExtractorCaffe;
    delete poseRenderer;
    delete opOutputToCvMat;
}

void OpenPose::initOpenPose(){
    op::log("OpenPose ROS Node", op::Priority::High);
    std::cout << "OpenPose->loggin_level_flag:" << FLAGS_logging_level << std::endl; 
    op::check(0 <= FLAGS_logging_level && FLAGS_logging_level <= 255, "Wrong logging_level value.", __LINE__, __FUNCTION__, __FILE__);
    op::log("", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);
    const auto outputSize = op::flagsToPoint(FLAGS_resolution, "640x480");
    const auto netInputSize = op::flagsToPoint(FLAGS_net_resolution, "640x480");
    const auto netOutputSize = netInputSize;

    const auto poseModel = op::flagsToPoseModel(FLAGS_model_pose);

    if(FLAGS_alpha_pose < 0. || FLAGS_alpha_pose > 1.)
        op::error("Alpha value for blending must be in the range [0,1].", __LINE__, __FUNCTION__, __FILE__);
    if(FLAGS_scale_gap <= 0. || FLAGS_scale_number > 1.)
        op::error("Incompatilble flag configuration: scale_gap must be greater than 0 o scale_number = 1.", __LINE__, __FUNCTION__, __FILE__);
    op::log("", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);

    cvMatToOpInput = new op::CvMatToOpInput(netInputSize, FLAGS_scale_number, (float)FLAGS_scale_gap);
    cvMatToOpOutput = new op::CvMatToOpOutput(outputSize);
    poseExtractorCaffe = new op::PoseExtractorCaffe(netInputSize, netOutputSize, outputSize, FLAGS_scale_number, poseModel, FLAGS_model_folder, FLAGS_num_gpu_start);
    poseRenderer = new op::PoseRenderer(netOutputSize, outputSize, poseModel, nullptr, (float)FLAGS_render_threshold, !FLAGS_disable_blending, (float)FLAGS_alpha_pose);

    opOutputToCvMat = new op::OpOutputToCvMat(outputSize);
    poseExtractorCaffe->initializationOnThread();
    poseRenderer->initializationOnThread();
}

cv::Mat OpenPose::framePoseEstimation(cv::Mat inputImage){
    op::Array<float> netInputArray;
    std::vector<float> scaleRatios;
    std::tie(netInputArray, scaleRatios) = cvMatToOpInput->format(inputImage);
    double scaleInputToOutput;
    op::Array<float> outputArray;
    std::tie(scaleInputToOutput, outputArray) = cvMatToOpOutput->format(inputImage);
    poseExtractorCaffe->forwardPass(netInputArray, {inputImage.cols, inputImage.rows}, scaleRatios);
    const auto poseKeyPoints = poseExtractorCaffe->getPoseKeypoints();
    poseRenderer->renderPose(outputArray, poseKeyPoints);
    return opOutputToCvMat->formatToCvMat(outputArray);
}
