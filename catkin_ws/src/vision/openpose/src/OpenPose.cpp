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

void OpenPose::initOpenPose(std::string modelFoler, op::PoseModel modelPose, op::Point<int> netResolution, op::Point<int> outputSize, 
    int numGpuStart, float scaleGap, float scaleNumber, bool disableBlending, float renderThreshold, float alphaPose){
    op::log("", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);
    /*const auto outputSize = op::flagsToPoint(outputSize, "640x480");
    const auto netInputSize = op::flagsToPoint(netResolution, "640x480");*/
    const auto netOutputSize = netResolution;

    if(alphaPose < 0. || alphaPose > 1.)
        op::error("Alpha value for blending must be in the range [0,1].", __LINE__, __FUNCTION__, __FILE__);
    if(scaleGap <= 0. || scaleNumber > 1.)
        op::error("Incompatilble flag configuration: scale_gap must be greater than 0 o scale_number = 1.", __LINE__, __FUNCTION__, __FILE__);
    op::log("", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);

    cvMatToOpInput = new op::CvMatToOpInput(netResolution, scaleNumber, scaleGap);
    cvMatToOpOutput = new op::CvMatToOpOutput(outputSize);
    poseExtractorCaffe = new op::PoseExtractorCaffe(netResolution, netOutputSize, outputSize, scaleNumber, modelPose, modelFoler, numGpuStart);
    poseRenderer = new op::PoseRenderer(netOutputSize, outputSize, modelPose, nullptr, renderThreshold, !disableBlending, alphaPose);

    opOutputToCvMat = new op::OpOutputToCvMat(outputSize);
    poseExtractorCaffe->initializationOnThread();
    poseRenderer->initializationOnThread();
}

void OpenPose::framePoseEstimation(cv::Mat inputImage, cv::Mat &outputImage, std::vector<std::map<int, std::vector<float> > > &keyPoints){
//void framePoseEstimation(cv::Mat inputImage, cv::Mat &outputImage, std::vector<std::pair<int, std::vector<float> > &keyPoints){
    op::Array<float> netInputArray;
    std::vector<float> scaleRatios;
    std::tie(netInputArray, scaleRatios) = cvMatToOpInput->format(inputImage);
    double scaleInputToOutput;
    op::Array<float> outputArray;
    std::tie(scaleInputToOutput, outputArray) = cvMatToOpOutput->format(inputImage);

    keyPoints.clear();

    poseExtractorCaffe->forwardPass(netInputArray, {inputImage.cols, inputImage.rows}, scaleRatios);
    const auto poseKeyPoints = poseExtractorCaffe->getPoseKeypoints();

    const auto numberPeopleDetected = poseKeyPoints.getSize(0);
    //std::cout << "OpenPose.->Number of people detected:" << numberPeopleDetected << std::endl; 
    const auto numberBodyParts = poseKeyPoints.getSize(1);
    //std::cout << "OpenPose.->Number of body parts:" << numberBodyParts << std::endl; 
    for(int i = 0; i < numberPeopleDetected; i++){
        std::map<int, std::vector<float> > bodyParts;
        for(int j = 0; j < numberBodyParts; j++){
            std::vector<float> data; 
            auto baseIndex = poseKeyPoints.getSize(2) * (i * numberBodyParts + j);
            auto x = poseKeyPoints[baseIndex];
            auto y = poseKeyPoints[baseIndex + 1];
            auto score = poseKeyPoints[baseIndex + 2];
            data.push_back(x);
            data.push_back(y);
            data.push_back(score);
            bodyParts[j] = data;
        }
        keyPoints.push_back(bodyParts);
    }
   
    poseRenderer->renderPose(outputArray, poseKeyPoints);

    outputImage = opOutputToCvMat->formatToCvMat(outputArray);
}
