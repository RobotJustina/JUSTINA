#include "openpose/OpenPose.hpp"

OpenPose::OpenPose(){
}

OpenPose::~OpenPose(){
    delete scaleAndSizeExtractor;
    delete cvMatToOpInput;
    delete cvMatToOpOutput;
    delete poseRenderer; 
    delete opOutputToCvMat;
    delete frameDisplayer;
}

void OpenPose::initOpenPose(std::string modelFoler, op::PoseModel modelPose, op::Point<int> netResolution, op::Point<int> outputSize, 
        int numGpuStart, float scaleGap, float scaleNumber, bool disableBlending, float renderThreshold, float alphaPose){
    op::log("", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);
    const auto netOutputSize = netResolution;

    if (alphaPose < 0. || alphaPose > 1.)
        op::error("Alpha value for blending must be in the range [0,1].", __LINE__, __FUNCTION__, __FILE__);
    if (scaleGap <= 0. && scaleNumber > 1)
        op::error("Incompatible flag configuration: scale_gap must be greater than 0 or scale_number = 1.",
                __LINE__, __FUNCTION__, __FILE__);

    op::log("", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);
    
    const bool enableGoogleLogging = true;
    scaleAndSizeExtractor = new op::ScaleAndSizeExtractor(netResolution, outputSize, scaleNumber, scaleGap);
    cvMatToOpInput = new op::CvMatToOpInput();
    cvMatToOpOutput = new op::CvMatToOpOutput();
    poseExtractor = std::make_shared<op::PoseExtractorCaffe>(
            modelPose, modelFoler, numGpuStart, std::vector<op::HeatMapType>{}, op::ScaleMode::ZeroToOne,
            enableGoogleLogging
            );

    poseRenderer = new op::PoseCpuRenderer(modelPose, renderThreshold, !disableBlending, alphaPose);
    // This is only for the GpuRenderer heatMap type
    //TODO Put the real value for the FLAGS_alpha_heatmap for now is hardcode
    //poseRenderer = new op::PoseGpuRenderer(modelPose, renderThreshold, !disableBlending, alphaPose);
    //TODO Put the real value for the FLAGS_part_to_show for now is hardcode
    //poseRenderer->setElementToRender(19);

    opOutputToCvMat = new op::OpOutputToCvMat();
    frameDisplayer = new op::FrameDisplayer("OpenPose Tutorial - Example 2", outputSize);
    poseExtractor->initializationOnThread();
    poseRenderer->initializationOnThread();
}

void OpenPose::framePoseEstimation(cv::Mat inputImage, cv::Mat &outputImage, std::vector<std::map<int, std::vector<float> > > &keyPoints){ 
    const op::Point<int> imageSize{inputImage.cols, inputImage.rows};
    keyPoints.clear();
    // Step 2 - Get desired scale sizes
    std::vector<double> scaleInputToNetInputs;
    std::vector<op::Point<int>> netInputSizes;
    double scaleInputToOutput;
    op::Point<int> outputResolution;
    std::tie(scaleInputToNetInputs, netInputSizes, scaleInputToOutput, outputResolution) = scaleAndSizeExtractor->extract(imageSize);
    // Step 3 - Format input image to OpenPose input and output formats
    const auto netInputArray = cvMatToOpInput->createArray(inputImage, scaleInputToNetInputs, netInputSizes);
    auto outputArray = cvMatToOpOutput->createArray(inputImage, scaleInputToOutput, outputResolution); 
    // Step 4 - Estimate poseKeypoints
    poseExtractor->forwardPass(netInputArray, imageSize, scaleInputToNetInputs);
    const auto poseKeyPoints = poseExtractor->getPoseKeypoints();
    const auto scaleNetToOutput = poseExtractor->getScaleNetToOutput();
    // Step 5 - Reject the estimation in the vector float
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
    // Step 6 - Render pose
    poseRenderer->renderPose(outputArray, poseKeyPoints, scaleInputToOutput, scaleNetToOutput);
    // Step 7 - OpenPose output format to cv::Mat
    outputImage = opOutputToCvMat->formatToCvMat(outputArray);
}
