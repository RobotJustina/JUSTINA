#include "justina_tools/JustinaVision.h"

bool JustinaVision::is_node_set = false;
//Members for operating pano maker
ros::Publisher JustinaVision::pubTakePanoMaker;
ros::Publisher JustinaVision::pubClearPanoMaker;
ros::Publisher JustinaVision::pubMakePanoMaker;
ros::Subscriber JustinaVision::subPanoImage;
sensor_msgs::Image JustinaVision::lastImage;
bool JustinaVision::panoImageRecived;
//Members for operating skeleton finder
ros::Publisher JustinaVision::pubSktEnableRecog;
ros::Subscriber JustinaVision::subGestures;
ros::Subscriber JustinaVision::subSkeletons;
ros::Subscriber JustinaVision::subLeftHandPositions;
ros::Subscriber JustinaVision::subRightHandPositions;
std::vector<vision_msgs::Skeleton> JustinaVision::lastSkeletons;
std::vector<vision_msgs::GestureSkeleton> JustinaVision::lastGestureRecog;
std::vector<geometry_msgs::Point> JustinaVision::lastLeftHandPos;
std::vector<geometry_msgs::Point> JustinaVision::lastRightHandPos;
//Members for operating face recognizer
ros::Publisher JustinaVision::pubStartFaceDetection;
ros::Publisher JustinaVision::pubStartFaceRecognition;
ros::Publisher JustinaVision::pubStartFaceRecognition2D;
ros::Publisher JustinaVision::pubSetIdFaceRecognition;
ros::Publisher JustinaVision::pubTrainerFaces;
ros::Publisher JustinaVision::pubClearFacesDB;
ros::Publisher JustinaVision::pubClearFacesDBByID;
/*ros::Publisher JustinaVision::pubTrainFace;
ros::Publisher JustinaVision::pubRecFace;
ros::Publisher JustinaVision::pubRecFaceByID;
ros::Subscriber JustinaVision::subTrainer;*/
ros::Subscriber JustinaVision::subFaces;
ros::ServiceClient JustinaVision::cltPanoFaceReco;
std::vector<vision_msgs::VisionFaceObject> JustinaVision::lastRecognizedFaces;
int JustinaVision::lastFaceRecogResult = 0;
//Service for face recognition
ros::ServiceClient JustinaVision::cltDetectPanoFaces;
ros::ServiceClient JustinaVision::cltDetectFaces;
ros::ServiceClient JustinaVision::cltDetectWaving;
ros::ServiceClient JustinaVision::cltFaceRecognition;
ros::ServiceClient JustinaVision::cltFaceRecognition2D;
//Members for thermal camera
ros::Publisher JustinaVision::pubStartThermalCamera;
ros::Publisher JustinaVision::pubStopThermalCamera;
//Members for operation of qr reader
ros::Publisher JustinaVision::pubQRReaderStart;
//Services for getting point cloud
ros::ServiceClient JustinaVision::cltGetRgbdWrtKinect;
ros::ServiceClient JustinaVision::cltGetRgbdWrtRobot;
//Detect objects
ros::ServiceClient JustinaVision::cltDetectObjects;
ros::ServiceClient JustinaVision::cltDetectAllObjects;
ros::ServiceClient JustinaVision::cltDetectAllObjectsVot;
ros::ServiceClient JustinaVision::cltDetecObjectsYOLO;
ros::Publisher JustinaVision::pubObjStartRecog;
ros::Publisher JustinaVision::pubObjStopRecog;
ros::Publisher JustinaVision::pubObjStartWin;
ros::Publisher JustinaVision::pubObjStopWin;
ros::Publisher JustinaVision::pubEnableObjsDetectYOLO;
ros::Subscriber JustinaVision::subGetRecoObjYOLO;
std::vector<vision_msgs::VisionObject> JustinaVision::lastObjRecoYOLO;
//Sevices for line finding
ros::ServiceClient JustinaVision::cltFindLines;
//Service for find plane
ros::ServiceClient JustinaVision::cltFindPlane;
ros::ServiceClient JustinaVision::cltFindTable;
//Service for find vacant plane
ros::ServiceClient JustinaVision::cltFindVacantPlane;
//Services for thermal camera
ros::ServiceClient JustinaVision::cltGetAngle;
//Members for detect hand in front of gripper
ros::Publisher JustinaVision::pubStartHandFrontDetectBB;
ros::Publisher JustinaVision::pubStopHandFrontDetectBB;
ros::Subscriber JustinaVision::subHandFrontDetectBB;
bool JustinaVision::isHandFrontDetectedBB = false;
ros::Publisher JustinaVision::pubStartHandNearestDetectBB;
ros::Publisher JustinaVision::pubStopHandNearestDetectBB;
ros::Subscriber JustinaVision::subHandNearestDetectBB;
geometry_msgs::Point32 JustinaVision::lastHandNearestDetectedBB;
bool JustinaVision::isHandNearestDetectedBB = false;
ros::ServiceClient JustinaVision::srvTrainObject;
ros::ServiceClient JustinaVision::srvTrainObjectByHeight;
ros::Publisher JustinaVision::pubMove_base_train_vision;
//Members for detect gripper pos
ros::ServiceClient JustinaVision::cltGripperPos;
ros::ServiceClient JustinaVision::cltCubesSeg;
ros::ServiceClient JustinaVision::cltCutlerySeg;
ros::ServiceClient JustinaVision::cltGetTray;
ros::ServiceClient JustinaVision::cltGetDishwasher;

bool JustinaVision::setNodeHandle(ros::NodeHandle* nh)
{
    if(JustinaVision::is_node_set)
        return true;
    if(nh == 0)
        return false;

    std::cout << "JustinaVision.->Setting ros node..." << std::endl;
    //Members for operating pano maker
    JustinaVision::pubTakePanoMaker = nh->advertise<std_msgs::Empty>("/vision/pano_maker/take_image", 1);
    JustinaVision::pubClearPanoMaker = nh->advertise<std_msgs::Empty>("/vision/pano_maker/clear_images", 1);
    JustinaVision::pubMakePanoMaker = nh->advertise<std_msgs::Empty>("/vision/pano_maker/make_panoramic", 1);
    JustinaVision::subPanoImage = nh->subscribe("/vision/pano_maker/panoramic_image", 1, &callbackPanoRecived);
    JustinaVision::panoImageRecived = false;
    //Members for operating skeleton finder
    JustinaVision::pubSktEnableRecog = nh->advertise<std_msgs::Bool>("/vision/skeleton_finder/enable_tracking", 1);
    JustinaVision::subGestures = nh->subscribe("/vision/gesture_recog_skeleton/gesture_recog", 1, &JustinaVision::callbackGestures);
    JustinaVision::subSkeletons = nh->subscribe("/vision/openpose/skeleton_recog", 1, &JustinaVision::callbackSkeletons);
    JustinaVision::subLeftHandPositions = nh->subscribe("/vision/gesture_recog_skeleton/left_hand_pos", 1, &JustinaVision::callbackLeftHandPositions);
    JustinaVision::subRightHandPositions = nh->subscribe("/vision/gesture_recog_skeleton/right_hand_pos", 1, &JustinaVision::callbackRightHandPositions);
    //Members for operating face recognizer
    JustinaVision::pubStartFaceDetection = nh->advertise<std_msgs::Bool>("/vision/face_recognizer/start_detect", 1);
    JustinaVision::pubStartFaceRecognition = nh->advertise<std_msgs::Bool>("/vision/face_recognizer/start_recog", 1);
    JustinaVision::pubStartFaceRecognition2D = nh->advertise<std_msgs::Bool>("/vision/face_recognizer/start_recog_2D", 1);
    JustinaVision::pubSetIdFaceRecognition = nh->advertise<std_msgs::String>("/vision/face_recognizer/set_id_face_recognizer", 1);
    JustinaVision::pubTrainerFaces = nh->advertise<vision_msgs::VisionFaceTrainObject>("/vision/face_recognizer/trainer_faces", 1);
    JustinaVision::pubClearFacesDB = nh->advertise<std_msgs::Empty>("/vision/face_recognizer/clear_faces", 1);
    JustinaVision::pubClearFacesDBByID = nh->advertise<std_msgs::String>("/vision/face_recognizer/clear_face_id", 1);
    /*JustinaVision::pubTrainFace = nh->advertise<std_msgs::String>("/vision/face_recognizer/run_face_trainer", 1);
    JustinaVision::pubRecFace = nh->advertise<std_msgs::Empty>("/vision/face_recognizer/run_face_recognizer", 1);
    JustinaVision::pubRecFaceByID = nh->advertise<std_msgs::String>("/vision/face_recognizer/run_face_recognizer_id", 1);
    JustinaVision::subTrainer = nh->subscribe("/vision/face_recognizer/trainer_result", 1, &JustinaVision::callbackTrainer);*/
    JustinaVision::subFaces = nh->subscribe("/vision/face_recognizer/faces", 1, &JustinaVision::callbackFaces);
    JustinaVision::cltDetectPanoFaces = nh->serviceClient<vision_msgs::GetFacesFromImage>("/vision/face_recognizer/detect_pano_faces");
    JustinaVision::cltDetectFaces = nh->serviceClient<vision_msgs::FaceRecognition>("/vision/face_recognizer/detect_faces");
    JustinaVision::cltDetectWaving = nh->serviceClient<vision_msgs::FaceRecognition>("/vision/face_recognizer/detect_waving");
    JustinaVision::cltFaceRecognition = nh->serviceClient<vision_msgs::FaceRecognition>("/vision/face_recognizer/face_recognition");
    JustinaVision::cltFaceRecognition2D = nh->serviceClient<vision_msgs::FaceRecognition>("/vision/face_recognizer/face_recognition_2D");
    //Members for operation of thermal camera
    JustinaVision::pubStartThermalCamera = nh->advertise<std_msgs::Empty>("/vision/thermal_vision/start_video", 1);
    JustinaVision::pubStopThermalCamera = nh->advertise<std_msgs::Empty>("/vision/thermal_vision/stop_video", 1);
    //Members for operation of qr reader
    JustinaVision::pubQRReaderStart = nh->advertise<std_msgs::Bool>("/vision/qr/start_qr", 1);
    //Services for getting point cloud
    JustinaVision::cltGetRgbdWrtKinect = nh->serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_kinect");
    JustinaVision::cltGetRgbdWrtRobot = nh->serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot");
    //Detect objects
    JustinaVision::cltDetectObjects         = nh->serviceClient<vision_msgs::DetectObjects>("/vision/obj_reco/det_objs");
    JustinaVision::cltDetectAllObjects      = nh->serviceClient<vision_msgs::DetectObjects>("/vision/obj_reco/det_all_objs");
    JustinaVision::cltDetectAllObjectsVot      = nh->serviceClient<vision_msgs::DetectObjects>("/vision/obj_reco/vot_objs");
    JustinaVision::cltDetecObjectsYOLO      = nh->serviceClient<vision_msgs::DetectObjects>("/vision/obj_reco/det_objs_YOLO");
    JustinaVision::pubObjStartWin           = nh->advertise<std_msgs::Bool>("/vision/obj_reco/enableDetectWindow", 1);
    JustinaVision::pubObjStopWin            = nh->advertise<std_msgs::Bool>("/vision/obj_reco/enableDetectWindow", 0);
    JustinaVision::pubObjStartRecog         = nh->advertise<std_msgs::Bool>("/vision/obj_reco/enableRecognizeTopic", 1);
    JustinaVision::pubObjStopRecog          = nh->advertise<std_msgs::Bool>("/vision/obj_reco/enableRecognizeTopic", 0);
    JustinaVision::pubEnableObjsDetectYOLO  = nh->advertise<std_msgs::Bool>("/vision/obj_reco/enable_det_objs_YOLO", 1);
    JustinaVision::srvTrainObject           = nh->serviceClient<vision_msgs::TrainObject>("/vision/obj_reco/trainObject");
    JustinaVision::srvTrainObjectByHeight   = nh->serviceClient<vision_msgs::TrainObject>("/vision/obj_reco/train_byHeight");
    JustinaVision::pubMove_base_train_vision = nh->advertise<std_msgs::String>("/hardware/obj_train_base", 1);
    JustinaVision::subGetRecoObjYOLO        = nh->subscribe("/vision/obj_reco/get_det_objs_YOLO", 1, &JustinaVision::callbackGetRecoObjYOLO); 
    //Sevices for line finding
    JustinaVision::cltFindLines = nh->serviceClient<vision_msgs::FindLines>("/vision/line_finder/find_lines_ransac");
    //Service for find plane
    JustinaVision::cltFindPlane = nh->serviceClient<vision_msgs::FindPlane>("/vision/geometry_finder/findPlane");
    JustinaVision::cltFindTable = nh->serviceClient<vision_msgs::FindPlane>("/vision/geometry_finder/findTable");
    //Service for find vacant plane
    JustinaVision::cltFindVacantPlane = nh->serviceClient<vision_msgs::FindPlane>("/vision/geometry_finder/vacantPlane");
    //Services for get angle of thermal camera
    JustinaVision::cltGetAngle = nh->serviceClient<vision_msgs::GetThermalAngle>("/vision/thermal_angle");
    JustinaVision::is_node_set = true;
    //Detect hand in front of gripper
    JustinaVision::pubStartHandFrontDetectBB = nh->advertise<geometry_msgs::Point32>("/vision/hand_detect_in_bb/start_hand_front_recog", 1);
    JustinaVision::pubStopHandFrontDetectBB = nh->advertise<std_msgs::Empty>("/vision/hand_detect_in_bb/stop_hand_front_recog", 1);
    JustinaVision::subHandFrontDetectBB = nh->subscribe("/vision/hand_detect_in_bb/hand_in_front", 1, callbackHandFrontDetectBB);
    JustinaVision::pubStartHandNearestDetectBB = nh->advertise<std_msgs::Empty>("/vision/hand_detect_in_bb/start_nearest_recog", 1);
    JustinaVision::pubStopHandNearestDetectBB = nh->advertise<std_msgs::Empty>("/vision/hand_detect_in_bb/stop_nearest_recog", 1);
    JustinaVision::subHandNearestDetectBB = nh->subscribe("/vision/hand_detect_in_bb/hand_nearest_detect", 1, callbackHandNearestDetectBB);
    //Services for detect gripper pos
    JustinaVision::cltGripperPos = nh->serviceClient<vision_msgs::DetectGripper>("/vision/obj_reco/gripper");
    //Services for segment cubes
    JustinaVision::cltCubesSeg = nh->serviceClient<vision_msgs::GetCubes>("/vision/cubes_segmentation/cubes_seg");
    //Services for segment cutlery
    JustinaVision::cltCutlerySeg = nh->serviceClient<vision_msgs::GetCubes>("/vision/cubes_segmentation/cutlery_seg");
    JustinaVision::cltGetTray = nh ->serviceClient<vision_msgs::SRV_DetectPlasticTrayZones>("/vision/obj_reco/plastic_tray");
    JustinaVision::cltGetDishwasher = nh ->serviceClient<vision_msgs::SRV_FindDishwasher>("/vision/obj_reco/dishwasher");

    return true;
}

//Methods for pano maker
void JustinaVision::takePano(){
    std_msgs::Empty msg;
    JustinaVision::pubTakePanoMaker.publish(msg);
}
    
void JustinaVision::clearPano(){
    std_msgs::Empty msg;
    JustinaVision::panoImageRecived = false;
    JustinaVision::pubClearPanoMaker.publish(msg);
}

void JustinaVision::makePano(){
    std_msgs::Empty msg;
    JustinaVision::pubMakePanoMaker.publish(msg);
}

bool JustinaVision::isPanoImageRecived(){
    return JustinaVision::panoImageRecived;
}

sensor_msgs::Image JustinaVision::getLastPanoImage(){
    JustinaVision::panoImageRecived = false;
    return JustinaVision::lastImage;
}

//Methods for operating skeleton finder
void JustinaVision::startSkeletonFinding()
{
    JustinaVision::lastSkeletons.clear();
    JustinaVision::lastGestureRecog.clear();
    JustinaVision::lastLeftHandPos.clear();
    JustinaVision::lastRightHandPos.clear();
    std_msgs::Bool msg;
    msg.data = true;
    JustinaVision::pubSktEnableRecog.publish(msg);
}

void JustinaVision::stopSkeletonFinding()
{
    JustinaVision::lastSkeletons.clear();
    JustinaVision::lastGestureRecog.clear();
    JustinaVision::lastLeftHandPos.clear();
    JustinaVision::lastRightHandPos.clear();
    std_msgs::Bool msg;
    msg.data = false;
    JustinaVision::pubSktEnableRecog.publish(msg);
}

void JustinaVision::getLastSkeletons(std::vector<vision_msgs::Skeleton> &skeletons){
    skeletons = JustinaVision::lastSkeletons;
    JustinaVision::lastSkeletons.clear();
}

void JustinaVision::getLastGesturesRecognize(std::vector<vision_msgs::GestureSkeleton> &gestures){
    gestures = JustinaVision::lastGestureRecog;
    JustinaVision::lastGestureRecog.clear();
}

void JustinaVision::getLastLeftHandPositions(std::vector<geometry_msgs::Point> &leftHandPositions){
    leftHandPositions = JustinaVision::lastLeftHandPos;
    JustinaVision::lastLeftHandPos.clear();
}

void JustinaVision::getLastRightHandPositions(std::vector<geometry_msgs::Point> &rightHandPositions){
    rightHandPositions = JustinaVision::lastRightHandPos;
    JustinaVision::lastRightHandPos.clear();
}

//Methods for operating face recognizer
void JustinaVision::startFaceDetection(bool enable){
    std::cout << "JustinaVision.->Starting face detection 2D. " << std::endl;
    std_msgs::Bool msg;
    msg.data = enable;
    JustinaVision::pubStartFaceDetection.publish(msg);
}

void JustinaVision::startFaceRecognition(bool enable)
{
    std::cout << "JustinaVision.->Starting face recognition. " << std::endl;
    std_msgs::Bool msg;
    msg.data = enable;
    JustinaVision::pubStartFaceRecognition.publish(msg);
}

void JustinaVision::startFaceRecognition2D(bool enable)
{
    std::cout << "JustinaVision.->Starting face recognition 2D. " << std::endl;
    std_msgs::Bool msg;
    msg.data = enable;
    JustinaVision::pubStartFaceRecognition2D.publish(msg);
}
    
void JustinaVision::setIdFaceRecognition(std::string id){
    std::cout << "JustinaVision.->Set id face recognition. " << std::endl;
    std_msgs::String msg;
    msg.data = id;
    JustinaVision::pubSetIdFaceRecognition.publish(msg);
}

/*void JustinaVision::facRecognize()
{
    std::cout << "JustinaVision.->Starting face recognition without id" << std::endl;
    std_msgs::Empty msg;
    JustinaVision::pubRecFace.publish(msg);
}

void JustinaVision::facRecognize(std::string id)
{
    std::cout << "JustinaVision.->Starting face recognition of id: " << id << std::endl;
    std_msgs::String msg;
    msg.data = id;
    JustinaVision::pubRecFaceByID.publish(msg);
}

void JustinaVision::facTrain(std::string id)
{
    std::cout << "JustinaVision.->Training face with id: " << id << std::endl;
    std_msgs::String msg;
    msg.data = id;
    JustinaVision::pubTrainFace.publish(msg);
}*/

void JustinaVision::faceTrain(std::string id, int numOfFrames)
{
    std::cout << "JustinaVision.->Training face with id " << id << " with " << numOfFrames << " frames." <<  std::endl;
    vision_msgs::VisionFaceTrainObject msg;
    msg.id = id;
    msg.frames = numOfFrames;
    JustinaVision::pubTrainerFaces.publish(msg);
}

void JustinaVision::facClearByID(std::string id)
{
    std::cout << "JustinaVision.->Clearing face data base entry: " << id << std::endl;
    std_msgs::String msg;
    msg.data = id;
    JustinaVision::pubClearFacesDBByID.publish(msg);
}

void JustinaVision::facClearAll()
{
    std::cout << "JustinaVision.->Clearing all data base of known faces. " << std::endl;
    std_msgs::Empty msg;
    JustinaVision::pubClearFacesDB.publish(msg);
}

bool JustinaVision::getMostConfidentFace(std::string& id, float& posX, float& posY, float& posZ, float& confidence, int& gender, bool& isSmiling)
{
    if(JustinaVision::lastRecognizedFaces.size() < 1)
        return false;

    vision_msgs::VisionFaceObject bestFace;
    int bestFaceIdx = -1;
    float bestConfidence = -1;
    for(size_t i=0; i < JustinaVision::lastRecognizedFaces.size(); i++)
        if(JustinaVision::lastRecognizedFaces[i].confidence > bestConfidence)
        {
            bestConfidence = JustinaVision::lastRecognizedFaces[i].confidence;
            bestFaceIdx = int(i);
        }

    if(bestFaceIdx >= 0)
        bestFace = JustinaVision::lastRecognizedFaces[bestFaceIdx];
    else
        return false;

    id = bestFace.id;
    posX = bestFace.face_centroid.x;
    posY = bestFace.face_centroid.y;
    posZ = bestFace.face_centroid.z;
    confidence = bestFace.confidence;
    gender = bestFace.gender;
    isSmiling = bestFace.smile;
    JustinaVision::lastRecognizedFaces.clear();
    return true;
}

bool JustinaVision::getLastRecognizedFaces(std::vector<vision_msgs::VisionFaceObject>& faces)
{
    if(JustinaVision::lastRecognizedFaces.size() < 1)
        return false;

    faces.clear();
    for(size_t i=0; i < JustinaVision::lastRecognizedFaces.size(); i++)
        faces.push_back(JustinaVision::lastRecognizedFaces[i]);

    JustinaVision::lastRecognizedFaces.clear();
    return true;
}

/*int JustinaVision::getLastTrainingResult()
{
    return JustinaVision::lastFaceRecogResult;
}*/

vision_msgs::VisionFaceObjects JustinaVision::getRecogFromPano(sensor_msgs::Image image){
    vision_msgs::VisionFaceObjects faces;
    vision_msgs::GetFacesFromImage srv;
    srv.request.panoramic_image = image;
    if(cltDetectPanoFaces.call(srv)){
        faces = srv.response.faces;
        std::cout << "Detect " << faces.recog_faces.size() << " faces" << std::endl;
    }
    else
        std::cout << "Failed in call service GetFacesFromImage" << std::endl;
    return faces;
}

vision_msgs::VisionFaceObjects JustinaVision::getFaces(){
    vision_msgs::VisionFaceObjects faces;
    vision_msgs::FaceRecognition srv;
    if(cltDetectFaces.call(srv)){
        faces = srv.response.faces;
        std::cout << "Detect " << faces.recog_faces.size() << " faces" << std::endl;
    }
    else
        std::cout << "Failed in call service FaceRecognition" << std::endl;
    return faces;

}

vision_msgs::VisionFaceObjects JustinaVision::getFaceRecognition(std::string id){
    vision_msgs::VisionFaceObjects faces;
    vision_msgs::FaceRecognition srv;
    srv.request.id = id;
    if(cltFaceRecognition.call(srv)){
        faces = srv.response.faces;
        std::cout << "Detect " << faces.recog_faces.size() << " faces" << std::endl;
    }
    else
        std::cout << "Failed in call service FaceRecognition" << std::endl;
    return faces;
}

vision_msgs::VisionFaceObjects JustinaVision::getFaceRecognition2D(std::string id){
    vision_msgs::VisionFaceObjects faces;
    vision_msgs::FaceRecognition srv;
    srv.request.id = id;
    if(cltFaceRecognition2D.call(srv)){
        faces = srv.response.faces;
        std::cout << "Detect " << faces.recog_faces.size() << " faces" << std::endl;
    }
    else
        std::cout << "Failed in call service FaceRecognition" << std::endl;
    return faces;
}

std::vector<vision_msgs::VisionRect> JustinaVision::detectWaving(){
    vision_msgs::FindWaving srv;
    std::vector<vision_msgs::VisionRect> wavingRect;
    if(cltDetectWaving.call(srv)){
        wavingRect = srv.response.bounding_box;
        std::cout << "Detect " << wavingRect.size() << " waving person" << std::endl;
    }
    else
        std::cout << "Failed in call service Waving detect person" << std::endl;
    return wavingRect;
}

//Object detection
void JustinaVision::startObjectFinding()
{
    std_msgs::Bool msg;
    msg.data = true;
    JustinaVision::pubObjStartRecog.publish(msg);
}

void JustinaVision::stopObjectFinding()
{
    std_msgs::Bool msg;
    msg.data = false;
    JustinaVision::pubObjStopRecog.publish(msg);
}

void JustinaVision::startObjectFindingWindow()
{
    std_msgs::Bool msg;
    msg.data = true;
    JustinaVision::pubObjStartWin.publish(msg);
}

void JustinaVision::stopObjectFindingWindow()
{
    std_msgs::Bool msg;
    msg.data = false;
    JustinaVision::pubObjStopWin.publish(msg);
}


bool JustinaVision::detectObjects(std::vector<vision_msgs::VisionObject>& recoObjList, bool saveFiles)
{
    std::cout << "JustinaVision.->Trying to detect objects... " << std::endl;
    vision_msgs::DetectObjects srv;
    srv.request.saveFiles = saveFiles;
    if(!cltDetectObjects.call(srv))
    {
        std::cout << std::endl << "Justina::Vision can't detect anything" << std::endl << std::endl;
        return false;
    }
    recoObjList=srv.response.recog_objects;
    if(recoObjList.size() < 1)
    {
        std::cout << std::endl << "Justina::Vision can't detect anything" << std::endl << std::endl;
        return false;
    }
    std::cout << "JustinaVision.->Detected " << int(recoObjList.size()) << " objects" << std::endl;
    return true;
}

bool JustinaVision::detectAllObjects(std::vector<vision_msgs::VisionObject>& recoObjList, bool saveFiles)
{
    std::cout << "JustinaVision.->Trying to detect objects... " << std::endl;
    vision_msgs::DetectObjects srv;
    srv.request.saveFiles = saveFiles;
    if(!cltDetectAllObjects.call(srv))
    {
        std::cout << std::endl << "Justina::Vision can't detect anything" << std::endl << std::endl;
        return false;
    }
    recoObjList=srv.response.recog_objects;
    if(recoObjList.size() < 1)
    {
        std::cout << std::endl << "Justina::Vision can't detect anything" << std::endl << std::endl;
        return false;
    }
    std::cout << "JustinaVision.->Detected " << int(recoObjList.size()) << " objects" << std::endl;
    return true;
}

bool JustinaVision::detectAllObjectsVot(std::vector<vision_msgs::VisionObject>& recoObjList, sensor_msgs::Image &image, int iterations){
    vision_msgs::DetectObjects srv;
    srv.request.iterations = iterations;
    if(!cltDetectAllObjectsVot.call(srv))
    {
        std::cout << std::endl << "Justina::Vision can't detect anything" << std::endl << std::endl;
        return -1;
    }
    recoObjList = srv.response.recog_objects;
    image = srv.response.image;
    if(recoObjList.size() < 1)
    {
        std::cout << std::endl << "Justina::Vision can't detect anything" << std::endl << std::endl;
        return false;
    }
    std::cout << "JustinaVision.->Detected " << int(recoObjList.size()) << " objects" << std::endl;
    return true;
}

void JustinaVision::enableDetectObjsYOLO(bool enable){
    std::cout << "JustinaVision.->Enable to detect objects YOLO... " << std::endl;
    std_msgs::Bool msg;
    msg.data = enable;
    pubEnableObjsDetectYOLO.publish(msg);
}
    
//Action client for YOLO object recog
bool JustinaVision::detectObjectsYOLO(std::vector<vision_msgs::VisionObject>& yoloObjects){
    std::cout << "JustinaVision.->Trying to detect objects YOLO... " << std::endl;
    vision_msgs::DetectObjects srv;
    if(!cltDetecObjectsYOLO.call(srv))
    {
        std::cout << std::endl << "Justina::Vision can't detect anything with YOLO" << std::endl << std::endl;
        return false;
    }
    yoloObjects = srv.response.recog_objects;
    /*recoObjList=srv.response.recog_objects;
    if(recoObjList.size() < 1)
    {
        std::cout << std::endl << "Justina::Vision can't detect anything" << std::endl << std::endl;
        return false;
    }
    std::cout << "JustinaVision.->Detected " << int(recoObjList.size()) << " objects" << std::endl;*/
    return true;
}

void JustinaVision::callbackGetRecoObjYOLO(const vision_msgs::VisionObjectList::ConstPtr& msg){
    JustinaVision::lastObjRecoYOLO = msg->ObjectList;
}

void JustinaVision::getObjectsYOLO(std::vector<vision_msgs::VisionObject>& yoloObjects){
    yoloObjects = JustinaVision::lastObjRecoYOLO;
    JustinaVision::lastObjRecoYOLO.clear();
}

//Methods for move the train object and move the tranining base
void JustinaVision::trainObject(const std::string name)
{
    vision_msgs::TrainObject srv;
    srv.request.name = name;
    srvTrainObject.call(srv);
}

void JustinaVision::trainObjectByHeight(const std::string name)
{
    vision_msgs::TrainObject srv;
    srv.request.name = name;
    srvTrainObjectByHeight.call(srv);
}

void JustinaVision::moveBaseTrainVision(const std_msgs::String& msg)
{
    pubMove_base_train_vision.publish(msg);
}

//Methods for line finding
bool JustinaVision::findLine(float& x1, float& y1, float& z1, float& x2, float& y2, float& z2)
{
    std::cout << "JustinaVision.->Trying to find a straight line." << std::endl;
    point_cloud_manager::GetRgbd srvGetCloud;
    if(!JustinaVision::cltGetRgbdWrtRobot.call(srvGetCloud))
    {
        std::cout << "JustinaVision.->Cannot find line: cannot get point cloud :'(" << std::endl;
        return false;
    }

    vision_msgs::FindLines srvFindLines;//It really finds only one line
    srvFindLines.request.point_cloud = srvGetCloud.response.point_cloud;

    if(!JustinaVision::cltFindLines.call(srvFindLines) || srvFindLines.response.lines.size() < 2)
    {
        std::cout << "JustinaVision.->Cannot find lines. " << std::endl;
        return false;
    }

    x1 = srvFindLines.response.lines[0].x;
    y1 = srvFindLines.response.lines[0].y;
    z1 = srvFindLines.response.lines[0].z;
    x2 = srvFindLines.response.lines[1].x;
    y2 = srvFindLines.response.lines[1].y;
    z2 = srvFindLines.response.lines[1].z;

    return true;
}

//Methods for plane findinig
bool JustinaVision::findPlane(){
    std::cout << "JustinaVision.->Trying to find a plane" << std::endl;
    vision_msgs::FindPlane fp;
    fp.request.name = "";
    if(!JustinaVision::cltFindPlane.call(fp)){
        std::cout << "JustinaVision.->Cannot find a plane" << std::endl;
        return false;
    }
    std::cout << "JustinaVision.->Find a plane" << std::endl;
    return true;
}

//Methods for find plane
bool JustinaVision::findVacantPlane(std::vector<float>& vacantPlane, std::vector<int>& inliersOnPlane)
{
    std::cout << "JustinaVision.->Trying to find a vacantPlane" << std::endl;
    vision_msgs::FindPlane fp;
    fp.request.name="";
    if(!JustinaVision::cltFindVacantPlane.call(fp))
    {
        std::cout << "JustinaVision.->Cannot a vacantPlane" << std::endl;
        return false;
    }

    for (int i = 0; i < (int)fp.response.centroidFreeSpace.size(); ++i)
    {
        vacantPlane.push_back(fp.response.centroidFreeSpace[i].x);
        vacantPlane.push_back(fp.response.centroidFreeSpace[i].y);
        vacantPlane.push_back(fp.response.centroidFreeSpace[i].z);
        inliersOnPlane.push_back(fp.response.inliers[i].data);
    }
    return true;
}

bool JustinaVision::findTable(std::vector<float>& nearestPoint)
{
    std::cout << "JustinaVision.->Trying to find a table" << std::endl;

    vision_msgs::FindPlane fp;
    fp.request.name="";

    nearestPoint.clear();

    if(!JustinaVision::cltFindTable.call(fp))
    {
        std::cout << "JustinaVision.->Cannot a table" << std::endl;
        return false;
    }
    else
    {
        nearestPoint.push_back(fp.response.nearestPoint.x);
        nearestPoint.push_back(fp.response.nearestPoint.y);
        nearestPoint.push_back(fp.response.nearestPoint.z);    
    }
    

    return true;
}

//Methods for Gripper Pos
bool JustinaVision::getGripperPos(geometry_msgs::Point& gripperPos)
{
    std::cout << "JustinaVision.-> Trying to get gripper position whith vision feedback" << std::endl;
    vision_msgs::DetectGripper srvDetectGripper;

    if(!JustinaVision::cltGripperPos.call(srvDetectGripper))
    {
        std::cout << "JustinaVision.->Error trying to call gripper pos service" << std::endl;
        return false;
    }

    gripperPos = srvDetectGripper.response.gripper_position;

    return true;
}


//Methods for the thermal camera
void JustinaVision::startThermalCamera(){
	std_msgs::Empty msg;
	JustinaVision::pubStartThermalCamera.publish(msg);
}

void JustinaVision::stopThermalCamera(){
	std_msgs::Empty msg;
	JustinaVision::pubStopThermalCamera.publish(msg);
}

float JustinaVision::getAngleTC(){
	vision_msgs::GetThermalAngle srv;
	float angle;
	JustinaVision::cltGetAngle.call(srv);
	angle=srv.response.th_angle;
	return angle;
}

//Methods for the qr reader
void JustinaVision::JustinaVision::startQRReader(){
    std_msgs::Bool msg;
    msg.data = true;
    pubQRReaderStart.publish(msg);
}

void JustinaVision::stopQRReader(){
    std_msgs::Bool msg;
    msg.data = false;
    pubQRReaderStart.publish(msg);
}

void JustinaVision::callbackFaces(const vision_msgs::VisionFaceObjects::ConstPtr& msg)
{
    JustinaVision::lastRecognizedFaces = msg->recog_faces;
}

void JustinaVision::callbackTrainer(const std_msgs::Int32::ConstPtr& msg)
{
    JustinaVision::lastFaceRecogResult = msg->data;
}

//Methods for the hand detect in front of gripper
void JustinaVision::startHandFrontDetectBB(float x, float y, float z)
{
	geometry_msgs::Point32 msg;
	msg.x = x;
	msg.y = y;
	msg.z = z;
	pubStartHandFrontDetectBB.publish(msg);
}

void JustinaVision::stopHandFrontDetectBB()
{
	std_msgs::Empty msg;
	pubStopHandFrontDetectBB.publish(msg);
}

bool JustinaVision::getDetectionHandFrontBB()
{
	return JustinaVision::isHandFrontDetectedBB;
}

bool JustinaVision::getDetectionHandNearestBB(geometry_msgs::Point32& nearestPoint){
    if(JustinaVision::isHandNearestDetectedBB){
        nearestPoint = lastHandNearestDetectedBB;
        JustinaVision::isHandNearestDetectedBB = false;
        return true;
    }
    return false;
}

//callbacks for pano maker
void JustinaVision::callbackPanoRecived(const sensor_msgs::Image msg){
    JustinaVision::panoImageRecived = true;
    JustinaVision::lastImage = msg;
}


//callbacks for the hand detect in front of gripper
void JustinaVision::callbackHandFrontDetectBB(const std_msgs::Bool::ConstPtr& msg)
{
	JustinaVision::isHandFrontDetectedBB = msg->data;
}

void JustinaVision::callbackHandNearestDetectBB(const geometry_msgs::Point32 msg){
    JustinaVision::lastHandNearestDetectedBB = msg;
    isHandNearestDetectedBB = true;
}

//calbacks for the skeletons and gestures
void JustinaVision::callbackSkeletons(const vision_msgs::Skeletons::ConstPtr& msg){
    JustinaVision::lastSkeletons.clear();
    for(int i = 0; i < msg->skeletons.size(); i++)
        JustinaVision::lastSkeletons.push_back(msg->skeletons[i]);
}

void JustinaVision::callbackGestures(const vision_msgs::GestureSkeletons::ConstPtr& msg){
    JustinaVision::lastGestureRecog.clear();
    for(int i = 0; i < msg->recog_gestures.size(); i++)
        JustinaVision::lastGestureRecog.push_back(msg->recog_gestures[i]);
}

void JustinaVision::callbackLeftHandPositions(const vision_msgs::HandSkeletonPos leftHandPositions){
    JustinaVision::lastLeftHandPos.clear();
    for(int i = 0; i < leftHandPositions.hands_position.size(); i++)
        JustinaVision::lastLeftHandPos.push_back(leftHandPositions.hands_position[i]);
}

void JustinaVision::callbackRightHandPositions(const vision_msgs::HandSkeletonPos rightHandPositions){
    JustinaVision::lastRightHandPos.clear();
    for(int i = 0; i < rightHandPositions.hands_position.size(); i++)
        JustinaVision::lastRightHandPos.push_back(rightHandPositions.hands_position[i]);
}

//Methods for cube segmentation
bool JustinaVision::getCubesSeg(vision_msgs::CubesSegmented& cubes)
{
    std::cout << "JustinaVision.-> Trying to get Cubes Segmented" << std::endl;
    vision_msgs::GetCubes srvSegmentedCubes;
    srvSegmentedCubes.request.cubes_input=cubes;

    if(!JustinaVision::cltCubesSeg.call(srvSegmentedCubes))
    {
        std::cout << "JustinaVision.->Error trying to call segment cubes service" << std::endl;
        return false;
    }

    cubes = srvSegmentedCubes.response.cubes_output;

    return true;
}

//Methods for cutlery segmentation
bool JustinaVision::getCutlerySeg(vision_msgs::CubesSegmented& cutleries)
{
    std::cout << "JustinaVision.-> Trying to get cutleries Segmented" << std::endl;
    vision_msgs::GetCubes srvSegmentedCutleries;
    srvSegmentedCutleries.request.cubes_input=cutleries;

    if(!JustinaVision::cltCutlerySeg.call(srvSegmentedCutleries))
    {
        std::cout << "JustinaVision.->Error trying to call segment cutleriess service" << std::endl;
        return false;
    }

    cutleries = srvSegmentedCutleries.response.cubes_output;

    return true;
}

bool JustinaVision::isStillOnTable(vision_msgs::Cube my_cutlery)
{
    std::cout << "JustinaVision.-> Trying to state if the object is still on the table" << std::endl;
    vision_msgs::CubesSegmented cutleries;
    cutleries.recog_cubes.resize(1);
    cutleries.recog_cubes[0].color = my_cutlery.color;

    vision_msgs::GetCubes srvSegmentedCutleries;
    srvSegmentedCutleries.request.cubes_input=cutleries;
    bool stillontable = false;

    if(!JustinaVision::cltCutlerySeg.call(srvSegmentedCutleries))
    {
        std::cout << "JustinaVision.->Error trying to call segment cutleries service" << std::endl;
        return false;
    }

    cutleries = srvSegmentedCutleries.response.cubes_output;


    std::cout << "JustinaVision.-> searching the object on the table...." <<std::endl;
    if(cutleries.recog_cubes[0].detected_cube == true && cutleries.recog_cubes[0].color == my_cutlery.color &&
       cutleries.recog_cubes[0].cube_centroid.x <= my_cutlery.maxPoint.x && cutleries.recog_cubes[0].cube_centroid.x >= my_cutlery.minPoint.x &&
       cutleries.recog_cubes[0].cube_centroid.y <= my_cutlery.maxPoint.y && cutleries.recog_cubes[0].cube_centroid.y >= my_cutlery.minPoint.y &&
       cutleries.recog_cubes[0].cube_centroid.z <= my_cutlery.maxPoint.z && cutleries.recog_cubes[0].cube_centroid.z >= my_cutlery.minPoint.z){
       
        std::cout << "JustinaVision.-> the object is still on the table" <<std::endl;
        stillontable = true; 
    }

    else{
        std::cout << "JustinaVision.-> the object is NOT on the table anymore" <<std::endl;
        stillontable = false;     
    }

    return stillontable;

}

bool JustinaVision::getTray(vision_msgs::MSG_VisionPlasticTray &tray)
{
    std::cout << "JustinaVision.-> Trying to compute the tray position" << std::endl;
    vision_msgs::SRV_DetectPlasticTrayZones srv;    
    
    if(!JustinaVision::cltGetTray.call(srv))
    {
        std::cout << "JustinaVision.->Error trying to call detect plastic tray zones" << std::endl;
        return false;
    }

    tray = srv.response.plastic_tray_zones;
    return true;
}

bool JustinaVision::getDishwasher(vision_msgs::MSG_VisionDishwasher &dishwasher)
{
    std::cout << "JustinaVision.->Trying to compute the dishwasher position" << std::endl;
    vision_msgs::SRV_FindDishwasher srv;

    if(!JustinaVision::cltGetDishwasher.call(srv)){
        std::cout << "JustinaVision.->Error trying to call detect dishwasher zone" << std::endl;
        return false;
    }
    dishwasher = srv.response.dishwasher;
    return true;
}
