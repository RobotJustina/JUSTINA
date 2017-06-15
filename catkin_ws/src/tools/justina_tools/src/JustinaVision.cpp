#include "justina_tools/JustinaVision.h"

bool JustinaVision::is_node_set = false;
//Members for operating skeleton finder
ros::Publisher JustinaVision::pubSktStartRecog;
ros::Publisher JustinaVision::pubSktStopRecog;
//Members for operating face recognizer
ros::Publisher JustinaVision::pubFacStartRecog;
ros::Publisher JustinaVision::pubFacStartRecogOld;
ros::Publisher JustinaVision::pubFacStopRecog;
ros::Publisher JustinaVision::pubTrainFace;
ros::Publisher JustinaVision::pubTrainFaceNum;
ros::Publisher JustinaVision::pubRecFace;
ros::Publisher JustinaVision::pubRecFaceByID;
ros::Publisher JustinaVision::pubClearFacesDB;
ros::Publisher JustinaVision::pubClearFacesDBByID;
ros::Subscriber JustinaVision::subFaces;
ros::Subscriber JustinaVision::subTrainer;
std::vector<vision_msgs::VisionFaceObject> JustinaVision::lastRecognizedFaces;
int JustinaVision::lastFaceRecogResult = 0;
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
ros::Publisher JustinaVision::pubObjStartRecog;
ros::Publisher JustinaVision::pubObjStopRecog;
ros::Publisher JustinaVision::pubObjStartWin;
ros::Publisher JustinaVision::pubObjStopWin;
//Sevices for line finding
ros::ServiceClient JustinaVision::cltFindLines;
//Service for find plane
ros::ServiceClient JustinaVision::cltFindPlane;
//Service for find vacant plane
ros::ServiceClient JustinaVision::cltFindVacantPlane;
//Services for thermal camera
ros::ServiceClient JustinaVision::cltGetAngle;
//Members for detect hand in front of gripper
ros::Publisher JustinaVision::pubStartHandDetectBB;
ros::Publisher JustinaVision::pubStopHandDetectBB;
ros::Subscriber JustinaVision::subHandDetectBB;
bool JustinaVision::isHandDetectedBB = false;
ros::ServiceClient JustinaVision::srvTrainObject;
ros::Publisher JustinaVision::pubMove_base_train_vision;

bool JustinaVision::setNodeHandle(ros::NodeHandle* nh)
{
    if(JustinaVision::is_node_set)
        return true;
    if(nh == 0)
        return false;

    std::cout << "JustinaVision.->Setting ros node..." << std::endl;
    JustinaVision::pubSktStartRecog = nh->advertise<std_msgs::Empty>("/vision/skeleton_finder/start_tracking", 1);
    JustinaVision::pubSktStopRecog = nh->advertise<std_msgs::Empty>("/vision/skeleton_finder/stop_tracking", 1);
    //Members for operating face recognizer
    JustinaVision::pubFacStartRecog = nh->advertise<std_msgs::Empty>("/vision/face_recognizer/start_recog", 1);
    JustinaVision::pubFacStartRecogOld = nh->advertise<std_msgs::Empty>("/vision/face_recognizer/start_recog_old", 1);
    JustinaVision::pubFacStopRecog = nh->advertise<std_msgs::Empty>("/vision/face_recognizer/stop_recog", 1);
    JustinaVision::pubTrainFace = nh->advertise<std_msgs::String>("/vision/face_recognizer/run_face_trainer", 1);
    JustinaVision::pubTrainFaceNum = nh->advertise<vision_msgs::VisionFaceTrainObject>("/vision/face_recognizer/run_face_trainer_frames", 1);
    JustinaVision::pubRecFace = nh->advertise<std_msgs::Empty>("/vision/face_recognizer/run_face_recognizer", 1);
    JustinaVision::pubRecFaceByID = nh->advertise<std_msgs::String>("/vision/face_recognizer/run_face_recognizer_id", 1);
    JustinaVision::pubClearFacesDB = nh->advertise<std_msgs::Empty>("/vision/face_recognizer/clearfacesdb", 1);
    JustinaVision::pubClearFacesDBByID = nh->advertise<std_msgs::String>("/vision/face_recognizer/clearfacesdbbyid", 1);
    JustinaVision::subFaces = nh->subscribe("/vision/face_recognizer/faces", 1, &JustinaVision::callbackFaces);
    JustinaVision::subTrainer = nh->subscribe("/vision/face_recognizer/trainer_result", 1, &JustinaVision::callbackTrainer);
    //Members for operation of thermal camera
    JustinaVision::pubStartThermalCamera = nh->advertise<std_msgs::Empty>("/vision/thermal_vision/start_video", 1);
    JustinaVision::pubStopThermalCamera = nh->advertise<std_msgs::Empty>("/vision/thermal_vision/stop_video", 1);
    //Members for operation of qr reader
    JustinaVision::pubQRReaderStart = nh->advertise<std_msgs::Bool>("/vision/qr/start_qr", 1);
    //Services for getting point cloud
    JustinaVision::cltGetRgbdWrtKinect = nh->serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_kinect");
    JustinaVision::cltGetRgbdWrtRobot = nh->serviceClient<point_cloud_manager::GetRgbd>("/hardware/point_cloud_man/get_rgbd_wrt_robot");
    //Detect objects
    JustinaVision::cltDetectObjects = nh->serviceClient<vision_msgs::DetectObjects>("/vision/obj_reco/det_objs");
    JustinaVision::cltDetectAllObjects = nh->serviceClient<vision_msgs::DetectObjects>("/vision/obj_reco/det_all_objs");
    JustinaVision::pubObjStartWin = nh->advertise<std_msgs::Bool>("/vision/obj_reco/enableDetectWindow", 1);
    JustinaVision::pubObjStopWin = nh->advertise<std_msgs::Bool>("/vision/obj_reco/enableDetectWindow", 0);
    JustinaVision::pubObjStartRecog = nh->advertise<std_msgs::Bool>("/vision/obj_reco/enableRecognizeTopic", 1);
    JustinaVision::pubObjStopRecog = nh->advertise<std_msgs::Bool>("/vision/obj_reco/enableRecognizeTopic", 0);
    //Sevices for line finding
    JustinaVision::cltFindLines = nh->serviceClient<vision_msgs::FindLines>("/vision/line_finder/find_lines_ransac");
    //Service for find plane
    JustinaVision::cltFindPlane = nh->serviceClient<vision_msgs::FindPlane>("/vision/geometry_finder/findPlane");
    //Service for find vacant plane
    JustinaVision::cltFindVacantPlane = nh->serviceClient<vision_msgs::FindPlane>("/vision/geometry_finder/vacantPlane");
    //Services for get angle of thermal camera
    JustinaVision::cltGetAngle = nh->serviceClient<vision_msgs::GetThermalAngle>("/vision/thermal_angle");
    JustinaVision::is_node_set = true;
    //Detect hand in front of gripper
    JustinaVision::pubStartHandDetectBB = nh->advertise<geometry_msgs::Point32>("/vision/hand_detect_in_bb/start_recog", 1);
    JustinaVision::pubStopHandDetectBB = nh->advertise<std_msgs::Empty>("/vision/hand_detect_in_bb/stop_recog", 1);
    JustinaVision::subHandDetectBB = nh->subscribe("/vision/hand_detect_in_bb/hand_in_front", 1, callbackHandDetectBB);
    JustinaVision::srvTrainObject = nh->serviceClient<vision_msgs::TrainObject>("/vision/obj_reco/trainObject");
    JustinaVision::pubMove_base_train_vision = nh->advertise<std_msgs::String>("/move_base_train_obj", 1);
    return true;
}

//Methods for operating skeleton finder
void JustinaVision::startSkeletonFinding()
{
    std_msgs::Empty msg;
    JustinaVision::pubSktStartRecog.publish(msg);
}

void JustinaVision::stopSkeletonFinding()
{
    std_msgs::Empty msg;
    JustinaVision::pubSktStopRecog.publish(msg);
}

//Methods for operating face recognizer
void JustinaVision::startFaceRecognition()
{
    std::cout << "JustinaVision.->Starting face recognition. " << std::endl;
    std_msgs::Empty msg;
    JustinaVision::pubFacStartRecog.publish(msg);
}

void JustinaVision::startFaceRecognitionOld()
{
    std::cout << "JustinaVision.->Starting face recognition old. " << std::endl;
    std_msgs::Empty msg;
    JustinaVision::pubFacStartRecogOld.publish(msg);
}

void JustinaVision::stopFaceRecognition()
{
    std::cout << "JustinaVision.->Stopping face recognition. " << std::endl;
    std_msgs::Empty msg;
    JustinaVision::pubFacStopRecog.publish(msg);
}

void JustinaVision::facRecognize()
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
}

void JustinaVision::facTrain(std::string id, int numOfFrames)
{
    std::cout << "JustinaVision.->Training face with id " << id << " with " << numOfFrames << " frames." <<  std::endl;
    vision_msgs::VisionFaceTrainObject msg;
    msg.id = id;
    msg.frames = numOfFrames;
    JustinaVision::pubTrainFaceNum.publish(msg);
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

    //JustinaVision::lastRecognizedFaces.clear();
    return true;
}

int JustinaVision::getLastTrainingResult()
{
    return JustinaVision::lastFaceRecogResult;
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
void JustinaVision::startHandDetectBB(float x, float y, float z)
{
	geometry_msgs::Point32 msg;
	msg.x = x;
	msg.y = y;
	msg.z = z;
	pubStartHandDetectBB.publish(msg);
}

void JustinaVision::stopHandDetectBB()
{
	std_msgs::Empty msg;
	pubStopHandDetectBB.publish(msg);
}

bool JustinaVision::getDetectionHandBB()
{
	return JustinaVision::isHandDetectedBB;

}

//callbacks for the hand detect in front of gripper
void JustinaVision::callbackHandDetectBB(const std_msgs::Bool::ConstPtr& msg)
{
	JustinaVision::isHandDetectedBB = msg->data;
}

//Methods for move the train object and move the tranining base
void JustinaVision::trainObject(const std::string name)
{
    vision_msgs::TrainObject srv;
    srv.request.name = name;
    srvTrainObject.call(srv);
}

void JustinaVision::moveBaseTrainVision(const std_msgs::String& msg)
{
    pubMove_base_train_vision.publish(msg);
}

