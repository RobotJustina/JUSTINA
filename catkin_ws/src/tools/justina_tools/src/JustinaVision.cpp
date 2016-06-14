#include "justina_tools/JustinaVision.h"

bool JustinaVision::is_node_set = false;
//Members for operating skeleton finder
ros::Publisher JustinaVision::pubSktStartRecog;
ros::Publisher JustinaVision::pubSktStopRecog;
//Members for operating face recognizer
ros::Publisher JustinaVision::pubFacStartRecog;
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
//Detect objects
ros::ServiceClient JustinaVision::cltDetectObjects;

bool JustinaVision::setNodeHandle(ros::NodeHandle* nh)
{
    if(JustinaVision::is_node_set)
        return true;
    if(nh == 0)
        return false;

    std::cout << "JustinaVision.->Setting ros node..." << std::endl;
    JustinaVision::pubSktStartRecog = nh->advertise<std_msgs::Empty>("/vision/skeleton_finder/start_recog", 1);
    JustinaVision::pubSktStopRecog = nh->advertise<std_msgs::Empty>("/vision/skeleton_finder/stop_recog", 1);
    //Members for operating face recognizer
    JustinaVision::pubFacStartRecog = nh->advertise<std_msgs::Empty>("/vision/face_recognizer/start_recog", 1);
    JustinaVision::pubFacStopRecog = nh->advertise<std_msgs::Empty>("/vision/face_recognizer/stop_recog", 1);
    JustinaVision::pubTrainFace = nh->advertise<std_msgs::String>("/vision/face_recognizer/run_face_trainer", 1);
    JustinaVision::pubTrainFaceNum = nh->advertise<vision_msgs::VisionFaceTrainObject>("/vision/face_recognizer/run_face_trainer_frames", 1);
    JustinaVision::pubRecFace = nh->advertise<std_msgs::Empty>("/vision/face_recognizer/run_face_recognizer", 1);
    JustinaVision::pubRecFaceByID = nh->advertise<std_msgs::String>("/vision/face_recognizer/run_face_recognizer_id", 1);
    JustinaVision::pubClearFacesDB = nh->advertise<std_msgs::Empty>("/vision/face_recognizer/clearfacesdb", 1);
    JustinaVision::pubClearFacesDBByID = nh->advertise<std_msgs::String>("/vision/face_recognizer/clearfacesdbbyid", 1);
    JustinaVision::subFaces = nh->subscribe("/vision/face_recognizer/faces", 1, &JustinaVision::callbackFaces);
    JustinaVision::subTrainer = nh->subscribe("/vision/face_recognizer/trainer_result", 1, &JustinaVision::callbackTrainer);
    //detect objects                                                                                    
    JustinaVision::cltDetectObjects = nh->serviceClient<vision_msgs::DetectObjects>("/vision/det_objs");
    
    JustinaVision::is_node_set = true;
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

    JustinaVision::lastRecognizedFaces.clear();
    return true;
}

int JustinaVision::getLastTrainingResult()
{
    return JustinaVision::lastFaceRecogResult;
}

//Object detection
bool JustinaVision::detectObjects(std::vector<vision_msgs::VisionObject>& recoObjList)
{
    std::cout << "JustinaVision.->Trying to detect objects... " << std::endl;
    vision_msgs::DetectObjects srv;
    if(!cltDetectObjects.call(srv))
    {
        std::cout << std::endl << "Justina::Vision can't detect anything" << std::endl << std::endl;
        return false;
    }
    recoObjList=srv.response.recog_objects;
    std::cout << "JustinaVision.->Detected " << int(recoObjList.size()) << " objects" << std::endl;
    return true;
}

void JustinaVision::callbackFaces(const vision_msgs::VisionFaceObjects::ConstPtr& msg)
{
    JustinaVision::lastRecognizedFaces = msg->recog_faces;
}

void JustinaVision::callbackTrainer(const std_msgs::Int32::ConstPtr& msg)
{
    JustinaVision::lastFaceRecogResult = msg->data;
}
