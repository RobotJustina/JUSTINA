/*
 * JustinaKnowledge.cpp
 *
 *  Created on: 05/02/2017
 *      Author: rey
 */

#include "justina_tools/JustinaKnowledge.h"

ros::ServiceClient * JustinaKnowledge::cliKnownLoc;
ros::ServiceClient * JustinaKnowledge::cliAddUpKnownLoc;
ros::Subscriber * JustinaKnowledge::subUpdateKnowmLoc;
ros::Subscriber * JustinaKnowledge::subInitKnowmLoc;
ros::Publisher * JustinaKnowledge::pubEnableEdit;
ros::Publisher * JustinaKnowledge::pubLoadFromFile;
ros::Publisher * JustinaKnowledge::pubDeleteKnownLoc;
ros::Publisher * JustinaKnowledge::pubSaveInFile;
ros::ServiceClient * JustinaKnowledge::cliGetPredQues;
ros::ServiceClient * JustinaKnowledge::cliGetPredLaArmPose;
ros::ServiceClient * JustinaKnowledge::cliGetPredRaArmPose;
ros::ServiceClient * JustinaKnowledge::cliAddUpdateObjectViz;
ros::ServiceClient * JustinaKnowledge::cliIsInArea;
ros::ServiceClient * JustinaKnowledge::cliGetVisitLocationsPath;
ros::ServiceClient * JustinaKnowledge::cliGetPlanPath;
ros::ServiceClient * JustinaKnowledge::cliGetRoomOfPoint;
bool JustinaKnowledge::updateKnownLoc = false;
bool JustinaKnowledge::initKnownLoc = false;
tf::TransformListener* JustinaKnowledge::tf_listener;

JustinaKnowledge::~JustinaKnowledge(){
    delete cliKnownLoc;
    delete cliAddUpKnownLoc;
    delete subUpdateKnowmLoc;
    delete subInitKnowmLoc;
    delete pubEnableEdit;
    delete pubLoadFromFile;
    delete pubDeleteKnownLoc;
    delete pubSaveInFile;
    delete cliGetPredLaArmPose;
    delete cliAddUpdateObjectViz;
    delete cliIsInArea;
    delete cliGetVisitLocationsPath;
    delete cliGetPlanPath;
    delete tf_listener;
}

void JustinaKnowledge::setNodeHandle(ros::NodeHandle * nh) {
    tf_listener = new tf::TransformListener();
    cliKnownLoc = new ros::ServiceClient(
            nh->serviceClient<knowledge_msgs::KnownLocations>(
                "/knowledge/known_locations"));
    cliAddUpKnownLoc = new ros::ServiceClient(
            nh->serviceClient<knowledge_msgs::AddUpdateKnownLoc>(
                "/knowledge/add_update_known_locations"));
    subUpdateKnowmLoc = new ros::Subscriber(
            nh->subscribe("/knowledge/update_location_markers", 1, &JustinaKnowledge::callBackUpdateKnownLoc));
    subInitKnowmLoc = new ros::Subscriber(
            nh->subscribe("/knowledge/init_location_markers", 1, &JustinaKnowledge::callBackInitKnownLoc));
    pubEnableEdit = new ros::Publisher(
            nh->advertise<std_msgs::Bool>("/knowledge/edit_known_loc", 1));
    pubLoadFromFile = new ros::Publisher(
            nh->advertise<std_msgs::String>("/knowledge/load_from_file", 1));
    pubDeleteKnownLoc = new ros::Publisher(
            nh->advertise<std_msgs::String>("/knowledge/delete_known_locations", 1));
    pubSaveInFile = new ros::Publisher(
            nh->advertise<std_msgs::String>("/knowledge/save_in_file", 1));
    cliGetPredQues = new ros::ServiceClient(
            nh->serviceClient<knowledge_msgs::GetPredefinedQuestions>(
                "/knowledge/get_predefined_questions"));
    cliGetPredLaArmPose = new ros::ServiceClient(
            nh->serviceClient<knowledge_msgs::GetPredefinedArmsPoses>(
                "/knowledge/la_predefined_poses"));
    cliGetPredRaArmPose = new ros::ServiceClient(
            nh->serviceClient<knowledge_msgs::GetPredefinedArmsPoses>(
                "/knowledge/ra_predefined_poses"));
    cliAddUpdateObjectViz = new ros::ServiceClient(
            nh->serviceClient<env_msgs::AddUpdateObjectViz>(
                "/knowledge/object_description"));
    cliIsInArea = new ros::ServiceClient(
            nh->serviceClient<knowledge_msgs::IsPointInKnownArea>(
                "/knowledge/is_point_in_area"));
    cliGetVisitLocationsPath = new ros::ServiceClient(
            nh->serviceClient<knowledge_msgs::GetVisitLocationsPath>(
                "/knowledge/get_visit_locations_in_path"));
    cliGetPlanPath = new ros::ServiceClient(
            nh->serviceClient<navig_msgs::PlanPath>(
                "/navigation/mvn_pln/plan_path"));
    cliGetRoomOfPoint = new ros::ServiceClient(
            nh->serviceClient<knowledge_msgs::GetRoomOfPoint>(
                "/knowledge/get_room_of_point"));
    tf_listener->waitForTransform("map", "base_link", ros::Time(0), ros::Duration(5.0));
}

void JustinaKnowledge::callBackUpdateKnownLoc(
        const std_msgs::Bool::ConstPtr updateKnownLoc){
    JustinaKnowledge::updateKnownLoc = updateKnownLoc->data;
}

void JustinaKnowledge::callBackInitKnownLoc(
        const std_msgs::Bool::ConstPtr initKnownLoc){
    JustinaKnowledge::initKnownLoc = initKnownLoc->data;
}

void JustinaKnowledge::getRobotPose(float &currentX, float &currentY, float &currentTheta){
    tf::StampedTransform transform;
    tf::Quaternion q;
    tf_listener->lookupTransform("map", "base_link", ros::Time(0), transform);
    q = transform.getRotation();

    currentX = transform.getOrigin().x();
    currentY = transform.getOrigin().y();
    currentTheta = atan2((float)q.z(), (float)q.w()) * 2;
}

void JustinaKnowledge::getRobotPoseRoom(std::string &location){
    float currX, currY, currTheta;
    JustinaKnowledge::getRobotPose(currX, currY, currTheta);
    location = JustinaKnowledge::getRoomOfPoint(currX, currY);
}

void JustinaKnowledge::getKnownLocations(
        std::map<std::string, std::vector<float> >& locations) {
    locations.clear();
    knowledge_msgs::KnownLocations srv;
    if (cliKnownLoc->call(srv)) {
        for (std::vector<knowledge_msgs::MapKnownLocation>::iterator it =
                srv.response.locations.begin();
                it != srv.response.locations.end(); ++it) {
            locations.insert(
                    std::pair<std::string, std::vector<float> >(it->name,
                        it->value));
        }
    } else {
        ROS_ERROR("Failed to call service known_locations");
    }
}
        
void JustinaKnowledge::getKnownLocation(std::string location, float &x, float &y, float &a){
    std::map<std::string, std::vector<float> > locations;
    JustinaKnowledge::getKnownLocations(locations);
    std::map<std::string, std::vector<float> >::iterator it;
    it = locations.find(location);
    if(it != locations.end()){
        std::vector<float> components = it->second;
        x = components[0];
        y = components[1];
        if(components.size() == 3)
            a = components[2];
    }
}

bool JustinaKnowledge::existKnownLocation(std::string location){
    std::map<std::string, std::vector<float> > locations;
    JustinaKnowledge::getKnownLocations(locations);
    std::map<std::string, std::vector<float> >::iterator it;
    it = locations.find(location);
    if(it == locations.end())
        return false;
    return true;
}

void JustinaKnowledge::getUpdateKnownLoc(bool& updateKnownLoc){
    updateKnownLoc = JustinaKnowledge::updateKnownLoc;
    JustinaKnowledge::updateKnownLoc = false;
}

void JustinaKnowledge::getInitKnownLoc(bool& initKnownLoc){
    initKnownLoc = JustinaKnowledge::initKnownLoc;
    JustinaKnowledge::initKnownLoc = false;
}

void JustinaKnowledge::enableInteractiveUpdate(bool enable){
    std_msgs::Bool msg;
    msg.data = enable;
    pubEnableEdit->publish(msg);
}

void JustinaKnowledge::loadFromFile(const std::string filePath){
    std_msgs::String msg;
    msg.data = filePath;
    pubLoadFromFile->publish(msg);
}

void JustinaKnowledge::saveInFile(const std::string filePath){
    std_msgs::String msg;
    msg.data = filePath;
    pubSaveInFile->publish(msg);
}

void JustinaKnowledge::addUpdateKnownLoc(std::string name, std::vector<float> values){
    knowledge_msgs::AddUpdateKnownLoc srv;
    srv.request.loc.name = name;
    srv.request.loc.value = values;
    if (cliAddUpKnownLoc->call(srv)) {
    } else {
        ROS_ERROR("Failed to call service known_locations");
    }
}

void JustinaKnowledge::addUpdateKnownLoc(std::string name){
    knowledge_msgs::AddUpdateKnownLoc srv;
    std::vector<float> values;
    float x, y, theta;
    getRobotPose(x, y, theta);
    srv.request.loc.name = name;
    values.push_back(x);
    values.push_back(y);
    srv.request.loc.value = values;
    if (cliAddUpKnownLoc->call(srv)) {
    } else {
        ROS_ERROR("Failed to call service known_locations");
    }
}

void JustinaKnowledge::addUpdateKnownLoc(std::string name, float ori){
    knowledge_msgs::AddUpdateKnownLoc srv;
    std::vector<float> values;
    float x, y, theta;
    getRobotPose(x, y, theta);
    values.push_back(x);
    values.push_back(y);
    values.push_back(ori);
    srv.request.loc.name = name;
    srv.request.loc.value = values;
    if (cliAddUpKnownLoc->call(srv)) {
    } else {
        ROS_ERROR("Failed to call service known_locations");
    }
}

void JustinaKnowledge::addUpdateKnownLoc(std::string name, float x, float y){
    knowledge_msgs::AddUpdateKnownLoc srv;
    std::vector<float> values;
    values.push_back(x);
    values.push_back(y);
    srv.request.loc.name = name;
    srv.request.loc.value = values;
    if (cliAddUpKnownLoc->call(srv)) {
    } else {
        ROS_ERROR("Failed to call service known_locations");
    }
}

void JustinaKnowledge::addUpdateKnownLoc(std::string name, float x, float y, float ori){
    knowledge_msgs::AddUpdateKnownLoc srv;
    std::vector<float> values;
    values.push_back(x);
    values.push_back(y);
    values.push_back(ori);
    srv.request.loc.name = name;
    srv.request.loc.value = values;
    if (cliAddUpKnownLoc->call(srv)) {
    } else {
        ROS_ERROR("Failed to call service known_locations");
    }
}

void JustinaKnowledge::deleteKnownLoc(const std::string name){
    std_msgs::String msg;
    msg.data = name;
    pubDeleteKnownLoc->publish(msg);
}

void JustinaKnowledge::getPredQuestions(std::map<std::string, std::string> &predQues){
    knowledge_msgs::GetPredefinedQuestions srv;
    if (cliGetPredQues->call(srv)) {
        for(int i = 0; i < srv.response.predefinedQuestions.size(); i++){
            predQues[srv.response.predefinedQuestions[i].question] = srv.response.predefinedQuestions[i].answer;
        }
    } else {
        ROS_ERROR("Failed to call service known_locations");
    }
}

void JustinaKnowledge::getPredQuestions(std::vector<std::string> &questions){
    knowledge_msgs::GetPredefinedQuestions srv;
    if (cliGetPredQues->call(srv)) {
        for(int i = 0; i < srv.response.predefinedQuestions.size(); i++){
            questions.push_back(srv.response.predefinedQuestions[i].question);
        }
    } else {
        ROS_ERROR("Failed to call service known_locations");
    }
}

bool JustinaKnowledge::comparePredQuestion(std::string question, std::string &answer){
    std::map<std::string, std::string> predQues;
    getPredQuestions(predQues);
    boost::replace_all(question, ",", " ");
    boost::replace_all(question, ".", " ");
    std::cout << "JustinaKnowledge.->Ask answer:" << question << std::endl;
    /*std::replace(question.begin(), question.end(), "," , " ");
      std::replace(question.begin(), question.end(), "," , ".");*/
    std::map<std::string, std::string>::iterator quesFound = predQues.find(question);
    if(quesFound == predQues.end())
        return false;
    answer = quesFound->second;
    std::cout << "JustinaKnowledge.->Answer:" << answer << std::endl;
    return true;
}

void JustinaKnowledge::getPredLaArmPose(std::string name, std::vector<float> &poses){
    knowledge_msgs::GetPredefinedArmsPoses srv;
    srv.request.name = name;
    if (cliGetPredLaArmPose->call(srv)) {
        for(int i = 0; i < srv.response.angles.size(); i++)
            poses.push_back(srv.response.angles[i].data);
    } else 
        ROS_ERROR("Failed to call service known_locations");
}

void JustinaKnowledge::getPredRaArmPose(std::string name, std::vector<float> &poses){
    knowledge_msgs::GetPredefinedArmsPoses srv;
    srv.request.name = name;
    if (cliGetPredRaArmPose->call(srv)) {
        for(int i = 0; i < srv.response.angles.size(); i++)
            poses.push_back(srv.response.angles[i].data);
    } else 
        ROS_ERROR("Failed to call service known_locations");
}

void JustinaKnowledge::addUpdateObjectViz(std::string id, float minX, float minY, float minZ, float maxX, float maxY, float maxZ, float centroidX, float centroidY, float centroidZ, float colorR, float colorG, float colorB, std::string frame_original, std::string frame_goal){
    env_msgs::ObjectViz objectViz;
    objectViz.id.data = id;
    objectViz.frame_original.data = frame_original;
    objectViz.frame_goal.data = frame_goal;
    objectViz.centroid.x = centroidX;
    objectViz.centroid.y = centroidY;
    objectViz.centroid.z = centroidZ;
    objectViz.minPoint.x = minX;
    objectViz.minPoint.y = minY;
    objectViz.minPoint.z = minZ;
    objectViz.maxPoint.x = maxX;
    objectViz.maxPoint.y = maxY;
    objectViz.maxPoint.z = maxZ;
    objectViz.color.x = colorR;
    objectViz.color.y = colorG;
    objectViz.color.z = colorB;
    env_msgs::AddUpdateObjectViz srv;
    srv.request.object = objectViz;
    if (cliAddUpdateObjectViz->call(srv)) {
    } else 
        ROS_ERROR("Failed to call add object viz");
}

bool JustinaKnowledge::isPointInKnownArea(float x, float y, std::string location){
    knowledge_msgs::IsPointInKnownArea srv;
    geometry_msgs::Point32 data;
    data.x = x;
    data.y = y;
    data.z = 0.0f;
    srv.request.location = location;
    srv.request.point = data;
    if (cliIsInArea->call(srv)) {
        return srv.response.isInLocation;
    } else 
        ROS_ERROR("Failed to call service known_locations");
    return false;
}

std::vector<std::string> JustinaKnowledge::getRoomsFromPath(nav_msgs::Path path){
    knowledge_msgs::GetVisitLocationsPath srv;
    std::vector<std::string> locations;
    srv.request.path = path;
    if(!cliGetVisitLocationsPath->call(srv)){
        ROS_ERROR("Failed to call service get visit Locations path");
        return locations;
    }
    for(int i = 0; i < srv.response.locations.size(); i++)
        locations.push_back(srv.response.locations[i].data);
    return locations;
}

std::vector<std::string> JustinaKnowledge::getRoomsFromPath(float startX, float startY, float goalX, float goalY){
    navig_msgs::PlanPath srv;
    std::vector<std::string> locations;
    srv.request.start_pose.position.x = startX;
    srv.request.start_pose.position.y = startY;
    srv.request.goal_pose.position.x = goalX;
    srv.request.goal_pose.position.y = goalY;
    if(!cliGetPlanPath->call(srv)){
        ROS_ERROR("Failed to call service get plan path");
        return locations;
    }
    return getRoomsFromPath(srv.response.path);
}

std::vector<std::string> JustinaKnowledge::getRoomsFromPath(float startX, float startY, std::string goalLocation){
    navig_msgs::PlanPath srv;
    std::vector<std::string> locations;
    srv.request.start_pose.position.x = startX;
    srv.request.start_pose.position.y = startY;
    srv.request.goal_location_id = goalLocation;
    if(!cliGetPlanPath->call(srv)){
        ROS_ERROR("Failed to call service get plan path");
        return locations;
    }
    return getRoomsFromPath(srv.response.path);
}
        
std::vector<std::string> JustinaKnowledge::getRoomsFromPath(std::string startLocation, std::string goalLocation){
    navig_msgs::PlanPath srv;
    std::vector<std::string> locations;
    srv.request.start_location_id = startLocation;
    srv.request.goal_location_id = goalLocation;
    if(!cliGetPlanPath->call(srv)){
        ROS_ERROR("Failed to call service get plan path");
        return locations;
    }
    knowledge_msgs::GetVisitLocationsPath srv2;
    srv2.request.path = srv.response.path;
    if(!cliGetVisitLocationsPath->call(srv2)){
        ROS_ERROR("Failed to call service get visit Locations path");
        return locations;
    }
    return getRoomsFromPath(srv.response.path);
}

std::string JustinaKnowledge::getRoomOfPoint(float x, float y){
    geometry_msgs::Point32 point;
    point.x = x;
    point.y = y;
    point.z = 0.0;
    knowledge_msgs::GetRoomOfPoint srv;
    srv.request.point = point; 
    if(!cliGetRoomOfPoint->call(srv)){
        ROS_ERROR("Failed to call service get plan path");
        return "";
    }
    return srv.response.location.data;
}
