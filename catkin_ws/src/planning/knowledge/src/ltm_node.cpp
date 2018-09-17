#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include "ros/ros.h"
#include "knowledge_msgs/KnownLocations.h"
#include "knowledge_msgs/AddUpdateKnownLoc.h"
#include "knowledge_msgs/IsPointInKnownArea.h"
#include "knowledge_msgs/GetVisitLocationsPath.h"
#include "knowledge_msgs/GetRoomOfPoint.h"

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <visualization_msgs/Marker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <nav_msgs/Path.h>

#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "std_msgs/ColorRGBA.h"

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

using namespace visualization_msgs;

typedef struct _Vertex2 {
    float x, y;
    _Vertex2() {
    }
    _Vertex2(float x, float y) {
        this->x = x;
        this->y = y;
    }
    static _Vertex2 Zero() {
        _Vertex2 vertex;
        vertex.x = 0.0;
        vertex.y = 0.0;
        return vertex;
    }
    _Vertex2 sub(_Vertex2 v) {
        _Vertex2 vertex;
        vertex.x = v.x - x;
        vertex.y = v.y - y;
        return vertex;
    }
    float norm() {
        return sqrt(pow(x, 2) + pow(y, 2));
    }
} Vertex2;

typedef struct _Segment {
    Vertex2 v1, v2;
    _Segment() {
    }
    _Segment(Vertex2 v1, Vertex2 v2) {
        this->v1 = v1;
        this->v2 = v2;
    }
} Segment;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
Marker areasMarker;
interactive_markers::MenuHandler menu_handler;

std::map<std::string, std::vector<float> > locations;
std::map<std::string, std::pair<std::string, std::vector<std::pair<float, float> > > > delimitation;
bool initKnowLoco = false;
bool updateKnowLoc = false;
bool enableEditKnowLoc = false;

Marker makeBox(InteractiveMarker &msg) {
    Marker marker;

    marker.type = Marker::SPHERE;
    marker.pose.position.z = 0.0;
    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;
    marker.color.r = 0.0;
    marker.color.g = 0.75;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    return marker;
}

InteractiveMarkerControl& makeBoxControl(InteractiveMarker &msg) {
    InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back(makeBox(msg));
    msg.controls.push_back(control);

    return msg.controls.back();
}

void processFeedback(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    std::ostringstream s;
    /*s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";*/

    std::ostringstream mouse_point_ss;
    /*if (feedback->mouse_point_valid) {
      mouse_point_ss << " at " << feedback->mouse_point.x << ", "
      << feedback->mouse_point.y << ", " << feedback->mouse_point.z
      << " in frame " << feedback->header.frame_id;
      }*/

    tf::Quaternion q;
    std::map<std::string, std::vector<float> >::iterator it;

    switch (feedback->event_type) {
        case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
            ROS_INFO_STREAM(
                    s.str() << ": button click" << mouse_point_ss.str() << ".");
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
            ROS_INFO_STREAM(
                    s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << ".");
            break;

        case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
            /*ROS_INFO_STREAM(
              s.str() << ": pose changed" << "\nposition = " << feedback->pose.position.x << ", " << feedback->pose.position.y << ", " << feedback->pose.position.z << "\norientation = " << feedback->pose.orientation.w << ", " << feedback->pose.orientation.x << ", " << feedback->pose.orientation.y << ", " << feedback->pose.orientation.z << "\nframe: " << feedback->header.frame_id << " time: " << feedback->header.stamp.sec << "sec, " << feedback->header.stamp.nsec << " nsec");*/
            tf::quaternionMsgToTF(feedback->pose.orientation, q);
            /*std::cout << "Angle:" << q.getAngle() << ", " << "Axis:"
              << q.getAxis().x() << ", " << q.getAxis().y() << ", "
              << q.getAxis().z() << std::endl;*/
            it = locations.find(feedback->marker_name);
            if (it != locations.end()) {
                it->second[0] = feedback->pose.position.x;
                it->second[1] = feedback->pose.position.y;
                if (it->second.size() > 2)
                    it->second[2] = q.getAngle() * (q.getAxis().z() > 0 ? 1 : -1);
            }
            updateKnowLoc = true;
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
            ROS_INFO_STREAM(
                    s.str() << ": mouse down" << mouse_point_ss.str() << ".");
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
            ROS_INFO_STREAM(s.str() << ": mouse up" << mouse_point_ss.str() << ".");
            break;
    }

    server->applyChanges();
}

void makeLocMarker(float xpos, float ypos, float zpos, float theta,
        bool with_orientation, bool with_control, const std::string name) {
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";
    int_marker.pose.position.x = xpos;
    int_marker.pose.position.y = ypos;
    int_marker.pose.position.z = zpos;
    tf::quaternionTFToMsg(tf::Quaternion(tf::Vector3(0, 0, 1), theta),
            int_marker.pose.orientation);
    int_marker.scale = 1.0;

    int_marker.name = name;
    int_marker.description = name;

    makeBoxControl(int_marker);

    if (with_control) {
        int_marker.controls[0].interaction_mode =
            InteractiveMarkerControl::MOVE_PLANE;
        int_marker.controls[0].orientation.w = 1;
        int_marker.controls[0].orientation.x = 0;
        int_marker.controls[0].orientation.y = 1;
        int_marker.controls[0].orientation.z = 0;
        int_marker.controls[0].name = "move plane z = 0";
        int_marker.controls[0].always_visible = true;
    }

    if (with_control && with_orientation) {
        InteractiveMarkerControl control;
        // make a control that rotates around the view axis
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "rotate_z";
        control.always_visible = true;
        int_marker.controls.push_back(control);
    }

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);

    menu_handler.apply(*server, int_marker.name);
}

bool loadKnownLocations(std::string path,
        std::map<std::string, std::vector<float> > & locations) {
    std::cout << "Ltm.->Loading known locations from " << path << std::endl;
    std::vector<std::string> lines;
    std::ifstream file(path.c_str());
    std::string tempStr;
    while (std::getline(file, tempStr))
        lines.push_back(tempStr);

    //Extraction of lines without comments
    for (size_t i = 0; i < lines.size(); i++) {
        size_t idx = lines[i].find("//");
        if (idx != std::string::npos)
            lines[i] = lines[i].substr(0, idx);
    }

    locations.clear();
    float locX, locY, locAngle;
    bool parseSuccess;
    for (size_t i = 0; i < lines.size(); i++) {
        //std::cout << "Ltm.->Parsing line: " << lines[i] << std::endl;
        std::vector<std::string> parts;
        std::vector<float> loc;
        boost::split(parts, lines[i], boost::is_any_of(" ,\t"),
                boost::token_compress_on);
        if (parts.size() < 3)
            continue;
        //std::cout << "Ltm.->Parsing splitted line: " << lines[i] << std::endl;
        parseSuccess = true;
        std::stringstream ssX(parts[1]);
        if (!(ssX >> locX))
            parseSuccess = false;
        std::stringstream ssY(parts[2]);
        if (!(ssY >> locY))
            parseSuccess = false;
        loc.push_back(locX);
        loc.push_back(locY);
        if (parts.size() >= 4) {
            std::stringstream ssAngle(parts[3]);
            if (!(ssAngle >> locAngle))
                parseSuccess = false;
            loc.push_back(locAngle);
        }

        if (parseSuccess) {
            locations[parts[0]] = loc;
        }
    }
    std::cout << "Ltm.->Total number of known locations: "
        << locations.size() << std::endl;
    for (std::map<std::string, std::vector<float> >::iterator it =
            locations.begin(); it != locations.end(); it++) {
        std::cout << "Ltm.->Location " << it->first << " " << it->second[0]
            << " " << it->second[1];
        if (it->second.size() > 2)
            std::cout << " " << it->second[2];
        std::cout << std::endl;
    }
    if (locations.size() < 1)
        std::cout << "Ltm.->WARNING: Cannot load known locations from file: "
            << path << ". There are no known locations." << std::endl;

    return true;
}

bool loadKnownDelimitation(std::string path, std::map<std::string, std::pair<std::string, std::vector<std::pair<float, float> > > > & delimitation){
    std::cout << "Ltm.->Loading known locations from " << path << std::endl;
    std::vector<std::string> lines;
    std::ifstream file(path.c_str());
    std::string tempStr;
    while (std::getline(file, tempStr))
        lines.push_back(tempStr);

    //Extraction of lines without comments
    for (size_t i = 0; i < lines.size(); i++) {
        size_t idx = lines[i].find("//");
        if (idx != std::string::npos)
            lines[i] = lines[i].substr(0, idx);
    }

    delimitation.clear();
    for (size_t i = 0; i < lines.size(); i++) {
        //std::cout << "Ltm.->Parsing line: " << lines[i] << std::endl;
        std::vector<std::string> parts;
        std::pair<std::string, std::vector<std::pair<float, float > > > delimi;
        bool parseSuccess = true;
        std::vector<std::pair<float, float> > vertex;
        boost::split(parts, lines[i], boost::is_any_of(" ,\t"), boost::token_compress_on);
        std::string id0 , id1;

        if (parts.size() < 2)
            continue;
        //std::cout << "Ltm.->Parsing splitted line: " << lines[i] << std::endl;

        if(!(parts[1].compare("furniture") == 0 || parts[1].compare("room") == 0))
            continue;
        if(parts[1].compare("room") == 0){
            int div = parts.size() % 2;
            if(div != 0)
                continue;
            if(parts.size() < 10)
                continue;
            id0 = parts[0];
            id1 = parts[0];
            for(int i = 2; i < parts.size() - 1; i+=2){
                float locX, locY;
                std::stringstream ssX(parts[i]);
                if (!(ssX >> locX))
                    parseSuccess = false;
                std::stringstream ssY(parts[i + 1]);
                if (!(ssY >> locY))
                    parseSuccess = false;
                if(!parseSuccess)
                    break;
                std::pair<float, float> pair = std::make_pair(locX, locY);
                vertex.push_back(pair);
            }
        }
        else{
            if(parts.size() < 3)
                continue;
            id0 = parts[0];
            id1 = parts[2];
        }
        if(parseSuccess){
            std::cout << "ltm_node.->Adding to area " << id0 << std::endl; 
            delimi = std::make_pair(id1, vertex); 
            delimitation[id0] = delimi;
        }
    }
    return true;
}

float getDeterminant(Vertex2 vertex1, Vertex2 vertex2, Vertex2 vertex3) {
        float determinant = (vertex2.x - vertex1.x) * (vertex3.y - vertex2.y)
                        - (vertex2.y - vertex1.y) * (vertex3.x - vertex2.x);
            return determinant;
}

bool testSegmentIntersect(Segment segment1, Segment segment2) {
    float determinante1 = getDeterminant(segment1.v2, segment1.v1, segment2.v1);
    float determinante2 = getDeterminant(segment1.v2, segment1.v1, segment2.v2);
    if ((determinante1 < 0 && determinante2 > 0)
            || (determinante1 > 0 && determinante2 < 0)) {
        determinante1 = getDeterminant(segment2.v2, segment2.v1, segment1.v1);
        determinante2 = getDeterminant(segment2.v2, segment2.v1, segment1.v2);
        if ((determinante1 < 0 && determinante2 > 0)
                || (determinante1 > 0 && determinante2 < 0))
            return true;
        else
            return false;
    } else if (determinante1 == 0 || determinante2 == 0)
        return false;
    else
        return false;
}

bool validatePointInArea(std::vector<std::pair<float, float> > vertex, geometry_msgs::Point32 point ){
    bool test = false;
    // std::cout << "ltm_node.->Testing the point:" << point.x << "," << point.y << std::endl;  
    Vertex2 v0(point.x, point.y);
    for(int j = 0; j < vertex.size() && !test; j++){
        Vertex2 v1(vertex[j].first, vertex[j].second);
        Vertex2 v2;
        if(j < vertex.size() - 1)
            v2 = Vertex2(vertex[j + 1].first, vertex[j + 1].second);
        else
            v2 = Vertex2(vertex[0].first, vertex[0].second);
        float det = getDeterminant(v1, v2, v0);
        //std::cout << "tlm_node.->Determinant:" << det << std::endl; 
        if(det < 0)
            test = true;
    }
    return test;
}

void initMarkersLoc(const std::map<std::string, std::vector<float> > locations){

    server.reset(
            new interactive_markers::InteractiveMarkerServer(
                "/hri/rviz/location_markers", "", false));

    for (std::map<std::string, std::vector<float> >::const_iterator it =
            locations.begin(); it != locations.end(); ++it) {
        if (it->second.size() >= 2) {
            if (it->second.size() == 2)
                makeLocMarker(it->second[0], it->second[1], 0.075, 0, false,
                        false, it->first);
            else
                makeLocMarker(it->second[0], it->second[1], 0.075,
                        it->second[2], true, false, it->first);
        }
    }

    server->applyChanges();
}

bool getKnownLocations(knowledge_msgs::KnownLocations::Request &req,
        knowledge_msgs::KnownLocations::Response &res) {

    for (std::map<std::string, std::vector<float> >::const_iterator it =
            locations.begin(); it != locations.end(); ++it) {

        knowledge_msgs::MapKnownLocation map;
        map.name = it->first;
        map.value.push_back(it->second[0]);
        map.value.push_back(it->second[1]);
        if (it->second.size() > 2)
            map.value.push_back(it->second[2]);

        res.locations.push_back(map);
    }

    return true;
}

void callbackEnableKnownLocations(const std_msgs::Bool::ConstPtr& enable){
    server.reset(
            new interactive_markers::InteractiveMarkerServer(
                "/hri/rviz/location_markers", "", false));

    for (std::map<std::string, std::vector<float> >::const_iterator it =
            locations.begin(); it != locations.end(); ++it) {
        if (it->second.size() >= 2) {
            if (it->second.size() == 2)
                makeLocMarker(it->second[0], it->second[1], 0.075, 0, false,
                        enable->data, it->first);
            else
                makeLocMarker(it->second[0], it->second[1], 0.075,
                        it->second[2], true, enable->data, it->first);
        }
    }

    server->applyChanges();

    enableEditKnowLoc = enable;

}

bool addOrUpdateKnownLoc(knowledge_msgs::AddUpdateKnownLoc::Request &req, knowledge_msgs::AddUpdateKnownLoc::Response &res){
    std::vector<float> new_values = req.loc.value;
    std::map<std::string, std::vector<float> >::iterator it;
    bool updateControl = true;

    it = locations.find(req.loc.name);
    if (it != locations.end()) {
        it->second[0] = new_values[0];
        it->second[1] = new_values[1];
        if (it->second.size() > 2 && new_values.size() > 2){
            it->second[2] = new_values[2];
            updateControl = false;
        }else if(it->second.size() > 2 && new_values.size() == 2)
            it->second.erase(it->second.end() - 1);
        else if(it->second.size() == 2 && new_values.size() > 2)
            it->second.push_back(new_values[2]);
        updateKnowLoc = true;
        initKnowLoco = false;
    }
    else{
        locations[req.loc.name] = new_values;
        updateKnowLoc = false;
        initKnowLoco = true;
    }

    if(updateControl){
        server.reset(
                new interactive_markers::InteractiveMarkerServer(
                    "/hri/rviz/location_markers", "", false));

        for (std::map<std::string, std::vector<float> >::const_iterator it =
                locations.begin(); it != locations.end(); ++it) {
            if (it->second.size() >= 2) {
                if (it->second.size() == 2)
                    makeLocMarker(it->second[0], it->second[1], 0.075, 0, false,
                            enableEditKnowLoc, it->first);
                else
                    makeLocMarker(it->second[0], it->second[1], 0.075,
                            it->second[2], true, enableEditKnowLoc, it->first);
            }
        }

        server->applyChanges();
    }
    return true;
}

void callbackLoadFromFile(const std_msgs::String::ConstPtr& locationsFilePath){
    locations.clear();
    if (!loadKnownLocations(locationsFilePath->data, locations))
        std::cout << "ltm_node.-> Can not load file of known locations." << std::endl;
    initMarkersLoc(locations);
    updateKnowLoc = false;
    initKnowLoco = true;
}

void callbackDeleteKnownLoc(const std_msgs::String::ConstPtr& location){
    std::map<std::string,std::vector<float> >::iterator it;
    it = locations.find (location->data);
    if(it != locations.end()){
        locations.erase(it);
        initMarkersLoc(locations);
        updateKnowLoc = false;
        initKnowLoco = true;
    }
}

void callbackSaveInFile(const std_msgs::String::ConstPtr& pathName){
    std::ofstream fileSave;
    fileSave.open(pathName->data.c_str());
    if (fileSave.is_open()) {
        for(std::map<std::string, std::vector<float> >::iterator it = locations.begin();
                it != locations.end(); ++it){
            fileSave << it->first << "\t";
            fileSave << it->second[0] << "\t";
            fileSave << it->second[1];
            if(it->second.size() > 2)
                fileSave << "\t" << it->second[2];
            fileSave << std::endl;
        }
    }
    fileSave.close();
}

bool isInLocation(knowledge_msgs::IsPointInKnownArea::Request &req, knowledge_msgs::IsPointInKnownArea::Response &res) {
    std::cout << "ltm_node.->Validating point if is in region:" << req.location << std::endl;
    std::string location = req.location;
    geometry_msgs::Point32 point = req.point;

    std::map<std::string, std::pair<std::string, std::vector<std::pair<float, float> > > >::iterator iterator;

    iterator = delimitation.find(req.location);

    res.isInLocation = true;
    bool haveVertex = false;
    if(iterator != delimitation.end()){
        std::cout << "ltm_node.->Have been found the location in map" << std::endl;
        std::pair<std::string, std::vector<std::pair<float, float> > > compose  = iterator->second;
        std::vector<std::pair<float, float> > vertex;
        if(!(compose.first.compare(iterator->first)) == 0){
            std::cout << "ltm_node.->The location is a furniture." << std::endl;
            std::map<std::string, std::pair<std::string, std::vector<std::pair<float, float> > > >::iterator iterator2;
            iterator2 = delimitation.find(compose.first);
            if(iterator2 != delimitation.end()){
                std::cout << "ltm_node.->Found the father location." << compose.first << std::endl;
                std::pair<std::string, std::vector<std::pair<float, float> > > compose2  = iterator2->second;
                if(compose2.second.size() > 0){
                    vertex = compose2.second;
                    haveVertex = true;
                }
            }
        }
        else{
            std::cout << "ltm_node.->The location is a room." << std::endl;
            if(compose.second.size() > 0){
                vertex = compose.second;
                haveVertex = true;
            }
        }
        if(haveVertex){
            std::cout << "ltm_node.->Validating if point is contained in location" << std::endl;
            std::cout << "ltm_node.->Vertex size:" << vertex.size() << std::endl;
            res.isInLocation = !validatePointInArea(vertex, point);
        }
    }

    return true;
}

bool getVisitLocationPath(knowledge_msgs::GetVisitLocationsPath::Request &req, knowledge_msgs::GetVisitLocationsPath::Response &res){
    std::vector<std_msgs::String> locations;
    for(int i = 0; i < req.path.poses.size(); i++){
        geometry_msgs::Pose pose = req.path.poses[i].pose;
        geometry_msgs::Point32 point;
        point.x = pose.position.x;
        point.y = pose.position.y;
        point.z = 0.0;

        std::map<std::string, std::pair<std::string, std::vector<std::pair<float, float> > > >::iterator iterator;

        float sizeAreas = 0;
        std::vector<std::string> areaLoc;
        for(iterator = delimitation.begin(); iterator != delimitation.end(); iterator++){
            std::pair<std::string, std::vector<std::pair<float, float> > > compose  = iterator->second;
            if(iterator->first.compare("arena") != 0 && compose.first.compare(iterator->first) == 0){
                if(!validatePointInArea(compose.second, point))
                    areaLoc.push_back(compose.first);
            }
        }

        if(areaLoc.size() > 0){
            std_msgs::String loc;
            if(!locations.size()){
                loc.data = areaLoc[0];
                locations.push_back(loc);
            }
            else{
                loc = locations[locations.size() -1];
                for(int j = 0; j < areaLoc.size(); j++){
                    if(loc.data.compare(areaLoc[j]) == 0)
                        break;
                    loc.data = areaLoc[j];
                    locations.push_back(loc);
                    break;
                }
            }
        }

    }
    res.locations = locations;
    return true;
}

bool getRoomOfPoint(knowledge_msgs::GetRoomOfPoint::Request &req, knowledge_msgs::GetRoomOfPoint::Response &res){
    geometry_msgs::Point32 point = req.point;
    point.z = 0.0;

    bool isInArena = false;
    bool isInLocation = false;

    std::map<std::string, std::pair<std::string, std::vector<std::pair<float, float> > > >::iterator iterator;

    for(iterator = delimitation.begin(); iterator != delimitation.end() && !isInLocation; iterator++){
        std::pair<std::string, std::vector<std::pair<float, float> > > compose  = iterator->second;
        if(!validatePointInArea(compose.second, point)){
            if(compose.first.compare(iterator->first) == 0){
                if(iterator->first.compare("arena") == 0)
                    isInArena = true;
                else
                    isInLocation = true;
            }
        }
    }
   
    res.location.data = ""; 
    if(isInArena)
        res.location.data = "arena";
    if(isInLocation)
        res.location.data = iterator->first;

    return true;
}

int main(int argc, char ** argv) {

    std::cout << "INITIALIZING KNOWN LOCATIONS." << std::endl;

    ros::init(argc, argv, "known_locations_node");
    ros::NodeHandle nh;

    ros::Rate rate(10);

    std::string locationsFilePath = "";
    for (int i = 0; i < argc; i++) {
        std::string strParam(argv[i]);
        if (strParam.compare("-f") == 0)
            locationsFilePath = argv[++i];
    }

    std::string delimitationFilePath = "";
    for (int i = 0; i < argc; i++) {
        std::string strParam(argv[i]);
        if (strParam.compare("-d") == 0)
            delimitationFilePath = argv[++i];
    }

    ros::Publisher pubUpdateKnownLoc = nh.advertise<std_msgs::Bool>(
            "/knowledge/update_location_markers", 1);
    ros::Publisher pubInitKnownLoc = nh.advertise<std_msgs::Bool>(
            "/knowledge/init_location_markers", 1);
    ros::Publisher pubAreasMarker = nh.advertise<visualization_msgs::Marker>(
            "/knowldege/areas_marker", 1);
    ros::Subscriber subEditKnownLoc = nh.subscribe(
            "/knowledge/edit_known_loc", 1, callbackEnableKnownLocations);
    ros::ServiceServer service = nh.advertiseService(
            "/knowledge/known_locations", getKnownLocations);
    ros::ServiceServer serviceUpd = nh.advertiseService(
            "/knowledge/add_update_known_locations", addOrUpdateKnownLoc);
    ros::ServiceServer serviceIsInArea = nh.advertiseService(
            "/knowledge/is_point_in_area", isInLocation);
    ros::ServiceServer serviceGetVisitLocationsPath = nh.advertiseService(
            "/knowledge/get_visit_locations_in_path", getVisitLocationPath);
    ros::ServiceServer serviceGetRoomOfPoint = nh.advertiseService(
            "/knowledge/get_room_of_point", getRoomOfPoint);
    ros::Subscriber subLoad = nh.subscribe(
            "/knowledge/load_from_file", 1, callbackLoadFromFile);
    ros::Subscriber subSave = nh.subscribe(
            "/knowledge/save_in_file", 1, callbackSaveInFile);
    ros::Subscriber subDelete = nh.subscribe(
            "/knowledge/delete_known_locations", 1, callbackDeleteKnownLoc);

    if (!loadKnownLocations(locationsFilePath, locations))
        std::cout << "ltm_node.-> Can not load file of known locations." << std::endl;
    if (!loadKnownDelimitation(delimitationFilePath, delimitation))
        std::cout << "ltm_node.-> Can not load file of known delimitation." << std::endl;
    initMarkersLoc(locations);

    areasMarker.header.frame_id = "map";
    areasMarker.header.stamp = ros::Time();
    areasMarker.ns = "areas_delimitation";
    areasMarker.id = 0;
    areasMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    areasMarker.action = visualization_msgs::Marker::ADD;
    areasMarker.pose.orientation.x = 0.0;
    areasMarker.pose.orientation.y = 0.0;
    areasMarker.pose.orientation.z = 0.0;
    areasMarker.pose.orientation.w = 1.0;
    areasMarker.scale.x = 1;
    areasMarker.scale.y = 1;
    areasMarker.scale.z = 1;
    areasMarker.color.a = 1; // Don't forget to set the alpha!
    areasMarker.color.r = 1;
    areasMarker.color.g = 1;
    areasMarker.color.b = 1;

    std::map<std::string, std::pair<std::string, std::vector<std::pair<float, float> > > >::iterator iterator;

    float sizeAreas = 0;
    for(iterator = delimitation.begin(); iterator != delimitation.end(); iterator++){
        std::pair<std::string, std::vector<std::pair<float, float> > > compose  = iterator->second;
        std::vector<std::pair<float, float> > vertex;
        if(compose.first.compare(iterator->first) == 0)
            sizeAreas++;
    }

    float i = 0;
    for(iterator = delimitation.begin(); iterator != delimitation.end(); iterator++){
        std::pair<std::string, std::vector<std::pair<float, float> > > compose  = iterator->second;
        std::vector<std::pair<float, float> > vertex;
        if(compose.first.compare(iterator->first) == 0){
            std::vector<std::pair<float, float> >::iterator vertexIterator;
            int numberVertex = 0;
            std_msgs::ColorRGBA color;
            //color.r = 0.0;
            color.r = i++ / (sizeAreas / 3.0);
            color.g = i / sizeAreas;
            color.b = 1.0 - i++ / ( sizeAreas / 2.0);
            color.a = 0.5;
            for(vertexIterator = compose.second.begin(); vertexIterator != compose.second.end(); vertexIterator++){
                geometry_msgs::Point vertexData;
                vertexData.x = vertexIterator->first;
                vertexData.y = vertexIterator->second;
                vertexData.z = 0.0;
                areasMarker.points.push_back(vertexData);
                areasMarker.colors.push_back(color);
                numberVertex++;
                if(numberVertex == 3 && (vertexIterator + 1) != compose.second.end()){
                    numberVertex = 1;
                    areasMarker.points.push_back(vertexData);
                    areasMarker.colors.push_back(color);
                }
                else if(numberVertex == 2 && (vertexIterator + 1) == compose.second.end()){
                    geometry_msgs::Point vertexData;
                    vertexData.x = compose.second.begin()->first;
                    vertexData.y = compose.second.begin()->second;
                    vertexData.z = 0.0;
                    areasMarker.points.push_back(vertexData);
                    areasMarker.colors.push_back(color);
                }
            }
        }
    }

    while (ros::ok()) {

        if(updateKnowLoc){
            std_msgs::Bool msg;
            msg.data = updateKnowLoc;
            pubUpdateKnownLoc.publish(msg);
        }

        if(initKnowLoco){
            std_msgs::Bool msg;
            msg.data = initKnowLoco;
            pubInitKnownLoc.publish(msg);
        }

        updateKnowLoc = false;
        initKnowLoco = false;

        pubAreasMarker.publish(areasMarker);

        rate.sleep();
        ros::spinOnce();

    }

    return 1;

}
