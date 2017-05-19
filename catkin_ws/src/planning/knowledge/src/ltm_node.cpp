#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include "ros/ros.h"
#include "knowledge_msgs/KnownLocations.h"
#include "knowledge_msgs/AddUpdateKnownLoc.h"

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

using namespace visualization_msgs;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;

std::map<std::string, std::vector<float> > locations;
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
    marker.color.r = 1.0;
    marker.color.g = 0.0;
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

    ros::Publisher pubUpdateKnownLoc = nh.advertise<std_msgs::Bool>(
            "/knowledge/update_location_markers", 1);
    ros::Publisher pubInitKnownLoc = nh.advertise<std_msgs::Bool>(
            "/knowledge/init_location_markers", 1);
    ros::Subscriber subEditKnownLoc = nh.subscribe(
            "/knowledge/edit_known_loc", 1, callbackEnableKnownLocations);
    ros::ServiceServer service = nh.advertiseService(
            "/knowledge/known_locations", getKnownLocations);
    ros::ServiceServer serviceUpd = nh.advertiseService(
            "/knowledge/add_update_known_locations", addOrUpdateKnownLoc);
    ros::Subscriber subLoad = nh.subscribe(
            "/knowledge/load_from_file", 1, callbackLoadFromFile);
    ros::Subscriber subSave = nh.subscribe(
            "/knowledge/save_in_file", 1, callbackSaveInFile);
    ros::Subscriber subDelete = nh.subscribe(
            "/knowledge/delete_known_locations", 1, callbackDeleteKnownLoc);

    if (!loadKnownLocations(locationsFilePath, locations))
        std::cout << "ltm_node.-> Can not load file of known locations." << std::endl;
    initMarkersLoc(locations);

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

        rate.sleep();
        ros::spinOnce();

    }

    return -1;

}
