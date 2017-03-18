/*
 * JustinaKnowledge.cpp
 *
 *  Created on: 05/02/2017
 *      Author: rey
 */

#include "justina_tools/JustinaKnowledge.h"
#include "justina_tools/JustinaNavigation.h"

ros::ServiceClient * JustinaKnowledge::cliKnownLoc;
ros::ServiceClient * JustinaKnowledge::cliAddUpKnownLoc;
ros::Subscriber * JustinaKnowledge::subUpdateKnowmLoc;
ros::Publisher * JustinaKnowledge::pubEnableEdit;
bool JustinaKnowledge::updateKnownLoc = false;

JustinaKnowledge::~JustinaKnowledge(){
  delete cliKnownLoc;
  delete cliAddUpKnownLoc;
  delete subUpdateKnowmLoc;
  delete pubEnableEdit;
}

void JustinaKnowledge::setNodeHandle(ros::NodeHandle * nh) {
	cliKnownLoc = new ros::ServiceClient(
			nh->serviceClient<knowledge::KnownLocations>(
					"/knowledge/known_locations"));
  cliAddUpKnownLoc = new ros::ServiceClient(
      nh->serviceClient<knowledge::Add_update_knownLoc>(
          "/knowledge/add_update_known_locations"));
  subUpdateKnowmLoc = new ros::Subscriber(
      nh->subscribe("/knowledge/update_location_markers", 1, &JustinaKnowledge::callBackUpdateKnownLoc));
  pubEnableEdit = new ros::Publisher(
      nh->advertise<std_msgs::Bool>("/knowledge/edit_known_loc", 1));
}

void JustinaKnowledge::callBackUpdateKnownLoc(
    const std_msgs::Bool::ConstPtr updateKnownLoc){
  JustinaKnowledge::updateKnownLoc = updateKnownLoc->data;
}

void JustinaKnowledge::getKnownLocations(
		std::map<std::string, std::vector<float> >& locations) {
	knowledge::KnownLocations srv;
  if (cliKnownLoc->call(srv)) {
		for (std::vector<knowledge::MapKnownLocation>::iterator it =
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

void JustinaKnowledge::getUpdateKnownLoc(bool& updateKnownLoc){
  updateKnownLoc = JustinaKnowledge::updateKnownLoc;
}

void JustinaKnowledge::enableInteractiveUpdate(bool enable){
  std_msgs::Bool msg;
  msg.data = enable;
  pubEnableEdit->publish(msg);
}

void JustinaKnowledge::addUpdateKnownLoc(std::string name, std::vector<float> values){
  knowledge::Add_update_knownLoc srv;
  srv.request.loc.name = name;
  srv.request.loc.value = values;
  if (cliAddUpKnownLoc->call(srv)) {
  } else {
    ROS_ERROR("Failed to call service known_locations");
  }
}

void JustinaKnowledge::addUpdateKnownLoc(std::string name){
	knowledge::Add_update_knownLoc srv;
	std::vector<float> values;
	float x, y, theta;
	JustinaNavigation::getRobotPose(x, y, theta);
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
	knowledge::Add_update_knownLoc srv;
	std::vector<float> values;
	float x, y, theta;
	JustinaNavigation::getRobotPose(x, y, theta);
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
	knowledge::Add_update_knownLoc srv;
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
	knowledge::Add_update_knownLoc srv;
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

