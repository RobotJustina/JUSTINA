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
ros::Publisher * JustinaKnowledge::pubEnableEdit;
ros::Publisher * JustinaKnowledge::pubLoadFromFile;
ros::Publisher * JustinaKnowledge::pubDeleteKnownLoc;
ros::Publisher * JustinaKnowledge::pubSaveInFile;
bool JustinaKnowledge::updateKnownLoc = false;
tf::TransformListener* JustinaKnowledge::tf_listener;

JustinaKnowledge::~JustinaKnowledge(){
  delete cliKnownLoc;
  delete cliAddUpKnownLoc;
  delete subUpdateKnowmLoc;
  delete pubEnableEdit;
  delete pubLoadFromFile;
  delete pubDeleteKnownLoc;
  delete pubSaveInFile;
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
  pubEnableEdit = new ros::Publisher(
      nh->advertise<std_msgs::Bool>("/knowledge/edit_known_loc", 1));
  pubLoadFromFile = new ros::Publisher(
      nh->advertise<std_msgs::String>("/knowledge/load_from_file", 1));
  pubDeleteKnownLoc = new ros::Publisher(
      nh->advertise<std_msgs::String>("/knowledge/delete_known_locations", 1));
  pubSaveInFile = new ros::Publisher(
      nh->advertise<std_msgs::String>("/knowledge/save_in_file", 1));
  tf_listener->waitForTransform("map", "base_link", ros::Time(0), ros::Duration(5.0));
}

void JustinaKnowledge::callBackUpdateKnownLoc(
    const std_msgs::Bool::ConstPtr updateKnownLoc){
  JustinaKnowledge::updateKnownLoc = updateKnownLoc->data;
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

void JustinaKnowledge::getKnownLocations(
		std::map<std::string, std::vector<float> >& locations) {
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

void JustinaKnowledge::getUpdateKnownLoc(bool& updateKnownLoc){
  updateKnownLoc = JustinaKnowledge::updateKnownLoc;
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

