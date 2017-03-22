/*
 *
 * JustinaKnowledge.h
 *
 *  Created on: 05/02/2017
 *      Author: rey
 */

#ifndef TOOLS_JUSTINA_TOOLS_SRC_JUSTINAKNOWLEDGE_H_
#define TOOLS_JUSTINA_TOOLS_SRC_JUSTINAKNOWLEDGE_H_

#include "ros/ros.h"

#include <map>
#include <vector>

#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "tf/transform_listener.h"

#include "knowledge_msgs/KnownLocations.h"
#include "knowledge_msgs/AddUpdateKnownLoc.h"
#include "knowledge_msgs/GetPredefinedQuestions.h"

#include <boost/algorithm/string/replace.hpp>

class JustinaKnowledge {
private:
	ros::NodeHandle * nh;
	static ros::ServiceClient * cliKnownLoc;
  static ros::ServiceClient * cliAddUpKnownLoc;
  static ros::Subscriber * subUpdateKnowmLoc;
  static ros::Publisher * pubEnableEdit;
  static ros::Publisher * pubLoadFromFile;
  static ros::Publisher * pubDeleteKnownLoc;
  static ros::Publisher * pubSaveInFile;
  static ros::ServiceClient * cliGetPredQues;
  static bool updateKnownLoc;
  static tf::TransformListener* tf_listener;

private:
  static void callBackUpdateKnownLoc(const std_msgs::Bool::ConstPtr updateKnownLoc);
  static void getRobotPose(float &currentX, float &currentY, float &currentTheta);

public:

  ~JustinaKnowledge();

	static void setNodeHandle(ros::NodeHandle * nh);
	static void getKnownLocations(
			std::map<std::string, std::vector<float> >& locations);
  static void getUpdateKnownLoc(bool& updateKnownLoc);
  static void enableInteractiveUpdate(bool enable);
  static void loadFromFile(const std::string filePath);
  static void saveInFile(const std::string filePath);
  static void addUpdateKnownLoc(std::string name, std::vector<float> values);
  static void addUpdateKnownLoc(std::string name);
  static void addUpdateKnownLoc(std::string name, float ori);
  static void addUpdateKnownLoc(std::string name, float x, float y);
  static void addUpdateKnownLoc(std::string name, float x, float y, float ori);
  static void deleteKnownLoc(const std::string name);
  static void getPredQuestions(std::map<std::string, std::string> &predQues);
  static void getPredQuestions(std::vector<std::string> &questions);
  static bool comparePredQuestion(std::string question, std::string &answer);
};

#endif /* TOOLS_JUSTINA_TOOLS_SRC_JUSTINAKNOWLEDGE_H_ */
