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

#include "knowledge/KnownLocations.h"
#include "knowledge/Add_update_knownLoc.h"

class JustinaKnowledge {
private:
	ros::NodeHandle * nh;
	static ros::ServiceClient * cliKnownLoc;
  static ros::ServiceClient * cliAddUpKnownLoc;
  static ros::Subscriber * subUpdateKnowmLoc;
  static ros::Publisher * pubEnableEdit;
  static bool updateKnownLoc;

private:
  static void callBackUpdateKnownLoc(const std_msgs::Bool::ConstPtr updateKnownLoc);

public:

  ~JustinaKnowledge();

	static void setNodeHandle(ros::NodeHandle * nh);
	static void getKnownLocations(
			std::map<std::string, std::vector<float> >& locations);
  static void getUpdateKnownLoc(bool& updateKnownLoc);
  static void enableInteractiveUpdate(bool enable);
  static void addUpdateKnownLoc(std::string name, std::vector<float> values);
  static void addUpdateKnownLoc(std::string name);
  static void addUpdateKnownLoc(std::string name, float ori);
  static void addUpdateKnownLoc(std::string name, float x, float y);
  static void addUpdateKnownLoc(std::string name, float x, float y, float ori);
};

#endif /* TOOLS_JUSTINA_TOOLS_SRC_JUSTINAKNOWLEDGE_H_ */
