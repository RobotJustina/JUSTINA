/*
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

#include "knowledge/KnownLocations.h"

class JustinaKnowledge {
private:
	ros::NodeHandle * nh;
	static ros::ServiceClient * cliKnownLoc;

public:

	static void setNodeHandle(ros::NodeHandle * nh);

	static void getKnownLocations(
			std::map<std::string, std::vector<float> >& locations);
};

#endif /* TOOLS_JUSTINA_TOOLS_SRC_JUSTINAKNOWLEDGE_H_ */
