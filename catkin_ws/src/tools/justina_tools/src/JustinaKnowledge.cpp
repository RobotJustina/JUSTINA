/*
 * JustinaKnowledge.cpp
 *
 *  Created on: 05/02/2017
 *      Author: rey
 */

#include "justina_tools/JustinaKnowledge.h"

ros::ServiceClient * JustinaKnowledge::cliKnownLoc;

void JustinaKnowledge::setNodeHandle(ros::NodeHandle * nh) {
	cliKnownLoc = new ros::ServiceClient(
			nh->serviceClient<knowledge::KnownLocations>(
					"/knowledge/known_locations"));
}

void JustinaKnowledge::getKnownLocations(
		std::map<std::string, std::vector<float> >& locations) {
	knowledge::KnownLocations srv;
	if (cliKnownLoc->call(srv)) {
		for (std::vector<knowledge::MapKnownLocation>::iterator it =
				srv.response.locations.begin();
				it != srv.response.locations.begin(); it++) {
			locations.insert(
					std::pair<std::string, std::vector<float> >(it->name,
							it->value));
		}
	} else {
		ROS_ERROR("Failed to call service known_locations");
	}
}

