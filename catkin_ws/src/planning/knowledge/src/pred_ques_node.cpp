#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include "ros/ros.h"

#include "knowledge_msgs/GetPredefinedQuestions.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

using boost::property_tree::ptree;

std::map<std::string, std::string> questions;

void parse(boost::property_tree::ptree pt){
  BOOST_FOREACH( ptree::value_type const& v, pt.get_child("questions") ) {
    if(v.first == "question"){
      std::string question = v.second.get<std::string>("q");
      std::string answer = v.second.get<std::string>("a");
      questions[question] = answer;
      /*std::cout << "pred_ques_node.->Question:" << question << std::endl;
      std::cout << "pred_ques_node.->Answer:" << answer << std::endl;*/
    }
  }
  std::cout << "pred_ques_node.->Loading predefined questions"
             << pt.get_child("questions").size() << std::endl;
}

void loadQuestions(std::string filePath){
  using boost::property_tree::ptree;
  ptree pt;
  read_xml(filePath, pt);
  parse(pt);
}

bool getPredefinedQuestions(knowledge_msgs::GetPredefinedQuestions::Request &req,
          knowledge_msgs::GetPredefinedQuestions::Response &res){
  for(std::map<std::string, std::string>::iterator it = questions.begin();
        it != questions.end(); it++){
    knowledge_msgs::MapPredefinedQuestions question;
    question.question = it->first;
    question.answer = it->second;
    res.predefinedQuestions.push_back(question);
  }
  return true;
}

int main(int argc, char ** argv) {

  std::cout << "INITIALIZING KNOWN PREDEFINED QUESTIONS." << std::endl;

  ros::init(argc, argv, "pred_questions_node");
	ros::NodeHandle nh;

  ros::Rate rate(10);

  std::string filePath = "";
	for (int i = 0; i < argc; i++) {
		std::string strParam(argv[i]);
    if (strParam.compare("-f") == 0)
      filePath = argv[++i];
  }

  ros::ServiceServer serGetPredQues = nh.advertiseService(
        "/knowledge/get_predefined_questions", getPredefinedQuestions);

  loadQuestions(filePath);

	while (ros::ok()) {

		rate.sleep();
		ros::spinOnce();

	}

	return -1;

}
