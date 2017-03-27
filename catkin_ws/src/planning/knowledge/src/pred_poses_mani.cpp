#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include <knowledge_msgs/GetPredefinedArmsPoses.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <cmath>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/filesystem/path.hpp>

std::map<std::string, std::vector<float> > laPredefPoses;
std::map<std::string, std::vector<float> > raPredefPoses;

std::map<std::string, std::vector<float> > loadArrayOfFloats(std::string path)
{
    std::cout << "ManipPln.->Extracting array of floats from file: " << path << std::endl;
    std::vector<std::string> lines;
    std::ifstream file(path.c_str());
    std::string tempStr;
    while(std::getline(file, tempStr))
        lines.push_back(tempStr);

    //Extraction of lines without comments
    for(size_t i=0; i < lines.size(); i++)
    {
        size_t idx = lines[i].find("//");
        if(idx != std::string::npos)
            lines[i] = lines[i].substr(0, idx);
    }

    std::map<std::string, std::vector<float> > data;

    float fValue;
    bool parseSuccess;
    for(size_t i=0; i<lines.size(); i++)
    {
        //std::cout << "ManipPln.->Parsing line: " << lines[i] << std::endl;
        std::vector<std::string> parts;
        boost::split(parts, lines[i], boost::is_any_of(" ,\t"), boost::token_compress_on);
        if(parts.size() < 2)
            continue;
        //First part should be the label and the next ones, the values
        if(!boost::filesystem::portable_posix_name(parts[0]))
            continue;
        parseSuccess = true;
        for(size_t j=1; j<parts.size() && parseSuccess; j++)
        {
            std::stringstream ssValue(parts[j]);
            if(!(ssValue >> fValue)) parseSuccess = false;
            else data[parts[0]].push_back(fValue);
        }
    }
    return data;
}

bool loadPredefinedPosesAndMovements(std::string folder, std::map<std::string, std::vector<float> > & laPredefPoses, std::map<std::string, std::vector<float> > & raPredefPoses)
{
    //
    //Load predefined positions for left arm
    //
    std::string leftArmPosesFile = folder + "left_arm_poses.txt";
    std::map<std::string, std::vector<float> > data = loadArrayOfFloats(leftArmPosesFile);
    for(std::map<std::string, std::vector<float> >::iterator i = data.begin(); i != data.end(); i++)
    {
        if(i->second.size() != 7)
        {
            std::cout << "ManipPln.->Invalid number of angles in left arm predef position " << i->first << std::endl;
            continue;
        }
       	laPredefPoses[i->first] = i->second;
    }
    std::cout << "ManipPln.->Left arm predefined positions: " <<std::endl;
    for(std::map<std::string, std::vector<float> >::iterator i = laPredefPoses.begin(); i != laPredefPoses.end(); i++)
    {
        std::cout << i->first << " ";
        for(int j=0; j < i->second.size(); j++)
            std::cout << i->second[j] << " ";
        std::cout << std::endl;
    }

    //
    //Load predefined positions for right arm
    //
    std::string rightArmPosesFile = folder + "right_arm_poses.txt";
    data = loadArrayOfFloats(rightArmPosesFile);
    for(std::map<std::string, std::vector<float> >::iterator i = data.begin(); i != data.end(); i++)
    {
        if(i->second.size() != 7)
        {
            std::cout << "ManipPln.->Invalid number of angles in right arm predef position " << i->first << std::endl;
            continue;
        }
        raPredefPoses[i->first] = i->second;
    }
    std::cout << "ManipPln.->Right arm predefined positions: " <<std::endl;
    for(std::map<std::string, std::vector<float> >::iterator i = laPredefPoses.begin(); i != laPredefPoses.end(); i++)
    {
        std::cout << i->first << " ";
        for(int j=0; j < i->second.size(); j++)
            std::cout << i->second[j] << " ";
        std::cout << std::endl;
    }
}

bool callbackLaGetPredefPoses(knowledge_msgs::GetPredefinedArmsPosesRequest &req, knowledge_msgs::GetPredefinedArmsPosesResponse &resp)
{
	std::map<std::string, std::vector<float> >::iterator it = laPredefPoses.find(req.name);
	if(it == laPredefPoses.end())
		return false;
	for(int i = 0; i < it->second.size(); i++)
	{
		std_msgs::Float32 msg;
		msg.data = it->second[i];
		resp.angles.push_back(msg);
	}
	return true;
}

bool callbackRaGetPredefPoses(knowledge_msgs::GetPredefinedArmsPosesRequest &req, knowledge_msgs::GetPredefinedArmsPosesResponse &resp)
{
	std::map<std::string, std::vector<float> >::iterator it = raPredefPoses.find(req.name);
	if(it == raPredefPoses.end())
		return false;
	for(int i = 0; i < it->second.size(); i++)
	{
		std_msgs::Float32 msg;
		msg.data = it->second[i];
		resp.angles.push_back(msg);
	}
	return true;
}
				
int main(int argc, char ** argv )
{
	ros::init(argc, argv, "pred_poses_mani_node");
	ros::NodeHandle nh;

    	std::string folder = "";
    	for(int i=0; i < argc; i++)
    	{
        	std::string strParam(argv[i]);
        	if(strParam.compare("-f") == 0)
            		folder = argv[++i];
    	}

	ros::Rate rate(10);

  	ros::ServiceServer serviceLaPoses = nh.advertiseService("/knowledge/la_predefined_poses", callbackLaGetPredefPoses);
  	ros::ServiceServer serviceRaPoses = nh.advertiseService("/knowledge/ra_predefined_poses", callbackRaGetPredefPoses);

	loadPredefinedPosesAndMovements(folder, laPredefPoses, raPredefPoses);

	while(ros::ok()){


		rate.sleep();
		ros::spinOnce();
	}

	return 1;
}
