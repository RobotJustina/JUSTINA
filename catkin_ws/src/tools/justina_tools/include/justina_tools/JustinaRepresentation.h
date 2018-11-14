#ifndef TOOLS_JUSTINA_TOOLS_SRC_JUSTINAREPRESENTATION_H_
#define TOOLS_JUSTINA_TOOLS_SRC_JUSTINAREPRESENTATION_H_

#include "ros/ros.h"

#include <map>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>

#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "tf/transform_listener.h"

#include "knowledge_msgs/PlanningCmdClips.h"
#include "knowledge_msgs/planning_cmd.h"
#include "knowledge_msgs/StrQueryKDB.h"
#include "knowledge_msgs/InitKDB.h"

#include <boost/algorithm/string/replace.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

class JustinaRepresentation {
    private:
        ros::NodeHandle * nh;

        static ros::Publisher * command_runCLIPS;
        static ros::Publisher * command_resetCLIPS;
        static ros::Publisher * command_factCLIPS;
        static ros::Publisher * command_ruleCLIPS;
        static ros::Publisher * command_agendaCLIPS;
        static ros::Publisher * command_sendCLIPS;
        static ros::Publisher * command_loadCLIPS;
        static ros::Publisher * command_sendAndRunCLIPS;
        static ros::Publisher * command_response;
        static ros::ServiceClient * cliSpechInterpretation;
        static ros::ServiceClient * cliStringInterpretation;
        static ros::ServiceClient * cliStrQueryKDB;
        static ros::ServiceClient * cliInitKDB;

        static bool strQueryKDB(std::string query, std::string &result, int timeout);

    public:

        ~JustinaRepresentation();

        static void setNodeHandle(ros::NodeHandle * nh);
        static void runCLIPS(bool enable);
        static void resetCLIPS(bool enable);
        static void factCLIPS(bool enable);
        static void ruleCLIPS(bool enable);
        static void agendaCLIPS(bool enable);
        static void sendCLIPS(std::string command);
        static void loadCLIPS(std::string file);
        static void getLocations(std::string path ,std::map<std::string, std::vector<std::string> >& locations);
        static void getObjects(std::string path ,std::map<std::string, std::vector<std::string> >& objects);
        static void addLocations(std::map<std::string, std::vector<std::string> >& locations, std::string name, std::vector<std::string> values);
        static void addObjects(std::map<std::string, std::vector<std::string> >& objects, std::string name, std::vector<std::string> values);
        static void sendAndRunCLIPS(std::string command);
        static bool speachInterpretation();
        static bool stringInterpretation(std::string strToInterpretation, std::string &strInterpreted);
        static bool orderInterpeted(std::string strInterpreted, std::string &typeOrder, std::string &object1, std::string &object2);
        static bool prepareInterpretedQuestionToQuery(std::string strInterpreted, std::string &query);
        static bool selectCategoryObjectByName(std::string idObject, std::string &category, int timeout);
        static bool answerQuestionFromKDB(std::string question, std::string &answer,int timeout);
        static bool initKDB(std::string filePath, bool run, float timeout);
        static bool insertKDB(std::string nameRule, std::vector<std::string> params, int timeout);
        static bool insertConfidenceAndGetCategory(std::string id, int index, float confidence, std::string &category,  int timeout);
        static bool selectTwoObjectsToGrasp(int &index1, int &index2, int timeout);
        static bool getDoorsPath(std::vector<std::string> rooms, std::vector<std::string> &doorLocations, int timeout);
        static bool updateStateDoor(int id, std::string loc1, std::string loc2, bool state, int timeout);
        static bool updateFurnitureFromObject(std::string name, int id, std::string furniture, std::string imageName, int timeout);
        static bool updateLocationFromFurniture(std::string name, int id, std::string location, int timeout);
        static bool getSemanticMap(std::vector<std::vector<std::string> > &semanticMap, int timeout);
        static bool isObjectInDefaultLocation(std::string name, int id, std::string location, bool &isInDefaultLocation, int timeout);
        static bool getOriginAndGoalFromObject(std::string name, int id, std::string &origin, std::string &destiny, int timeout);

};

#endif /* TOOLS_JUSTINA_TOOLS_SRC_JUSTINAREPRESENTATION_H_ */
