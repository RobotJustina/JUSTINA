#pragma once
#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaHRI.h"
#include "justina_tools/JustinaManip.h"
#include "justina_tools/JustinaNavigation.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTools.h"
#include "justina_tools/JustinaKnowledge.h"

class JustinaTasks
{
private:
    static bool is_node_set;

public:
    //
    //The startSomething functions, only publish the goal pose or path and return inmediately after starting movement
    //The others, block until a goal-reached signal is received
    //
    enum STATE{
        SM_GUIDING_MEMORIZING_OPERATOR_SAY,
        SM_GUIDING_MEMORIZING_OPERATOR,
        SM_GUIDING_PHASE,
        SM_GUIDING_STOP,
        SM_GUIDING_FINISHED,
        SM_WAIT_FOR_OPERATOR,
        SM_MEMORIZING_OPERATOR,
        SM_WAIT_FOR_LEGS_FOUND,
        SM_FOLLOWING_PHASE,
        SM_FOLLOWING_FINISHED
    };

    static bool setNodeHandle(ros::NodeHandle* nh);
    static bool alignWithTable();
    static bool alignWithTable(float distToTable);
    static bool graspNearestObject(bool withLeftArm);
    static bool graspNearestObject(std::vector<vision_msgs::VisionObject>& recoObjList, bool withLeftArm);
    static bool graspObject(float x, float y, float z, bool withLeftArm, std::string idObject = "");
    static bool placeObject(bool withLeftArm, float h = 0, bool placeBag = false);
    static void sayAndAsyncNavigateToLoc(std::string location, bool say = true);
    static bool sayAndSyncNavigateToLoc(std::string location, int timeout, bool say = true);
	static std::vector<vision_msgs::VisionFaceObject> waitRecognizedFace(
			float timeout, std::string id, bool &recognized);
	static bool findPerson(std::string person = "");
	static bool findAndFollowPersonToLoc(std::string goalLocation);
	static bool findObject(std::string idObject, geometry_msgs::Pose & pose, bool & withLeftOrRightArm);
	static bool moveActuatorToGrasp(float x, float y, float z, bool withLeftArm,
			std::string id);
	static bool dropObject(std::string id = "", bool withLeftOrRightArm = false, int timeout = 30000);
    static bool guideAPerson(std::string loc, int timeout = 0);
    static bool followAPersonAndRecogStop(std::string stopRecog);

private:
	static Eigen::Vector3d getNearestRecognizedFace(
			std::vector<vision_msgs::VisionFaceObject> facesObject,
			float distanceMax, bool &found);
	static Eigen::Vector3d turnAndRecognizeFace(std::string id, float initAngPan,
			float incAngPan, float maxAngPan, float incAngleTurn,
			float maxAngleTurn, bool &recog);

};
