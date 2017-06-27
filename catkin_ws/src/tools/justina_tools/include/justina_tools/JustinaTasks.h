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
        SM_GUIDING_MEMORIZING_OPERATOR_ELF,
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

    enum POSE{
        NONE,
        STANDING,
        SITTING,
        LYING
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
	static bool waitRecognizedFace(float timeout, std::string id, int gender, POSE pose, std::vector<vision_msgs::VisionFaceObject> &faces);
	static bool waitRecognizedGesture(std::vector<vision_msgs::GestureSkeleton> &gestures, float timeout);
	static bool findPerson(std::string person = "", int gender = -1, POSE pose = NONE);
    static bool findGesturePerson(std::string gesture);
    static bool tellGenderPerson(std::string &gender);
    static int manyGenderPerson(int gender);
	static bool findAndFollowPersonToLoc(std::string goalLocation);
	static bool findObject(std::string idObject, geometry_msgs::Pose & pose, bool & withLeftOrRightArm);
    static void closeToGoalWithDistanceTHR(float goalx, float goaly, float thr, float timeout);
	static bool moveActuatorToGrasp(float x, float y, float z, bool withLeftArm,
			std::string id);
	static bool dropObject(std::string id = "", bool withLeftOrRightArm = false, int timeout = 30000);
    static bool guideAPerson(std::string loc, int timeout = 0);
    static bool followAPersonAndRecogStop(std::string stopRecog);
    static bool findTable(std::string &ss);

private:
	static bool getNearestRecognizedFace(std::vector<vision_msgs::VisionFaceObject> facesObject, float distanceMax, Eigen::Vector3d &centroidFace, int &genderRecog);
	static bool turnAndRecognizeFace(std::string id, int gender, POSE pose, float initAngPan, float incAngPan,float maxAngPan, float initAngTil, float incAngTil, float maxAngTil,float incAngleTurn, float maxAngleTurn, Eigen::Vector3d &centroidFace, int &genderRecog);
	static bool getNearestRecognizedGesture(std::string typeGesture, std::vector<vision_msgs::GestureSkeleton> gestures, float distanceMax, Eigen::Vector3d &nearestGesture);
	static bool turnAndRecognizeGesture(std::string typeGesture, float initAngPan, float incAngPan, float maxAngPan, float initAngTil, float incAngTil, float maxAngTil, float incAngleTurn, float maxAngleTurn, Eigen::Vector3d &gesturePos);
};
