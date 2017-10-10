#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <tf/transform_listener.h>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

#include "vision_msgs/Skeletons.h"

using std::string;

xn::Context        g_Context;
xn::ScriptNode g_scriptNode;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator  g_UserGenerator;
ros::Publisher pubSkeletons;
tf::TransformListener * transformListener;
std::string frame_id;

XnBool g_bNeedPose   = FALSE;
XnChar g_strPose[20] = "";

bool hasAlreadyInitTracking = false;

void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
    ROS_INFO("New User %d", nId);

    if (g_bNeedPose)
        g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
    else
        g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
    ROS_INFO("Lost user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie) {
    ROS_INFO("Calibration started for user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationComplete(xn::SkeletonCapability& capability, XnUserID nId, XnCalibrationStatus eStatus, void* pCookie){
    if (eStatus == XN_CALIBRATION_STATUS_OK) {
        ROS_INFO("Calibration complete, start tracking user %d", nId);
        g_UserGenerator.GetSkeletonCap().StartTracking(nId);
    }
    else {
        ROS_INFO("Calibration failed for user %d", nId);
        if(eStatus == XN_CALIBRATION_STATUS_MANUAL_ABORT){
            ROS_INFO("Manual abort occured, stop attempting to calibrate!");
            return;
        }
        if (g_bNeedPose)
            g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
        else
            g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
    }
}

void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID nId, void* pCookie) {
    ROS_INFO("Pose %s detected for user %d", strPose, nId);
    g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
    g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

void publishTransform(XnUserID const& user, XnSkeletonJoint const& joint, string const& frame_id, string const& child_frame_id, vision_msgs::SkeletonJoint &skeletonJoint) {
    static tf::TransformBroadcaster br;

    XnSkeletonJointPosition joint_position;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, joint_position);
    double x = -joint_position.position.X / 1000.0;
    double y = joint_position.position.Y / 1000.0;
    double z = joint_position.position.Z / 1000.0;

    XnSkeletonJointOrientation joint_orientation;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, joint, joint_orientation);

    XnFloat* m = joint_orientation.orientation.elements;
    KDL::Rotation rotation(m[0], m[1], m[2],
            m[3], m[4], m[5],
            m[6], m[7], m[8]);
    double qx, qy, qz, qw;
    rotation.GetQuaternion(qx, qy, qz, qw);

    char child_frame_no[128];
    snprintf(child_frame_no, sizeof(child_frame_no), "%s_%d", child_frame_id.c_str(), user);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::Quaternion(qx, -qy, -qz, qw));

    // #4994
    tf::Transform change_frame;
    change_frame.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion frame_rotation;
    frame_rotation.setEuler(0, 0, M_PI);

    change_frame.setRotation(frame_rotation);

    transform = change_frame * transform;

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_no));

    tf::StampedTransform globalTransform;
    transformListener->lookupTransform("/base_link", frame_id,
                             ros::Time(0), globalTransform);
    transform = globalTransform * transform;
    
    geometry_msgs::Vector3 position;
    geometry_msgs::Quaternion orientation; 
    position.x = transform.getOrigin().getX();
    position.y = transform.getOrigin().getY();
    position.z = transform.getOrigin().getZ();
    orientation.x = transform.getRotation().getX();
    orientation.y = transform.getRotation().getY();
    orientation.z = transform.getRotation().getZ();
    orientation.w = transform.getRotation().getW();
    skeletonJoint.name_joint.data = std::string(const_cast<char *>(child_frame_id.c_str()));
    skeletonJoint.position = position;
    skeletonJoint.orientation = orientation;
}

void publishTransforms(const std::string& frame_id) {
    XnUserID users[15];
    XnUInt16 users_count = 15;
    g_UserGenerator.GetUsers(users, users_count);

    vision_msgs::Skeletons skeletons;

    for (int i = 0; i < users_count; ++i) {
        XnUserID user = users[i];
        if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
            continue;

        vision_msgs::Skeleton skeleton;

        vision_msgs::SkeletonJoint jointHead;
        vision_msgs::SkeletonJoint jointNeck;
        vision_msgs::SkeletonJoint jointTorso;
        vision_msgs::SkeletonJoint jointLeftShoulder;
        vision_msgs::SkeletonJoint jointLeftElbow;
        vision_msgs::SkeletonJoint jointLeftHand;
        vision_msgs::SkeletonJoint jointRightShoulder;
        vision_msgs::SkeletonJoint jointRightElbow;
        vision_msgs::SkeletonJoint jointRightHand;
        vision_msgs::SkeletonJoint jointLeftHip;
        vision_msgs::SkeletonJoint jointLeftKnee;
        vision_msgs::SkeletonJoint jointLeftFoot;
        vision_msgs::SkeletonJoint jointRightHip;
        vision_msgs::SkeletonJoint jointRightKnee;
        vision_msgs::SkeletonJoint jointRightFoot;

        publishTransform(user, XN_SKEL_HEAD,           frame_id, "head", jointHead);
        publishTransform(user, XN_SKEL_NECK,           frame_id, "neck", jointNeck);
        publishTransform(user, XN_SKEL_TORSO,          frame_id, "torso", jointTorso);

        publishTransform(user, XN_SKEL_RIGHT_SHOULDER,  frame_id, "left_shoulder", jointLeftShoulder);
        publishTransform(user, XN_SKEL_RIGHT_ELBOW,     frame_id, "left_elbow", jointLeftElbow);
        publishTransform(user, XN_SKEL_RIGHT_HAND,      frame_id, "left_hand", jointLeftHand);

        publishTransform(user, XN_SKEL_LEFT_SHOULDER, frame_id, "right_shoulder", jointRightShoulder);
        publishTransform(user, XN_SKEL_LEFT_ELBOW,    frame_id, "right_elbow", jointRightElbow);
        publishTransform(user, XN_SKEL_LEFT_HAND,     frame_id, "right_hand", jointRightHand);

        publishTransform(user, XN_SKEL_RIGHT_HIP,       frame_id, "left_hip", jointLeftHip);
        publishTransform(user, XN_SKEL_RIGHT_KNEE,      frame_id, "left_knee", jointLeftKnee);
        publishTransform(user, XN_SKEL_RIGHT_FOOT,      frame_id, "left_foot", jointLeftFoot);

        publishTransform(user, XN_SKEL_LEFT_HIP,      frame_id, "right_hip", jointRightHip);
        publishTransform(user, XN_SKEL_LEFT_KNEE,     frame_id, "right_knee", jointRightKnee);
        publishTransform(user, XN_SKEL_LEFT_FOOT,     frame_id, "right_foot", jointRightFoot);
    
        skeleton.user_id = user;

        skeleton.joints.push_back(jointHead);
        skeleton.joints.push_back(jointNeck);
        skeleton.joints.push_back(jointTorso);
        skeleton.joints.push_back(jointLeftShoulder);
        skeleton.joints.push_back(jointLeftElbow);
        skeleton.joints.push_back(jointLeftHand);
        skeleton.joints.push_back(jointRightShoulder);
        skeleton.joints.push_back(jointRightElbow);
        skeleton.joints.push_back(jointRightHand);
        skeleton.joints.push_back(jointLeftHip);
        skeleton.joints.push_back(jointLeftKnee);
        skeleton.joints.push_back(jointLeftFoot);
        skeleton.joints.push_back(jointRightHip);
        skeleton.joints.push_back(jointRightKnee);
        skeleton.joints.push_back(jointRightFoot);

        skeletons.skeletons.push_back(skeleton);
    }

    pubSkeletons.publish(skeletons);
}

#define CHECK_RC(nRetVal, what)                                     \
    if (nRetVal != XN_STATUS_OK)                                    \
    {                                                               \
        ROS_ERROR("%s failed: %s", what, xnGetStatusString(nRetVal));\
        return nRetVal;                                             \
    }

int initOpenNIContext(){
    string configFilename = ros::package::getPath("skeleton_finder") + "/openni_tracker.xml";
    xn::EnumerationErrors errors;
    XnStatus nRetVal = g_Context.InitFromXmlFile(configFilename.c_str(), g_scriptNode, &errors);
    CHECK_RC(nRetVal, "InitFromXml");

    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
    CHECK_RC(nRetVal, "Find depth generator");

    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
    if (nRetVal != XN_STATUS_OK) {
        nRetVal = g_UserGenerator.Create(g_Context);
        if (nRetVal != XN_STATUS_OK) {
            ROS_ERROR("NITE is likely missing: Please install NITE >= 1.5.2.21. Check the readme for download information. Error Info: User generator failed: %s", xnGetStatusString(nRetVal));
            return nRetVal;
        }
    }

    XnCallbackHandle hUserCallbacks, hCalibrationStart, hCalibrationComplete, hPoseDetected;
    if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
        ROS_INFO("Supplied user generator doesn't support skeleton");
        return 1;
    }
    nRetVal = g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);
    CHECK_RC(nRetVal, "Register to user callbacks");    
    nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationStart(UserCalibration_CalibrationStart, NULL, hCalibrationStart);
    CHECK_RC(nRetVal, "Register to calibration start");    
    nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationComplete(UserCalibration_CalibrationComplete, NULL, hCalibrationComplete);
    CHECK_RC(nRetVal, "Register to calibration complete");    

    if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration()) {
        g_bNeedPose = TRUE;
        if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
            ROS_INFO("Pose required, but not supported");
            return 1;
        }

        g_UserGenerator.GetPoseDetectionCap().RegisterToPoseDetected(UserPose_PoseDetected, NULL, hPoseDetected);
        CHECK_RC(nRetVal, "Register to Pose Detected");
        g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
    }

    g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

    nRetVal = g_Context.StartGeneratingAll();
    CHECK_RC(nRetVal, "StartGenerating");
    return 0;
}

void deleteOpenNIContext(){
    g_scriptNode.Release();
    g_DepthGenerator.Release();
    g_UserGenerator.Release();
    g_Context.Release();
}

void callbackEnableTracking(const std_msgs::Bool::ConstPtr& msg){
    if(msg->data){
        std::cout << "Traying start tracking skeleton" << std::endl;
        if(!hasAlreadyInitTracking){
            std::cout << "Staring tracking skeleton"<< std::endl;
            initOpenNIContext();
            hasAlreadyInitTracking = true;
        }else
            std::cout << "Has already start tracking skeleton" << std::endl;
    }
    else{
        std::cout << "Traying stop tracking skeleton" << std::endl;
        deleteOpenNIContext();
        hasAlreadyInitTracking = false;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "skeleton_finder_node");
    ros::NodeHandle nh;

    ros::Rate r(30);

    ros::NodeHandle pnh("~");
    frame_id = "kinect_link";
    ros::Subscriber subStartTracking = pnh.subscribe("/vision/skeleton_finder/enable_tracking", 1, callbackEnableTracking);
    pubSkeletons = pnh.advertise<vision_msgs::Skeletons>("/vision/skeleton_finder/skeleton_recog", 1);
    transformListener = new tf::TransformListener();
    transformListener->waitForTransform("/map", frame_id,
                              ros::Time::now(), ros::Duration(0.0));

    while (ros::ok()) {
        if(hasAlreadyInitTracking){
            g_Context.WaitAndUpdateAll();
            publishTransforms(frame_id);
        }
        r.sleep();
        ros::spinOnce();
    }

    deleteOpenNIContext();

    return 0;
}
