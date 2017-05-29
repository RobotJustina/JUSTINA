#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

using std::string;

xn::Context        g_Context;
xn::ScriptNode g_scriptNode;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator  g_UserGenerator;

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

void publishTransform(XnUserID const& user, XnSkeletonJoint const& joint, string const& frame_id, string const& child_frame_id) {
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
    //frame_rotation.setEulerZYX(3.1416, 0, 0);
    frame_rotation.setEuler(0, 0, M_PI);

    change_frame.setRotation(frame_rotation);

    transform = change_frame * transform;

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_no));
}

void publishTransforms(const std::string& frame_id) {
    XnUserID users[15];
    XnUInt16 users_count = 15;
    g_UserGenerator.GetUsers(users, users_count);

    for (int i = 0; i < users_count; ++i) {
        XnUserID user = users[i];
        if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
            continue;


        publishTransform(user, XN_SKEL_HEAD,           frame_id, "head");
        publishTransform(user, XN_SKEL_NECK,           frame_id, "neck");
        publishTransform(user, XN_SKEL_TORSO,          frame_id, "torso");

        publishTransform(user, XN_SKEL_LEFT_SHOULDER,  frame_id, "left_shoulder");
        publishTransform(user, XN_SKEL_LEFT_ELBOW,     frame_id, "left_elbow");
        publishTransform(user, XN_SKEL_LEFT_HAND,      frame_id, "left_hand");

        publishTransform(user, XN_SKEL_RIGHT_SHOULDER, frame_id, "right_shoulder");
        publishTransform(user, XN_SKEL_RIGHT_ELBOW,    frame_id, "right_elbow");
        publishTransform(user, XN_SKEL_RIGHT_HAND,     frame_id, "right_hand");

        publishTransform(user, XN_SKEL_LEFT_HIP,       frame_id, "left_hip");
        publishTransform(user, XN_SKEL_LEFT_KNEE,      frame_id, "left_knee");
        publishTransform(user, XN_SKEL_LEFT_FOOT,      frame_id, "left_foot");

        publishTransform(user, XN_SKEL_RIGHT_HIP,      frame_id, "right_hip");
        publishTransform(user, XN_SKEL_RIGHT_KNEE,     frame_id, "right_knee");
        publishTransform(user, XN_SKEL_RIGHT_FOOT,     frame_id, "right_foot");
    }
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
}

void deleteOpenNIContext(){
    g_scriptNode.Release();
    g_DepthGenerator.Release();
    g_UserGenerator.Release();
    g_Context.Release();
}

void callbackStartTracking(const std_msgs::Empty::ConstPtr& msg){
    std::cout << "Traying start tracking skeleton" << std::endl;
    if(!hasAlreadyInitTracking){
        std::cout << "Staring tracking skeleton"<< std::endl;
        initOpenNIContext();
        hasAlreadyInitTracking = true;
    }else
        std::cout << "Has already start tracking skeleton" << std::endl;

}

void callbackStopTracking(const std_msgs::Empty::ConstPtr& msg){
    std::cout << "Traying stop tracking skeleton" << std::endl;
    deleteOpenNIContext();
    hasAlreadyInitTracking = false;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "skeleton_finder_node");
    ros::NodeHandle nh;

    //initOpenNIContext();

    ros::Rate r(30);

    ros::NodeHandle pnh("~");
    string frame_id("kinect_link");
    ros::Subscriber subStartTracking = pnh.subscribe("/vision/skeleton_finder/start_tracking", 1, callbackStartTracking);
    ros::Subscriber subStopTracking = pnh.subscribe("/vision/skeleton_finder/stop_tracking", 1, callbackStopTracking);

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
