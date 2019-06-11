#include "ManipPln.h"

ManipPln::ManipPln()
{
    this->laNewGoal = false;
    this->raNewGoal = false;
    this->hdNewGoal = false;
    this->laFeedbackNewGoal = false;
    this->raFeedbackNewGoal = false;
    this->laNewGoalTraj = false;
    this->raNewGoalTraj = false;
}

ManipPln::~ManipPln()
{
}

void ManipPln::setNodeHandle(ros::NodeHandle* n)
{
    std::cout << "ManipPln.->Setting ros node..." << std::endl;
    this->nh = n;
    //Publishers for indicating that a goal pose has been reached
    this->pubLaGoalReached = nh->advertise<std_msgs::Bool>("/manipulation/la_goal_reached", 1);
    this->pubRaGoalReached = nh->advertise<std_msgs::Bool>("/manipulation/ra_goal_reached", 1);
    this->pubHdGoalReached = nh->advertise<std_msgs::Bool>("/manipulation/hd_goal_reached", 1);
    this->pubStartGetGripperPosition = nh->advertise<std_msgs::Bool>("/vision/obj_reco/start_gripper_position", 1);
    //Subscribers for the commands executed by this node
    this->subLaGoToAngles = nh->subscribe("/manipulation/manip_pln/la_goto_angles", 1, &ManipPln::callbackLaGoToAngles, this);
    this->subRaGoToAngles = nh->subscribe("/manipulation/manip_pln/ra_goto_angles", 1, &ManipPln::callbackRaGoToAngles, this);
    this->subHdGoToAngles = nh->subscribe("/manipulation/manip_pln/hd_goto_angles", 1, &ManipPln::callbackHdGoToAngles, this);
    this->subLaGoToPoseWrtArm = nh->subscribe("/manipulation/manip_pln/la_pose_wrt_arm", 1, &ManipPln::callbackLaGoToPoseWrtArm, this);
    this->subRaGoToPoseWrtArm = nh->subscribe("/manipulation/manip_pln/ra_pose_wrt_arm", 1, &ManipPln::callbackRaGoToPoseWrtArm, this);
    this->subLaGoToPoseWrtRobot = nh->subscribe("/manipulation/manip_pln/la_pose_wrt_robot", 1, &ManipPln::callbackLaGoToPoseWrtRobot, this);
    this->subRaGoToPoseWrtRobot = nh->subscribe("/manipulation/manip_pln/ra_pose_wrt_robot", 1, &ManipPln::callbackRaGoToPoseWrtRobot, this);
    this->subLaGoToPoseWrtArmFeedback = nh->subscribe("/manipulation/manip_pln/la_pose_wrt_arm_feedback", 1, &ManipPln::callbackLaGoToPoseWrtArmFeedback, this);
    this->subRaGoToPoseWrtArmFeedback = nh->subscribe("/manipulation/manip_pln/ra_pose_wrt_arm_feedback", 1, &ManipPln::callbackRaGoToPoseWrtArmFeedback, this);
    this->subLaGoToPoseWrtRobotFeedback = nh->subscribe("/manipulation/manip_pln/la_pose_wrt_robot_feedback", 1, &ManipPln::callbackLaGoToPoseWrtRobotFeedback, this);
    this->subRaGoToPoseWrtRobotFeedback = nh->subscribe("/manipulation/manip_pln/ra_pose_wrt_robot_feedback", 1, &ManipPln::callbackRaGoToPoseWrtRobotFeedback, this);
    this->subLaStopGoTo = nh->subscribe("/manipulation/manip_pln/la_stop", 1, &ManipPln::callbackLaStopGoTo, this);
    this->subRaStopGoTo = nh->subscribe("/manipulation/manip_pln/ra_stop", 1, &ManipPln::callbackRaStopGoTo, this);
    this->subLaGoToPoseWrtArmTraj = nh->subscribe("/manipulation/manip_pln/la_pose_wrt_arm_traj", 1, &ManipPln::callbackLaGoToPoseWrtArmTraj, this);
    this->subRaGoToPoseWrtArmTraj = nh->subscribe("/manipulation/manip_pln/ra_pose_wrt_arm_traj", 1, &ManipPln::callbackRaGoToPoseWrtArmTraj, this);
    this->subLaGoToPoseWrtRobotTraj = nh->subscribe("/manipulation/manip_pln/la_pose_wrt_robot_traj", 1, &ManipPln::callbackLaGoToPoseWrtRobotTraj, this);
    this->subRaGoToPoseWrtRobotTraj = nh->subscribe("/manipulation/manip_pln/ra_pose_wrt_robot_traj", 1, &ManipPln::callbackRaGoToPoseWrtRobotTraj, this);
    this->subLaGoToLoc = nh->subscribe("/manipulation/manip_pln/la_goto_loc", 1, &ManipPln::callbackLaGoToLoc, this);
    this->subRaGoToLoc = nh->subscribe("/manipulation/manip_pln/ra_goto_loc", 1, &ManipPln::callbackRaGoToLoc, this);
    this->subHdGoToLoc = nh->subscribe("/manipulation/manip_pln/hd_goto_loc", 1, &ManipPln::callbackHdGoToLoc, this);
    this->subLaMove = nh->subscribe("/manipulation/manip_pln/la_move", 1, &ManipPln::callbackLaMove, this);
    this->subRaMove = nh->subscribe("/manipulation/manip_pln/ra_move", 1, &ManipPln::callbackRaMove, this);
    this->subHdMove = nh->subscribe("/manipulation/manip_pln/hd_move", 1, &ManipPln::callbackHdMove, this);
    this->subGripperPosition = nh->subscribe("/vision/obj_reco/gripper_position", 1, &ManipPln::callbackgPos, this);
    //Publishers and subscribers for operating the hardware nodes
    this->subLaCurrentPose = nh->subscribe("/hardware/left_arm/current_pose", 1, &ManipPln::callbackLaCurrentPose, this);
    this->subRaCurrentPose = nh->subscribe("/hardware/right_arm/current_pose", 1, &ManipPln::callbackRaCurrentPose, this);
    this->subHdCurrentPose = nh->subscribe("/hardware/head/current_pose", 1, &ManipPln::callbackHdCurrentPose, this);
    this->pubLaGoalPose = nh->advertise<std_msgs::Float32MultiArray>("/hardware/left_arm/goal_pose", 1);
    this->pubRaGoalPose = nh->advertise<std_msgs::Float32MultiArray>("/hardware/right_arm/goal_pose", 1);
    this->pubHdGoalPose = nh->advertise<std_msgs::Float32MultiArray>("/hardware/head/goal_pose", 1);
    this->pubLaGoalTorque = nh->advertise<std_msgs::Float32MultiArray>("/hardware/left_arm/goal_torque", 1);
    this->pubRaGoalTorque = nh->advertise<std_msgs::Float32MultiArray>("/hardware/right_arm/goal_torque", 1);
    this->pubHdGoalTorque = nh->advertise<std_msgs::Float32MultiArray>("/hardware/head/goal_torque", 1);
    //Publishers of the marker manipulation
    this->pubMarkerManipulation = nh->advertise<visualization_msgs::MarkerArray>("/manipulation/manip_marker", 1); 
    //Stuff for tranformations and inverse kinematics
    this->cltIkFloatArray = nh->serviceClient<manip_msgs::InverseKinematicsFloatArray>("/manipulation/ik_geometric/ik_float_array");
    this->cltIkFloatArrayWithoutOpt = nh->serviceClient<manip_msgs::InverseKinematicsFloatArray>("/manipulation/ik_geometric/ik_float_array_without_opt");
    this->cltIkPath = nh->serviceClient<manip_msgs::InverseKinematicsPath>("/manipulation/ik_geometric/ik_path");
    this->cltIkPose = nh->serviceClient<manip_msgs::InverseKinematicsPose>("/manipulation/ik_geometric/ik_pose");
    this->cltDK = nh->serviceClient<manip_msgs::DirectKinematics>("/manipulation/ik_geometric/direct_kinematics");
    this->tf_listener = new tf::TransformListener();
    idMarker = 0;
}

bool ManipPln::loadPredefinedPosesAndMovements(std::string folder)
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
        this->laPredefPoses[i->first] = i->second;
    }
    std::cout << "ManipPln.->Left arm predefined positions: " <<std::endl;
    for(std::map<std::string, std::vector<float> >::iterator i = this->laPredefPoses.begin(); i != this->laPredefPoses.end(); i++)
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
        this->raPredefPoses[i->first] = i->second;
    }
    std::cout << "ManipPln.->Right arm predefined positions: " <<std::endl;
    for(std::map<std::string, std::vector<float> >::iterator i = this->laPredefPoses.begin(); i != this->laPredefPoses.end(); i++)
    {
        std::cout << i->first << " ";
        for(int j=0; j < i->second.size(); j++)
            std::cout << i->second[j] << " ";
        std::cout << std::endl;
    }
    return true;
}

std::map<std::string, std::vector<float> > ManipPln::loadArrayOfFloats(std::string path)
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

std::map<std::string, std::vector<std::vector<float> > > loadArrayOfArrayOfFloats(std::string path)
{
    std::map<std::string, std::vector<std::vector<float> > > foo;
    return foo;
}

void ManipPln::spin()
{
    ros::Rate loop(15);
    std_msgs::Bool msgLaGoalReached;
    std_msgs::Bool msgRaGoalReached;
    std_msgs::Bool msgHdGoalReached;
    std_msgs::Float32MultiArray msgLaGoalPose;
    std_msgs::Float32MultiArray msgRaGoalPose;
    std_msgs::Float32MultiArray msgHdGoalPose;
    while(ros::ok())
    {

        if(this->laNewGoal)
        {
            float error = this->calculateError(this->laCurrentPose, this->laGoalPose);
            if(error < 0.07)
            {
                msgLaGoalReached.data = true;
                pubLaGoalReached.publish(msgLaGoalReached);
                this->laNewGoal = false;
            }
            else
            {
                msgLaGoalPose.data = this->laGoalPose;
                msgLaGoalPose.data.insert(msgLaGoalPose.data.end(), this->laGoalSpeeds.begin(), this->laGoalSpeeds.end());
                pubLaGoalPose.publish(msgLaGoalPose);
            }
        }
        if(this->raNewGoal)
        {
            float error = this->calculateError(this->raCurrentPose, this->raGoalPose);
            if(error < 0.07)
            {
                msgRaGoalReached.data = true;
                pubRaGoalReached.publish(msgRaGoalReached);
                this->raNewGoal = false;
            }
            else
            {
                msgRaGoalPose.data = this->raGoalPose;
                msgRaGoalPose.data.insert(msgRaGoalPose.data.end(), this->raGoalSpeeds.begin(), this->raGoalSpeeds.end());
                pubRaGoalPose.publish(msgRaGoalPose);
            }
        }
        if(this->hdNewGoal)
        {
            float error = this->calculateError(this->hdCurrentPose, this->hdGoalPose);
            if(error < 0.05)
            {
                msgHdGoalReached.data = true;
                pubHdGoalReached.publish(msgHdGoalReached);
                this->hdNewGoal = false;
            }
            else
            {
                msgHdGoalPose.data = this->hdGoalPose;
                pubHdGoalPose.publish(msgHdGoalPose);
            }
        }
        if(this->laFeedbackNewGoal){

            float curr_gripper_x, curr_gripper_y, curr_gripper_z;
            // std::cout << "ManipPln.-> compute the goal la goal with feedback" << std::endl;

            if(gPos.x == 0 && gPos.y == 0 && gPos.z == 0){
                // std::cout << "ManipPln.-> Not detect gripper position with vision." << std::endl;
                tf::StampedTransform transform;
                tf_listener->waitForTransform("left_arm_link0", "left_arm_grip_center", ros::Time(0), ros::Duration(10.0));
                tf_listener->lookupTransform("left_arm_link0", "left_arm_grip_center", ros::Time(0), transform);
                curr_gripper_x = transform.getOrigin().getX();
                curr_gripper_y = transform.getOrigin().getY();
                curr_gripper_z = transform.getOrigin().getZ();
            }
            else{
                /*std::cout << "ManipPln.-> Detect gripper position with vision." << std::endl;
                tf::StampedTransform t;
                tf::Vector3 p(gPos.x, gPos.y, gPos.z);
                tf_listener->waitForTransform("left_arm_link0", "base_link", ros::Time(0), ros::Duration(10.0));
                tf_listener->lookupTransform("left_arm_link0", "base_link", ros::Time(0), t);
                p = t * p;
                curr_gripper_x = p.x();
                curr_gripper_y = p.y();
                curr_gripper_z = p.z();
                std::cout << "ManipPln.-> Not detect gripper position with vision." << std::endl;*/
                tf::StampedTransform transform;
                tf_listener->waitForTransform("left_arm_link0", "left_arm_grip_center", ros::Time(0), ros::Duration(10.0));
                tf_listener->lookupTransform("left_arm_link0", "left_arm_grip_center", ros::Time(0), transform);
                curr_gripper_x = transform.getOrigin().getX();
                curr_gripper_y = transform.getOrigin().getY();
                curr_gripper_z = transform.getOrigin().getZ();
            }

            float error = sqrt(pow(this->lCarGoalPose[0] - curr_gripper_x, 2) + pow(this->lCarGoalPose[1] - curr_gripper_y, 2) + pow(this->lCarGoalPose[2] - curr_gripper_z, 2));
            float dis = error;
            if(error <= THR_MIN){
                std::cout << "ManipPln.->The error is < that the threshold." << std::endl;
                std_msgs::Bool msgGoalReached;
                msgGoalReached.data = true;
                this->pubLaGoalReached.publish(msgGoalReached);
                this->laFeedbackNewGoal = false;
                std_msgs::Bool msg;
                msg.data = false;
                this->pubStartGetGripperPosition.publish(msg); 
            }
            else{
                std_msgs::Float32MultiArray middle_goal_msgs;
                middle_goal_msgs.data  = this->lCarGoalPose;
                middle_goal_msgs.data[0] = this->lCarGoalPose[0];
                middle_goal_msgs.data[1] = (this->lCarGoalPose[1] + curr_gripper_y) / 2.0f;
                middle_goal_msgs.data[2] = (this->lCarGoalPose[2] + curr_gripper_z) / 2.0f;
                // std::cout << "ManipPln.-> Middle point: " << middle_goal_msgs.data[0] << "," << middle_goal_msgs.data[1] << "," << middle_goal_msgs.data[2] << std::endl;

                //Validating that the goal new pose is in the workspace.
                // std::cout << "ManipPln.->Calling service for inverse kinematics..." << std::endl;
                manip_msgs::InverseKinematicsFloatArray srv;
                srv.request.cartesian_pose.data = middle_goal_msgs.data;
                if(!this->cltIkFloatArrayWithoutOpt.call(srv)){
                    std::cout << "ManipPln.->Cannot calculate inverse kinematics for the requested cartesian pose :'( " << std::endl;
                    std_msgs::Bool msg;
                    msg.data = false;
                    this->pubStartGetGripperPosition.publish(msg); 
                    this->laFeedbackNewGoal = false;
                }
                else{
                    std::vector<float> laGoalPose = srv.response.articular_pose.data;
                    std::vector<float> laGoalSpeeds;
                    this->calculateOptimalSpeeds(curr_gripper_x, curr_gripper_y, curr_gripper_z, middle_goal_msgs.data[0], middle_goal_msgs.data[1], middle_goal_msgs.data[2], this->laCurrentPose, laGoalPose, laGoalSpeeds);
                    msgLaGoalPose.data = laGoalPose;
                    msgLaGoalPose.data.insert(msgLaGoalPose.data.end(), laGoalSpeeds.begin(), laGoalSpeeds.end());
                    pubLaGoalPose.publish(msgLaGoalPose);
                    visualization_msgs::Marker marker;
                    marker.header.frame_id = "left_arm_link0";
                    marker.header.stamp = ros::Time();
                    marker.ns = "goal_marker";
                    marker.id = idMarker++;
                    marker.type = visualization_msgs::Marker::SPHERE;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.pose.position.x = middle_goal_msgs.data[0];
                    marker.pose.position.y = middle_goal_msgs.data[1];
                    marker.pose.position.z = middle_goal_msgs.data[2];
                    marker.pose.orientation.x = 0.0;
                    marker.pose.orientation.y = 0.0;
                    marker.pose.orientation.z = 0.0;
                    marker.pose.orientation.w = 1.0;
                    marker.scale.x = 0.04;
                    marker.scale.y = 0.04;
                    marker.scale.z = 0.04;
                    marker.color.a = 1.0;
                    marker.color.r = 0.0f;
                    marker.color.g = 0.0f;
                    marker.color.b = 0.6f;
                    manipMarker.markers.push_back(marker);
                }
            }
            
        }

        if(this->raFeedbackNewGoal){

            float curr_gripper_x, curr_gripper_y, curr_gripper_z;
            // std::cout << "ManipPln.-> compute the goal la goal with feedback" << std::endl;

            if(gPos.x == 0 && gPos.y == 0 && gPos.z == 0){
                // std::cout << "ManipPln.-> Not detect gripper position with vision." << std::endl;
                tf::StampedTransform transform;
                tf_listener->waitForTransform("right_arm_link0", "right_arm_grip_center", ros::Time(0), ros::Duration(10.0));
                tf_listener->lookupTransform("right_arm_link0", "right_arm_grip_center", ros::Time(0), transform);
                curr_gripper_x = transform.getOrigin().getX();
                curr_gripper_y = transform.getOrigin().getY();
                curr_gripper_z = transform.getOrigin().getZ();
            }
            else{
                /*std::cout << "ManipPln.-> Detect gripper position with vision." << std::endl;
                tf::StampedTransform t;
                tf::Vector3 p(gPos.x, gPos.y, gPos.z);
                tf_listener->waitForTransform("right_arm_link0", "base_link", ros::Time(0), ros::Duration(10.0));
                tf_listener->lookupTransform("right_arm_link0", "base_link", ros::Time(0), t);
                p = t * p;
                curr_gripper_x = p.x();
                curr_gripper_y = p.y();
                curr_gripper_z = p.z();
                std::cout << "ManipPln.-> Not detect gripper position with vision." << std::endl;*/
                tf::StampedTransform transform;
                tf_listener->waitForTransform("right_arm_link0", "right_arm_grip_center", ros::Time(0), ros::Duration(10.0));
                tf_listener->lookupTransform("right_arm_link0", "right_arm_grip_center", ros::Time(0), transform);
                curr_gripper_x = transform.getOrigin().getX();
                curr_gripper_y = transform.getOrigin().getY();
                curr_gripper_z = transform.getOrigin().getZ();
            }

            float error = sqrt(pow(this->rCarGoalPose[0] - curr_gripper_x, 2) + pow(this->rCarGoalPose[1] - curr_gripper_y, 2) + pow(this->rCarGoalPose[2] - curr_gripper_z, 2));
            float dis = error;
            if(error <= THR_MIN){
                // std::cout << "ManipPln.->The error is < that the threshold." << std::endl;
                std_msgs::Bool msgGoalReached;
                msgGoalReached.data = true;
                this->pubRaGoalReached.publish(msgGoalReached);
                this->raFeedbackNewGoal = false;
                std_msgs::Bool msg;
                msg.data = false;
                this->pubStartGetGripperPosition.publish(msg); 
            }
            else{
                std_msgs::Float32MultiArray middle_goal_msgs;
                middle_goal_msgs.data  = this->rCarGoalPose;
                middle_goal_msgs.data[0] = this->rCarGoalPose[0];
                middle_goal_msgs.data[1] = (this->rCarGoalPose[1] + curr_gripper_y) / 2.0f;
                middle_goal_msgs.data[2] = (this->rCarGoalPose[2] + curr_gripper_z) / 2.0f;
                std::cout << "ManipPln.-> Middle point: " << middle_goal_msgs.data[0] << "," << middle_goal_msgs.data[1] << "," << middle_goal_msgs.data[2] << std::endl;

                //Validating that the goal new pose is in the workspace.
                // std::cout << "ManipPln.->Calling service for inverse kinematics..." << std::endl;
                manip_msgs::InverseKinematicsFloatArray srv;
                srv.request.cartesian_pose.data = middle_goal_msgs.data;
                if(!this->cltIkFloatArrayWithoutOpt.call(srv)){
                    std::cout << "ManipPln.->Cannot calculate inverse kinematics for the requested cartesian pose :'( " << std::endl;
                    std_msgs::Bool msg;
                    msg.data = false;
                    this->pubStartGetGripperPosition.publish(msg); 
                    this->raFeedbackNewGoal = false;
                }
                else{
                    std::vector<float> raGoalPose = srv.response.articular_pose.data;
                    std::vector<float> raGoalSpeeds;
                    this->calculateOptimalSpeeds(curr_gripper_x, curr_gripper_y, curr_gripper_z, middle_goal_msgs.data[0], middle_goal_msgs.data[1], middle_goal_msgs.data[2], this->raCurrentPose, raGoalPose, raGoalSpeeds);
                    msgRaGoalPose.data = raGoalPose;
                    msgRaGoalPose.data.insert(msgRaGoalPose.data.end(), raGoalSpeeds.begin(), raGoalSpeeds.end());
                    pubRaGoalPose.publish(msgRaGoalPose);
                    visualization_msgs::Marker marker;
                    marker.header.frame_id = "right_arm_link0";
                    marker.header.stamp = ros::Time();
                    marker.ns = "goal_marker";
                    marker.id = idMarker++;
                    marker.type = visualization_msgs::Marker::SPHERE;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.pose.position.x = middle_goal_msgs.data[0];
                    marker.pose.position.y = middle_goal_msgs.data[1];
                    marker.pose.position.z = middle_goal_msgs.data[2];
                    marker.pose.orientation.x = 0.0;
                    marker.pose.orientation.y = 0.0;
                    marker.pose.orientation.z = 0.0;
                    marker.pose.orientation.w = 1.0;
                    marker.scale.x = 0.04;
                    marker.scale.y = 0.04;
                    marker.scale.z = 0.04;
                    marker.color.a = 1.0;
                    marker.color.r = 0.0f;
                    marker.color.g = 0.0f;
                    marker.color.b = 0.6f;
                    manipMarker.markers.push_back(marker);
                }
            }
        }

        if(laNewGoalTraj){
            std::vector<float> topGoalArticular = lGoalArticularTraj[0];
            std::vector<float> topGoalCartesian = lGoalCartesianTraj[0];
            float curr_gripper_x, curr_gripper_y, curr_gripper_z;
            tf::StampedTransform transform;
            tf_listener->waitForTransform("left_arm_link0", "left_arm_grip_center", ros::Time(0), ros::Duration(10.0));
            tf_listener->lookupTransform("left_arm_link0", "left_arm_grip_center", ros::Time(0), transform);
            curr_gripper_x = transform.getOrigin().getX();
            curr_gripper_y = transform.getOrigin().getY();
            curr_gripper_z = transform.getOrigin().getZ();
            //float error = this->calculateError(this->laCurrentPose, topGoalArticular);
            //if(error <= 0.07){
            float error = sqrt(pow(topGoalCartesian[0] - curr_gripper_x, 2) + pow(topGoalCartesian[1] - curr_gripper_y, 2) + pow(topGoalCartesian[2] - curr_gripper_z, 2));
            if(error <= THR_MIN){
                lGoalArticularTraj.erase(lGoalArticularTraj.begin());
                lGoalCartesianTraj.erase(lGoalCartesianTraj.begin());
                if(lGoalArticularTraj.size() == 0){
                    laNewGoalTraj = false;
                    std_msgs::Bool msgGoalReached;
                    msgGoalReached.data = true;
                    this->pubLaGoalReached.publish(msgGoalReached);
                }
                else{
                    std::vector<float> laGoalSpeeds;
                    std::vector<float> newGoalArticular = lGoalArticularTraj[0];
                    std::vector<float> newGoalCartesian = lGoalCartesianTraj[0];
                    this->calculateOptimalSpeeds(curr_gripper_x, curr_gripper_y, curr_gripper_z, newGoalCartesian[0], newGoalCartesian[1], newGoalCartesian[2], this->laCurrentPose, newGoalArticular, laGoalSpeeds);
                    msgLaGoalPose.data = newGoalArticular;
                    msgLaGoalPose.data.insert(msgLaGoalPose.data.end(), laGoalSpeeds.begin(), laGoalSpeeds.end());
                    pubLaGoalPose.publish(msgLaGoalPose);
                }
            }
        }
        
        if(raNewGoalTraj){
            std::vector<float> topGoalArticular = rGoalArticularTraj[0];
            std::vector<float> topGoalCartesian = rGoalCartesianTraj[0];
            float curr_gripper_x, curr_gripper_y, curr_gripper_z;
            tf::StampedTransform transform;
            tf_listener->waitForTransform("right_arm_link0", "right_arm_grip_center", ros::Time(0), ros::Duration(10.0));
            tf_listener->lookupTransform("right_arm_link0", "right_arm_grip_center", ros::Time(0), transform);
            curr_gripper_x = transform.getOrigin().getX();
            curr_gripper_y = transform.getOrigin().getY();
            curr_gripper_z = transform.getOrigin().getZ();
            //float error = this->calculateError(this->laCurrentPose, topGoalArticular);
            //if(error <= 0.07){
            float error = sqrt(pow(topGoalCartesian[0] - curr_gripper_x, 2) + pow(topGoalCartesian[1] - curr_gripper_y, 2) + pow(topGoalCartesian[2] - curr_gripper_z, 2));
            if(error <= THR_MIN){
                rGoalArticularTraj.erase(rGoalArticularTraj.begin());
                rGoalCartesianTraj.erase(rGoalCartesianTraj.begin());
                if(rGoalArticularTraj.size() == 0){
                    raNewGoalTraj = false;
                    std_msgs::Bool msgGoalReached;
                    msgGoalReached.data = true;
                    this->pubRaGoalReached.publish(msgGoalReached);
                }
                else{
                    std::vector<float> raGoalSpeeds;
                    std::vector<float> newGoalArticular = rGoalArticularTraj[0];
                    std::vector<float> newGoalCartesian = rGoalCartesianTraj[0];
                    this->calculateOptimalSpeeds(curr_gripper_x, curr_gripper_y, curr_gripper_z, newGoalCartesian[0], newGoalCartesian[1], newGoalCartesian[2], this->raCurrentPose, newGoalArticular, raGoalSpeeds);
                    msgRaGoalPose.data = newGoalArticular;
                    msgRaGoalPose.data.insert(msgRaGoalPose.data.end(), raGoalSpeeds.begin(), raGoalSpeeds.end());
                    pubRaGoalPose.publish(msgRaGoalPose);
                }
            }
        }

        pubMarkerManipulation.publish(manipMarker);
        gPos.x = 0;
        gPos.y = 0;
        gPos.z = 0;
        ros::spinOnce();
        loop.sleep();
    }
}

float ManipPln::calculateError(std::vector<float>& v1, std::vector<float>& v2)
{
    float max = 0;
    for(int i=0; i < v1.size(); i++)
    {
        float temp = fabs(v1[i] - v2[i]);
        if(temp > max)
            max = temp;
    }
    return max;
}

void ManipPln::calculateOptimalSpeeds(std::vector<float>& currentPose, std::vector<float>& goalPose, std::vector<float>& speeds)
{
    float maxSpeed = 0.05;
    float minSpeed = 0.01;
    float maxDiff = 0;
    for(size_t i=0; i < currentPose.size(); i++)
        if(fabs(currentPose[i] - goalPose[i]) > maxDiff)
            maxDiff = fabs(currentPose[i] - goalPose[i]);

    speeds.clear();
    for(size_t i=0; i < currentPose.size(); i++)
        speeds.push_back(minSpeed);
    if(maxDiff == 0)
        return;

    for(size_t i=0; i < currentPose.size(); i++)
        speeds[i] = fabs(currentPose[i] - goalPose[i]) / maxDiff * maxSpeed;
    for(size_t i=0; i < speeds.size(); i++)
        if(speeds[i] < minSpeed)
            speeds[i] = minSpeed;
}

void ManipPln::calculateOptimalSpeeds(float currx, float curry, float currz, float goalx, float goaly, float goalz, std::vector<float> currentArtPose, std::vector<float> goalArtPose , std::vector<float>& speeds){
    float dis = sqrt(pow(goalx - currx, 2) + pow(goaly - curry, 2) + pow(goalz - currz, 2));
    float dismax = THR_MAX;
    if(dis >= dismax){
        this->calculateOptimalSpeeds(currentArtPose, goalArtPose, speeds);
    }else{
        speeds.clear();
        for(unsigned int i = 0; i < currentArtPose.size(); i++)
            speeds.push_back(0.015);
    }
}

//
//Callback for subscribers for the commands executed by this node
//

void ManipPln::callbackLaGoToAngles(std_msgs::Float32MultiArray::Ptr msg)
{
    std::cout << "ManipPln.->Received Left Arm goal pose: ";
    for(int i=0; i< msg->data.size(); i++)
        std::cout << msg->data[i] << " ";
    std::cout << std::endl;

    if(msg->data.size() != 7 && msg->data.size() != 6 && msg->data.size() != 3)
    {
        std::cout << "ManipPln.->LaGoalPose must have 3, 6 or 7 values. " << std::endl;
        return;
    }

    while(msg->data.size() < 7) //All non-specified angles are considered to be zero
        msg->data.push_back(0);

    std_msgs::Bool msgGoalReached;
    msgGoalReached.data = false;
    this->pubLaGoalReached.publish(msgGoalReached);
    this->laGoalPose = msg->data;
    this->calculateOptimalSpeeds(this->laCurrentPose, this->laGoalPose, this->laGoalSpeeds);
    this->laNewGoal = true;
}

void ManipPln::callbackRaGoToAngles(std_msgs::Float32MultiArray::Ptr msg)
{
    std::cout << "ManipPln.->Received Right Arm goal pose: ";
    for(int i=0; i< msg->data.size(); i++)
        std::cout << msg->data[i] << " ";
    std::cout << std::endl;

    if(msg->data.size() != 7 && msg->data.size() != 6 && msg->data.size() != 3)
    {
        std::cout << "ManipPln.->RaGoalPose must have 3, 6 or 7 values. " << std::endl;
        return;
    }

    while(msg->data.size() < 7) //All non-specified angles are considered to be zero
        msg->data.push_back(0);

    std_msgs::Bool msgGoalReached;
    msgGoalReached.data = false;
    this->pubRaGoalReached.publish(msgGoalReached);
    this->raGoalPose = msg->data;
    this->calculateOptimalSpeeds(this->raCurrentPose, this->raGoalPose, this->raGoalSpeeds);
    this->raNewGoal = true;
}

void ManipPln::callbackHdGoToAngles(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data.size() != 2)
    {
        std::cout << "ManipPln.->HeadGoalPose must have 2 values. " << std::endl;
        return;
    }
    //std::cout << "ManipPln.->Head goal pose: ";
    //for(int i=0; i< 2; i++)
    //    std::cout << msg->data[i] << " ";
    //std::cout << std::endl;

    std_msgs::Bool msgGoalReached;
    msgGoalReached.data = false;
    this->pubHdGoalReached.publish(msgGoalReached);
    this->hdGoalPose = msg->data;
    this->hdNewGoal = true;
}

void ManipPln::callbackLaGoToPoseWrtArm(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    std::cout << "ManipPln.->Received Left arm goal pose (wrt arm): ";
    for(int i=0; i< msg->data.size(); i++)
        std::cout << msg->data[i] << " ";
    std::cout << std::endl;
    if(msg->data.size() != 7 && msg->data.size() != 6 && msg->data.size() != 3)
    {
        std::cout << "ManipPln.->Pose must have 3 (xyz), 6 (xyz-rpy) or 7 (xyz-rpy-e) values. Sorry. " << std::endl;
        return;
    }

    std::cout << "ManipPln.->Calling service for inverse kinematics..." << std::endl;
    manip_msgs::InverseKinematicsFloatArray srv;
    srv.request.cartesian_pose.data = msg->data;
    if(!this->cltIkFloatArray.call(srv))
    {
        std::cout << "ManipPln.->Cannot calculate inverse kinematics for the requested cartesian pose :'( " << std::endl;
        return;
    }
    std_msgs::Bool msgGoalReached;
    msgGoalReached.data = false;
    this->pubLaGoalReached.publish(msgGoalReached);
    this->laGoalPose = srv.response.articular_pose.data;
    this->calculateOptimalSpeeds(this->laCurrentPose, this->laGoalPose, this->laGoalSpeeds);
    this->laNewGoal = true;
}

void ManipPln::callbackRaGoToPoseWrtArm(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    std::cout << "ManipPln.->Received Right arm goal pose (wrt arm): ";
    for(int i=0; i< msg->data.size(); i++)
        std::cout << msg->data[i] << " ";
    std::cout << std::endl;
    if(msg->data.size() != 7 && msg->data.size() != 6 && msg->data.size() != 3)
    {
        std::cout << "ManipPln.->Pose must have 3 (xyz), 6 (xyz-rpy) or 7 (xyz-rpy-e) values. Sorry. " << std::endl;
        return;
    }

    std::cout << "ManipPln.->Calling service for inverse kinematics..." << std::endl;
    manip_msgs::InverseKinematicsFloatArray srv;
    srv.request.cartesian_pose.data = msg->data;
    if(!this->cltIkFloatArray.call(srv))
    {
        std::cout << "ManipPln.->Cannot calculate inverse kinematics for the requested cartesian pose :'( " << std::endl;
        return;
    }
    std_msgs::Bool msgGoalReached;
    msgGoalReached.data = false;
    this->pubRaGoalReached.publish(msgGoalReached);
    this->raGoalPose = srv.response.articular_pose.data;
    this->calculateOptimalSpeeds(this->raCurrentPose, this->raGoalPose, this->raGoalSpeeds);
    this->raNewGoal = true;
}

void ManipPln::callbackLaGoToPoseWrtRobot(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    std::cout << "ManipPln.->Received Left arm Goal Pose (wrt robot): " << std::endl;
    for(int i=0; i< msg->data.size(); i++)
        std::cout << msg->data[i] << " ";
    std::cout << std::endl;
    if(msg->data.size() != 7 && msg->data.size() != 6 && msg->data.size() != 3)
    {
        std::cout << "ManipPln.->Pose must have 3 (xyz), 6 (xyz-rpy) or 7 (xyz-rpy-e) values. Sorry. " << std::endl;
        return;
    }

    tf::StampedTransform ht;
    this->tf_listener->lookupTransform("left_arm_link0", "base_link", ros::Time(0), ht);

    tf::Vector3 p(msg->data[0], msg->data[1], msg->data[2]);
    tf::Quaternion q;
    if(msg->data.size() > 3)
        q.setRPY(msg->data[3], msg->data[4], msg->data[5]);
    else
        q.setRPY(0,0,0);

    p = ht * p; //These two lines make the transform
    q = ht * q;

    double dRoll, dPitch, dYaw;
    tf::Matrix3x3(q).getRPY(dRoll, dPitch, dYaw);

    std_msgs::Float32MultiArray msg_;
    msg_.data.push_back(p.x());
    msg_.data.push_back(p.y());
    msg_.data.push_back(p.z());
    if(msg->data.size() > 3)
    {
        msg_.data.push_back((float)dRoll);
        msg_.data.push_back((float)dPitch);
        msg_.data.push_back((float)dYaw);
    }
    if(msg->data.size() == 7)
        msg_.data.push_back(msg->data[6]);

    const std_msgs::Float32MultiArray::ConstPtr msgptr(new std_msgs::Float32MultiArray(msg_));
    this->callbackLaGoToPoseWrtArm(msgptr);
}

void ManipPln::callbackRaGoToPoseWrtRobot(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    std::cout << "ManipPln.->Received Right arm Goal Pose (wrt robot): " << std::endl;
    for(int i=0; i< msg->data.size(); i++)
        std::cout << msg->data[i] << " ";
    std::cout << std::endl;
    if(msg->data.size() != 7 && msg->data.size() != 6 && msg->data.size() != 3)
    {
        std::cout << "ManipPln.->Pose must have 3 (xyz), 6 (xyz-rpy) or 7 (xyz-rpy-e) values. Sorry. " << std::endl;
        return;
    }

    tf::StampedTransform ht;
    this->tf_listener->lookupTransform("right_arm_link0", "base_link", ros::Time(0), ht);

    tf::Vector3 p(msg->data[0], msg->data[1], msg->data[2]);
    tf::Quaternion q;
    if(msg->data.size() > 3)
        q.setRPY(msg->data[3], msg->data[4], msg->data[5]);
    else
        q.setRPY(0,0,0);

    p = ht * p; //These two lines make the transform
    q = ht * q;

    double dRoll, dPitch, dYaw;
    tf::Matrix3x3(q).getRPY(dRoll, dPitch, dYaw);

    std_msgs::Float32MultiArray msg_;
    msg_.data.push_back(p.x());
    msg_.data.push_back(p.y());
    msg_.data.push_back(p.z());
    if(msg->data.size() > 3)
    {
        msg_.data.push_back((float)dRoll);
        msg_.data.push_back((float)dPitch);
        msg_.data.push_back((float)dYaw);
    }
    if(msg->data.size() == 7)
        msg_.data.push_back(msg->data[6]);

    const std_msgs::Float32MultiArray::ConstPtr msgptr(new std_msgs::Float32MultiArray(msg_));
    this->callbackRaGoToPoseWrtArm(msgptr);
}


void ManipPln::callbackLaGoToPoseWrtArmFeedback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    std::cout << "ManipPln.->Received Left arm goal pose (wrt arm) with feedback: ";
    for(int i=0; i< msg->data.size(); i++)
        std::cout << msg->data[i] << " ";
    std::cout << std::endl;
    if(msg->data.size() != 7 && msg->data.size() != 6 && msg->data.size() != 3)
    {
        std::cout << "ManipPln.->Pose must have 3 (xyz), 6 (xyz-rpy) or 7 (xyz-rpy-e) values. Sorry. " << std::endl;
        return;
    }

    //Validating that the goal pose is in the workspace.
    std::cout << "ManipPln.->Calling service for inverse kinematics..." << std::endl;
    manip_msgs::InverseKinematicsFloatArray srv;
    srv.request.cartesian_pose.data = msg->data;
    if(!this->cltIkFloatArrayWithoutOpt.call(srv)){
        std::cout << "ManipPln.->Cannot calculate inverse kinematics for the requested cartesian pose :'( " << std::endl;
        this->laFeedbackNewGoal = false;
        std_msgs::Bool msgGetGripper;
        msgGetGripper.data = false;
        this->pubStartGetGripperPosition.publish(msgGetGripper); 
    }
    else{
        visualization_msgs::Marker marker;
        marker.header.frame_id = "left_arm_link0";
        marker.header.stamp = ros::Time();
        marker.ns = "goal_marker";
        marker.id = idMarker++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = msg->data[0];
        marker.pose.position.y = msg->data[1];
        marker.pose.position.z = msg->data[2];
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.02;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        marker.color.a = 1.0;
        marker.color.r = 0.6f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        manipMarker.markers.push_back(marker);
        lCarGoalPose = msg->data;
        this->laFeedbackNewGoal = true;
        std_msgs::Bool msgGetGripper;
        msgGetGripper.data = true;
        this->pubStartGetGripperPosition.publish(msgGetGripper);
    }
}

void ManipPln::callbackRaGoToPoseWrtArmFeedback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    std::cout << "ManipPln.->Received Right arm goal pose (wrt arm) with feedback: ";
    for(int i=0; i< msg->data.size(); i++)
        std::cout << msg->data[i] << " ";
    std::cout << std::endl;
    if(msg->data.size() != 7 && msg->data.size() != 6 && msg->data.size() != 3)
    {
        std::cout << "ManipPln.->Pose must have 3 (xyz), 6 (xyz-rpy) or 7 (xyz-rpy-e) values. Sorry. " << std::endl;
        return;
    }

    //Validating that the goal pose is in the workspace.
    std::cout << "ManipPln.->Calling service for inverse kinematics..." << std::endl;
    manip_msgs::InverseKinematicsFloatArray srv;
    srv.request.cartesian_pose.data = msg->data;
    if(!this->cltIkFloatArrayWithoutOpt.call(srv)){
        std::cout << "ManipPln.->Cannot calculate inverse kinematics for the requested cartesian pose :'( " << std::endl;
        this->raFeedbackNewGoal = false;
        std_msgs::Bool msgGetGripper;
        msgGetGripper.data = false;
        this->pubStartGetGripperPosition.publish(msgGetGripper); 
    }
    else{
        visualization_msgs::Marker marker;
        marker.header.frame_id = "right_arm_link0";
        marker.header.stamp = ros::Time();
        marker.ns = "goal_marker";
        marker.id = idMarker++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = msg->data[0];
        marker.pose.position.y = msg->data[1];
        marker.pose.position.z = msg->data[2];
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.02;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        marker.color.a = 1.0;
        marker.color.r = 0.6f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        manipMarker.markers.push_back(marker);
        rCarGoalPose = msg->data;
        this->raFeedbackNewGoal = true;
        std_msgs::Bool msgGetGripper;
        msgGetGripper.data = true;
        this->pubStartGetGripperPosition.publish(msgGetGripper);
    }
}

void ManipPln::callbackLaGoToPoseWrtRobotFeedback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    std::cout << "ManipPln.->Received Left arm Goal Pose (wrt robot) with feedback:" << std::endl;
    for(int i=0; i< msg->data.size(); i++)
        std::cout << msg->data[i] << " ";
    std::cout << std::endl;
    if(msg->data.size() != 7 && msg->data.size() != 6 && msg->data.size() != 3)
    {
        std::cout << "ManipPln.->Pose must have 3 (xyz), 6 (xyz-rpy) or 7 (xyz-rpy-e) values. Sorry. " << std::endl;
        return;
    }

    tf::StampedTransform ht;
    this->tf_listener->lookupTransform("left_arm_link0", "base_link", ros::Time(0), ht);

    tf::Vector3 p(msg->data[0], msg->data[1], msg->data[2]);
    tf::Quaternion q;
    if(msg->data.size() > 3)
        q.setRPY(msg->data[3], msg->data[4], msg->data[5]);
    else
        q.setRPY(0,0,0);

    p = ht * p; //These two lines make the transform
    q = ht * q;

    double dRoll, dPitch, dYaw;
    tf::Matrix3x3(q).getRPY(dRoll, dPitch, dYaw);

    std_msgs::Float32MultiArray msg_;
    msg_.data.push_back(p.x());
    msg_.data.push_back(p.y());
    msg_.data.push_back(p.z());
    if(msg->data.size() > 3)
    {
        msg_.data.push_back((float)dRoll);
        msg_.data.push_back((float)dPitch);
        msg_.data.push_back((float)dYaw);
    }
    if(msg->data.size() == 7)
        msg_.data.push_back(msg->data[6]);

    const std_msgs::Float32MultiArray::ConstPtr msgptr(new std_msgs::Float32MultiArray(msg_));
    this->callbackLaGoToPoseWrtArmFeedback(msgptr);
}

void ManipPln::callbackRaGoToPoseWrtRobotFeedback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    std::cout << "ManipPln.->Received Right arm Goal Pose (wrt robot) with feedback:" << std::endl;
    for(int i=0; i< msg->data.size(); i++)
        std::cout << msg->data[i] << " ";
    std::cout << std::endl;
    if(msg->data.size() != 7 && msg->data.size() != 6 && msg->data.size() != 3)
    {
        std::cout << "ManipPln.->Pose must have 3 (xyz), 6 (xyz-rpy) or 7 (xyz-rpy-e) values. Sorry. " << std::endl;
        return;
    }

    tf::StampedTransform ht;
    this->tf_listener->lookupTransform("right_arm_link0", "base_link", ros::Time(0), ht);

    tf::Vector3 p(msg->data[0], msg->data[1], msg->data[2]);
    tf::Quaternion q;
    if(msg->data.size() > 3)
        q.setRPY(msg->data[3], msg->data[4], msg->data[5]);
    else
        q.setRPY(0,0,0);

    p = ht * p; //These two lines make the transform
    q = ht * q;

    double dRoll, dPitch, dYaw;
    tf::Matrix3x3(q).getRPY(dRoll, dPitch, dYaw);

    std_msgs::Float32MultiArray msg_;
    msg_.data.push_back(p.x());
    msg_.data.push_back(p.y());
    msg_.data.push_back(p.z());
    if(msg->data.size() > 3)
    {
        msg_.data.push_back((float)dRoll);
        msg_.data.push_back((float)dPitch);
        msg_.data.push_back((float)dYaw);
    }
    if(msg->data.size() == 7)
        msg_.data.push_back(msg->data[6]);

    const std_msgs::Float32MultiArray::ConstPtr msgptr(new std_msgs::Float32MultiArray(msg_));
    this->callbackRaGoToPoseWrtArmFeedback(msgptr);
}

void ManipPln::callbackLaStopGoTo(const std_msgs::Empty::ConstPtr &msg){
    this->laFeedbackNewGoal = false;
    this->laNewGoalTraj = false;
    lGoalArticularTraj.clear();
    lGoalCartesianTraj.clear();
}

void ManipPln::callbackRaStopGoTo(const std_msgs::Empty::ConstPtr &msg){
    this->raFeedbackNewGoal = false;
    this->raNewGoalTraj = false;
    rGoalArticularTraj.clear();
    rGoalCartesianTraj.clear();
}

void ManipPln::callbackLaGoToPoseWrtArmTraj(const std_msgs::Float32MultiArray::ConstPtr& msg){
    std::cout << "ManipPln.->Received Left arm goal pose (wrt arm) with trajectory: ";
    for(int i=0; i< msg->data.size(); i++)
        std::cout << msg->data[i] << " ";
    std::cout << std::endl;
    if(msg->data.size() != 7 && msg->data.size() != 6 && msg->data.size() != 3)
    {
        std::cout << "ManipPln.->Pose must have 3 (xyz), 6 (xyz-rpy) or 7 (xyz-rpy-e) values. Sorry. " << std::endl;
        return;
    }
    lGoalArticularTraj.clear();
    lGoalCartesianTraj.clear();
    //Validating that the goal pose is in the workspace.
    std::cout << "ManipPln.->Calling service for inverse kinematics..." << std::endl;
    manip_msgs::InverseKinematicsFloatArray srv;
    srv.request.cartesian_pose.data = msg->data;
    if(!this->cltIkFloatArrayWithoutOpt.call(srv)){
        std::cout << "ManipPln.->Cannot calculate inverse kinematics for the requested cartesian pose :'( " << std::endl;
        this->laNewGoalTraj = false;
    }
    else{
        tf::StampedTransform transform;
        tf_listener->waitForTransform("left_arm_link0", "left_arm_grip_center", ros::Time(0), ros::Duration(10.0));
        tf_listener->lookupTransform("left_arm_link0", "left_arm_grip_center", ros::Time(0), transform);
        float curr_gripper_x = transform.getOrigin().getX();
        float curr_gripper_y = transform.getOrigin().getY();
        float curr_gripper_z = transform.getOrigin().getZ();
        float dx = msg->data[0] - curr_gripper_x;
        float dy = msg->data[1] - curr_gripper_y;
        float dz = msg->data[2] - curr_gripper_z;
        float norma = sqrt(dx * dx + dy * dy + dz * dz);

        int sf = 10;
        float x = curr_gripper_x;
        float y = curr_gripper_y;
        float z = curr_gripper_z;
        for(int s = 0; s < sf; s++){
            float xt = sqrt((msg->data[0] - x) * (msg->data[0] - x) + (msg->data[1] - y) * (msg->data[1] - y) + (msg->data[2] - z) * (msg->data[2] - z));
            if(xt <= THR_MIN * 1.5f) // if(xt <= THR_MIN) // This is for the old node of the head 
                break;
            float a3 = -2 * xt / pow((float) sf, 3);
            float a2 = 3 * xt / pow((float) sf, 2);
            float t = a2 * pow(s, 2) + a3 * pow(s, 3);
            x += dx / norma * t;
            y += dy / norma * t;
            z += dz / norma * t;
            visualization_msgs::Marker marker;
            marker.header.frame_id = "left_arm_link0";
            marker.header.stamp = ros::Time();
            marker.ns = "goal_marker";
            marker.id = idMarker++;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = z;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.02;
            marker.scale.y = 0.02;
            marker.scale.z = 0.02;
            marker.color.a = 1.0;
            marker.color.r = 0.6f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            manipMarker.markers.push_back(marker);

            std_msgs::Float32MultiArray nexPos;
            nexPos.data = msg->data;
            nexPos.data[0] = x;
            nexPos.data[1] = y;
            nexPos.data[2] = z; 
            srv.request.cartesian_pose.data = nexPos.data;
            if(!this->cltIkFloatArrayWithoutOpt.call(srv)){
                std::cout << "ManipPln.->Cannot calculate inverse kinematics for the requested cartesian pose :'( " << std::endl;
                this->lGoalArticularTraj.clear();
                this->lGoalCartesianTraj.clear();
                this->laNewGoalTraj = false;
                return;
            }
            lGoalArticularTraj.push_back(srv.response.articular_pose.data);
            std::vector<float> pose;
            pose.push_back(x);
            pose.push_back(y);
            pose.push_back(z);
            lGoalCartesianTraj.push_back(pose);
        }
        laNewGoalTraj = true;
    }
}

void ManipPln::callbackRaGoToPoseWrtArmTraj(const std_msgs::Float32MultiArray::ConstPtr& msg){
    std::cout << "ManipPln.->Received Right arm goal pose (wrt arm) with trajectory: ";
    for(int i=0; i< msg->data.size(); i++)
        std::cout << msg->data[i] << " ";
    std::cout << std::endl;
    if(msg->data.size() != 7 && msg->data.size() != 6 && msg->data.size() != 3)
    {
        std::cout << "ManipPln.->Pose must have 3 (xyz), 6 (xyz-rpy) or 7 (xyz-rpy-e) values. Sorry. " << std::endl;
        return;
    }

    rGoalArticularTraj.clear();
    rGoalCartesianTraj.clear();
    //Validating that the goal pose is in the workspace.
    std::cout << "ManipPln.->Calling service for inverse kinematics..." << std::endl;
    manip_msgs::InverseKinematicsFloatArray srv;
    srv.request.cartesian_pose.data = msg->data;
    if(!this->cltIkFloatArrayWithoutOpt.call(srv)){
        std::cout << "ManipPln.->Cannot calculate inverse kinematics for the requested cartesian pose :'( " << std::endl;
        this->raNewGoalTraj = false;
    }
    else{
        tf::StampedTransform transform;
        tf_listener->waitForTransform("right_arm_link0", "right_arm_grip_center", ros::Time(0), ros::Duration(10.0));
        tf_listener->lookupTransform("right_arm_link0", "right_arm_grip_center", ros::Time(0), transform);
        float curr_gripper_x = transform.getOrigin().getX();
        float curr_gripper_y = transform.getOrigin().getY();
        float curr_gripper_z = transform.getOrigin().getZ();
        float dx = msg->data[0] - curr_gripper_x;
        float dy = msg->data[1] - curr_gripper_y;
        float dz = msg->data[2] - curr_gripper_z;
        float norma = sqrt(dx * dx + dy * dy + dz * dz);

        int sf = 10;
        float x = curr_gripper_x;
        float y = curr_gripper_y;
        float z = curr_gripper_z;
        for(int s = 0; s < sf; s++){
            float xt = sqrt((msg->data[0] - x) * (msg->data[0] - x) + (msg->data[1] - y) * (msg->data[1] - y) + (msg->data[2] - z) * (msg->data[2] - z));
            if(xt <= THR_MIN * 1.5f) // if(xt <= THR_MIN) // This is for the old node of the head
                break;
            float a3 = -2 * xt / pow((float) sf, 3);
            float a2 = 3 * xt / pow((float) sf, 2);
            float t = a2 * pow(s, 2) + a3 * pow(s, 3);
            x += dx / norma * t;
            y += dy / norma * t;
            z += dz / norma * t;
            visualization_msgs::Marker marker;
            marker.header.frame_id = "right_arm_link0";
            marker.header.stamp = ros::Time();
            marker.ns = "goal_marker";
            marker.id = idMarker++;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = z;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.02;
            marker.scale.y = 0.02;
            marker.scale.z = 0.02;
            marker.color.a = 1.0;
            marker.color.r = 0.6f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            manipMarker.markers.push_back(marker);

            std_msgs::Float32MultiArray nexPos;
            nexPos.data = msg->data;
            nexPos.data[0] = x;
            nexPos.data[1] = y;
            nexPos.data[2] = z; 
            srv.request.cartesian_pose.data = nexPos.data;
            if(!this->cltIkFloatArrayWithoutOpt.call(srv)){
                std::cout << "ManipPln.->Cannot calculate inverse kinematics for the requested cartesian pose :'( " << std::endl;
                this->rGoalArticularTraj.clear();
                this->rGoalCartesianTraj.clear();
                this->raNewGoalTraj = false;
                return;
            }
            rGoalArticularTraj.push_back(srv.response.articular_pose.data);
            std::vector<float> pose;
            pose.push_back(x);
            pose.push_back(y);
            pose.push_back(z);
            rGoalCartesianTraj.push_back(pose);
        }
        raNewGoalTraj = true;
    }
}

void ManipPln::callbackLaGoToPoseWrtRobotTraj(const std_msgs::Float32MultiArray::ConstPtr& msg){
    std::cout << "ManipPln.->Received Left arm Goal Pose (wrt robot) with trajectory:" << std::endl;
    for(int i=0; i< msg->data.size(); i++)
        std::cout << msg->data[i] << " ";
    std::cout << std::endl;
    if(msg->data.size() != 7 && msg->data.size() != 6 && msg->data.size() != 3)
    {
        std::cout << "ManipPln.->Pose must have 3 (xyz), 6 (xyz-rpy) or 7 (xyz-rpy-e) values. Sorry. " << std::endl;
        return;
    }

    tf::StampedTransform ht;
    this->tf_listener->lookupTransform("left_arm_link0", "base_link", ros::Time(0), ht);

    tf::Vector3 p(msg->data[0], msg->data[1], msg->data[2]);
    tf::Quaternion q;
    if(msg->data.size() > 3)
        q.setRPY(msg->data[3], msg->data[4], msg->data[5]);
    else
        q.setRPY(0,0,0);

    p = ht * p; //These two lines make the transform
    q = ht * q;

    double dRoll, dPitch, dYaw;
    tf::Matrix3x3(q).getRPY(dRoll, dPitch, dYaw);

    std_msgs::Float32MultiArray msg_;
    msg_.data.push_back(p.x());
    msg_.data.push_back(p.y());
    msg_.data.push_back(p.z());
    if(msg->data.size() > 3)
    {
        msg_.data.push_back((float)dRoll);
        msg_.data.push_back((float)dPitch);
        msg_.data.push_back((float)dYaw);
    }
    if(msg->data.size() == 7)
        msg_.data.push_back(msg->data[6]);

    const std_msgs::Float32MultiArray::ConstPtr msgptr(new std_msgs::Float32MultiArray(msg_));
    this->callbackLaGoToPoseWrtArmTraj(msgptr);
}

void ManipPln::callbackRaGoToPoseWrtRobotTraj(const std_msgs::Float32MultiArray::ConstPtr& msg){
    std::cout << "ManipPln.->Received Right arm Goal Pose (wrt robot) with trajectory:" << std::endl;
    for(int i=0; i< msg->data.size(); i++)
        std::cout << msg->data[i] << " ";
    std::cout << std::endl;
    if(msg->data.size() != 7 && msg->data.size() != 6 && msg->data.size() != 3)
    {
        std::cout << "ManipPln.->Pose must have 3 (xyz), 6 (xyz-rpy) or 7 (xyz-rpy-e) values. Sorry. " << std::endl;
        return;
    }

    tf::StampedTransform ht;
    this->tf_listener->lookupTransform("right_arm_link0", "base_link", ros::Time(0), ht);

    tf::Vector3 p(msg->data[0], msg->data[1], msg->data[2]);
    tf::Quaternion q;
    if(msg->data.size() > 3)
        q.setRPY(msg->data[3], msg->data[4], msg->data[5]);
    else
        q.setRPY(0,0,0);

    p = ht * p; //These two lines make the transform
    q = ht * q;

    double dRoll, dPitch, dYaw;
    tf::Matrix3x3(q).getRPY(dRoll, dPitch, dYaw);

    std_msgs::Float32MultiArray msg_;
    msg_.data.push_back(p.x());
    msg_.data.push_back(p.y());
    msg_.data.push_back(p.z());
    if(msg->data.size() > 3)
    {
        msg_.data.push_back((float)dRoll);
        msg_.data.push_back((float)dPitch);
        msg_.data.push_back((float)dYaw);
    }
    if(msg->data.size() == 7)
        msg_.data.push_back(msg->data[6]);

    const std_msgs::Float32MultiArray::ConstPtr msgptr(new std_msgs::Float32MultiArray(msg_));
    this->callbackRaGoToPoseWrtArmTraj(msgptr);
}

void ManipPln::callbackLaGoToLoc(const std_msgs::String::ConstPtr& msg)
{
    if(this->laPredefPoses.find(msg->data) == this->laPredefPoses.end())
    {
        std::cout << "ManipPln.->Cannot find left arm predefined position: " << msg->data << std::endl;
        return;
    }

    std::cout << "ManipPln.->Left Arm goal pose: " << msg->data << " = ";
    for(int i=0; i< this->laPredefPoses[msg->data].size(); i++)
        std::cout << this->laPredefPoses[msg->data][i] << " ";
    std::cout << std::endl;

    std_msgs::Bool msgGoalReached;
    msgGoalReached.data = false;
    this->pubLaGoalReached.publish(msgGoalReached);
    this->laGoalPose = this->laPredefPoses[msg->data];
    this->calculateOptimalSpeeds(this->laCurrentPose, this->laGoalPose, this->laGoalSpeeds);
    this->laNewGoal = true;
}

void ManipPln::callbackRaGoToLoc(const std_msgs::String::ConstPtr& msg)
{
    if(this->raPredefPoses.find(msg->data) == this->raPredefPoses.end())
    {
        std::cout << "ManipPln.->Cannot find right arm predefined position: " << msg->data << std::endl;
        return;
    }

    std::cout << "ManipPln.->Right Arm goal pose: " << msg->data << " = ";
    for(int i=0; i< this->raPredefPoses[msg->data].size(); i++)
        std::cout << this->raPredefPoses[msg->data][i] << " ";
    std::cout << std::endl;

    std_msgs::Bool msgGoalReached;
    msgGoalReached.data = false;
    this->pubRaGoalReached.publish(msgGoalReached);
    this->raGoalPose = this->raPredefPoses[msg->data];
    this->calculateOptimalSpeeds(this->raCurrentPose, this->raGoalPose, this->raGoalSpeeds);
    this->raNewGoal = true;
}

void ManipPln::callbackHdGoToLoc(const std_msgs::String::ConstPtr& msg)
{
    std::cout << "ManipPln.->JE SUIS DESOLÉ. THIS COMMAND IS STILL NOT IMPLEMENTED" << std::endl;
}

void ManipPln::callbackLaMove(const std_msgs::String::ConstPtr& msg)
{
    std::cout << "ManipPln.->JE SUIS DESOLÉ. THIS COMMAND IS STILL NOT IMPLEMENTED" << std::endl;
}

void ManipPln::callbackRaMove(const std_msgs::String::ConstPtr& msg)
{
    std::cout << "ManipPln.->JE SUIS DESOLÉ. THIS COMMAND IS STILL NOT IMPLEMENTED" << std::endl;
}

void ManipPln::callbackHdMove(const std_msgs::String::ConstPtr& msg)
{
    std::cout << "ManipPln.->JE SUIS DESOLÉ. THIS COMMAND IS STILL NOT IMPLEMENTED" << std::endl;
}

//
//Callback for subscribers for operating the hardware nodes
//
void ManipPln::callbackLaCurrentPose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    //std::cout << "La pose received" << std::endl;
    this->laCurrentPose = msg->data;
}

void ManipPln::callbackRaCurrentPose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    //std::cout << "Ra pose received" << std::endl;
    this->raCurrentPose = msg->data;
}

void ManipPln::callbackHdCurrentPose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    //std::cout << "Hd pose received" << std::endl;
    this->hdCurrentPose = msg->data;
}

void ManipPln::callbackgPos(const geometry_msgs::Point::ConstPtr& msg){
    gPos.x = msg->x;
    gPos.y = msg->y;
    gPos.z = msg->z;
}
