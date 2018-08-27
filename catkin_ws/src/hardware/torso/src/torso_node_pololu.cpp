#include <ros/ros.h>
#include <iostream>
#include <memory>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <hardware_tools/JrkManager.hpp>

#define TH_ERR 0.001

float getPositionFromFeedback(int feedback){
    return 0.0076491823 * feedback - 0.5021359358;
}

int getFeedbackFromPosition(float position){
    return 130.657375698 * position + 66.7352975312;
}
bool newGoalPose = false;
std_msgs::Float32MultiArray goalPose;

std::shared_ptr<JrkManager> jrkManager;
int currFeedback = 0;

void callbackRelativeHeight(const std_msgs::Float32MultiArray::ConstPtr &msg){
    std::cout << "torso_node_pololu.->Reciving relative new goal pose." << std::endl;
    newGoalPose = true;
    currFeedback = jrkManager->getFeedback();
    float absPosition = getPositionFromFeedback(currFeedback) / 100.0f;
    goalPose.data[0] = absPosition + msg->data[0];
    unsigned int goalTarget;
    if(goalPose.data[0] < 0 || goalPose.data[0] > 0.3){
        std::cout << "torso_node_pololu.->Can not reached the goal position, adjust the nearest goal reached." << std::endl;
        if(goalPose.data[0] < 0)
            goalPose.data[0] = 0.0f;
        if(goalPose.data[0] > 0.3)
            goalPose.data[0] = 0.3f;
    }
    goalTarget = getFeedbackFromPosition(goalPose.data[0] * 100.0f);
    jrkManager->setTarget(goalTarget);
}

void callbackAbsoluteHeight(const std_msgs::Float32MultiArray::ConstPtr &msg){
    std::cout << "torso_node_pololu.->Reciving absolute new goal pose." << std::endl;
    newGoalPose = true;
    goalPose.data[0] = msg->data[0];
    if(msg->data[0] < 0 || goalPose.data[0] > 0.3){
        std::cout << "torso_node_pololu.->Can not reached the goal position, adjust the nearest goal reached." << std::endl;
        if(goalPose.data[0] < 0)
            goalPose.data[0] = 0.0f;
        if(goalPose.data[0] > 0.3)
            goalPose.data[0] = 0.3f;
    }
    unsigned int goalTarget = getFeedbackFromPosition(goalPose.data[0] * 100.0f);
    std::cout << "tosro_node_pololu.->Send goal target:" << goalTarget << std::endl;
    jrkManager->setTarget(goalTarget);
}

int main(int argc, char ** argv){
    ros::init(argc, argv, "torso_node");
    ros::NodeHandle n;
    ros::Rate rate(30);
    
    std::string port;
    int baudRate;
    bool correctParams = false;
    if(ros::param::has("~port")){
        ros::param::get("~port", port);
        correctParams = true;
    }
    if(ros::param::has("~baud")){
        ros::param::get("~baud", baudRate);
        correctParams &= true;
    }
    else
        correctParams &= true;

    if(!correctParams){
        std::cerr << "Can not initialized the arm left node, please put correct params to this node, for example." << std::endl;
        std::cerr << "port : tty01" << std::endl;
        std::cerr << "baud : 1000000" << std::endl;
        return -1;
    }
   
    jrkManager = std::make_shared<JrkManager>(port, baudRate, 100);

    ros::Publisher pubTorsoPose = n.advertise<std_msgs::Float32MultiArray>("/hardware/torso/current_pose", 1);
    ros::Publisher pubGoalReached = n.advertise<std_msgs::Bool>("/hardware/torso/goal_reached", 1);
    ros::Publisher pubJointState = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
    ros::Subscriber subRelativeHeight = n.subscribe("/hardware/torso/goal_rel_pose", 1, callbackRelativeHeight);
    ros::Subscriber subAbsoluteHeight = n.subscribe("/hardware/torso/goal_pose", 1, callbackAbsoluteHeight);

    jrkManager->getErrorsHalting();

    std::string names[5] = {"spine_connect","waist_connect","shoulders_connect", "shoulders_left_connect", "shoulders_right_connect"};

    float positions[5] = {0, 0, 0, 0, 0};

    sensor_msgs::JointState jointStates;
    jointStates.name.insert(jointStates.name.begin(), names, names + 5);
    jointStates.position.insert(jointStates.position.begin(), positions, positions + 5);

    std_msgs::Float32MultiArray msgCurrentPose;
    msgCurrentPose.data.resize(3);

    goalPose.data.resize(3);

    while(ros::ok()){
    
        try{
            currFeedback = jrkManager->getFeedback();
        }
        catch(JrkTimeout exceptionTimeout){
            std::cout << "torso_node_pololu.->Can not read the feedback torso." << exceptionTimeout.what() << std::endl;
        }
        // std::cout << "torso_node_pololu.->Feedback:" << currFeedback << std::endl;
        float position;
        if(currFeedback >= 0)
            position = getPositionFromFeedback(currFeedback) / 100.0f;
        else
            position = jointStates.position[0];

        if(newGoalPose){
            float err = fabs(position - goalPose.data[0]);
            if(err <= TH_ERR){
                std_msgs::Bool msg;
                msg.data = true;
                pubGoalReached.publish(msg);
                newGoalPose = false;
            }
        }

        jointStates.header.stamp = ros::Time::now();
        jointStates.position[0] = position;

        msgCurrentPose.data[0] = position;
        msgCurrentPose.data[1] = 0.0;
        msgCurrentPose.data[2] = 0.0;

        pubJointState.publish(jointStates);
        pubTorsoPose.publish(msgCurrentPose);
       
        rate.sleep();
        ros::spinOnce();
    }

    jrkManager->motorOff();

    return 1;

}
