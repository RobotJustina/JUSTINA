#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <hardware_tools/PololuJrkManager.hpp>

#define TH_ERR 0.

bool newGoalPose = false;
std_msgs::Float32MultiArray goalPose;

PololuJrkManager jrkManager;

void callbackRelativeHeight(const std_msgs::Float32MultiArray::ConstPtr &msg){
    std::cout << "torso_node_pololu.->Reciving relative new goal pose." << std::endl;
    int feedback = jrkManager.getFeedback(0);
    float absPosition = jrkManager.getPositionFromFeedback(feedback) / 100.0f;
    newGoalPose = true;
    goalPose.data[0] = absPosition + msg->data[0];
    unsigned int goalTarget = jrkManager.getFeedbackFromPosition(goalPose.data[0] * 100.0f);
    jrkManager.setTarget(0, goalTarget);
}

void callbackAbsoluteHeight(const std_msgs::Float32MultiArray::ConstPtr &msg){
    std::cout << "torso_node_pololu.->Reciving absolute new goal pose." << std::endl;
    newGoalPose = true;
    goalPose.data[0] = msg->data[0];
    unsigned int goalTarget = jrkManager.getFeedbackFromPosition(goalPose.data[0] * 100.0f);
    std::cout << "tosro_node_pololu.->Send goal target:" << goalTarget << std::endl;
    jrkManager.setTarget(0, goalTarget);
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

    ros::Publisher pubTorsoPose = n.advertise<std_msgs::Float32MultiArray>("/hardware/torso/current_pose", 1);
    ros::Publisher pubGoalReached = n.advertise<std_msgs::Bool>("/hardware/torso/goal_reached", 1);
    ros::Publisher pubJointState = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
    ros::Subscriber subRelativeHeight = n.subscribe("/hardware/torso/goal_rel_pose", 1, callbackRelativeHeight);
    ros::Subscriber subAbsoluteHeight = n.subscribe("/hardware/torso/goal_pose", 1, callbackAbsoluteHeight);

    jrkManager.init(port.c_str());
    jrkManager.getErrorFlagsHalting(0);

    std::string names[5] = {"spine_connect","waist_connect","shoulders_connect", "shoulders_left_connect", "shoulders_right_connect"};

    float positions[5] = {0, 0, 0, 0, 0};

    sensor_msgs::JointState jointStates;
    jointStates.name.insert(jointStates.name.begin(), names, names + 5);
    jointStates.position.insert(jointStates.position.begin(), positions, positions + 5);

    std_msgs::Float32MultiArray msgCurrentPose;
    msgCurrentPose.data.resize(3);

    goalPose.data.resize(3);

    while(ros::ok()){

        int feedback = jrkManager.getScaledFeedback(0);
        std::cout << "torso_node_pololu.->Feedback:" << feedback << std::endl;
        float position;
        if(feedback >= 0)
            position = jrkManager.getPositionFromFeedback(feedback) / 100.0f;
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

    return 1;

}
