#include <ros/ros.h>
#include <vector>
#include <hardware_tools/DynamixelManager.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

bool newGoalPose = false;

int goalPos[7] = {0, 0, 0, 0, 0, 0, 0};
int goalSpeeds[7] = {0, 0, 0, 0, 0, 0, 0};
int goalGripper[2] = {0, 0};

int zero_arm[7] = {1543, 1600, 1800, 2100, 2048, 1800, 1050};
int zero_gripper[2] = {2440, 2680};

bool torqueGripperCCW1 = true, torqueGripperCCW2 = false, gripperTorqueActive = false, newGoalGripper = true;
float torqueGripper;
uint16_t speedGripper = 200;
uint16_t currentLoadD21, currentLoadD22;

int attempts = 0;

bool validateCMD[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void callbackArmGoalPose(const std_msgs::Float32MultiArray::ConstPtr &msg){
    std::cout << "left_arm_node.-> Reciving new goal left arm pose." << std::endl;
    if(!(msg->data.size() == 7 || msg->data.size() == 14))
        std::cout << "Can not process the goal poses for the left arm" << std::endl;
    else{
        goalPos[0] = int( (msg->data[0]/(360.0/4095.0*M_PI/180.0)) + zero_arm[0]);
        goalPos[1] = int( (msg->data[1]/(360.0/4095.0*M_PI/180.0)) + zero_arm[1]);
        goalPos[2] = int( (msg->data[2]/(360.0/4095.0*M_PI/180.0)) + zero_arm[2]);
        goalPos[3] = int(-(msg->data[3]/(360.0/4095.0*M_PI/180.0)) + zero_arm[3]);
        goalPos[4] = int( (msg->data[4]/(360.0/4095.0*M_PI/180.0)) + zero_arm[4]);
        goalPos[5] = int( (msg->data[5]/(360.0/4095.0*M_PI/180.0)) + zero_arm[5]);
        goalPos[6] = int( (msg->data[6]/(360.0/4095.0*M_PI/180.0)) + zero_arm[6]);
        // std::cout << "left_arm_node.->goalPose[0]:" << goalPos[0] << std::endl;
        for(int i = 0; i < 7; i++)
            goalSpeeds[i] = 40;
        if(msg->data.size() == 14){
            for(int i = 7; i < 14; i++){
                goalSpeeds[i - 7] = msg->data[i] * 1023;
                if(goalSpeeds[i - 7] < 0)
                    goalSpeeds[i - 7] = 0;
                if(goalSpeeds[i - 7] > 1023)
                    goalSpeeds[i - 7] = 1023;
            }
        }
        newGoalPose = true;
    }
}

void callbackGripperPos(const std_msgs::Float32::ConstPtr &msg){
    goalGripper[0] = int(( (msg->data)/(360.0/4095.0*M_PI/180.0)) + zero_gripper[0] );
    goalGripper[1] = int((-(msg->data)/(360.0/4095.0*M_PI/180.0)) + zero_gripper[1] );
    gripperTorqueActive = false;
    newGoalGripper = true;
    for(int i = 0; i < 10; i++)
        validateCMD[i] = false;
}

void callbackGripperTorque(const std_msgs::Float32::ConstPtr &msg){
    float torque = msg->data;
    if(torque > 1.0)
        torque = 1.0;
    if(torque < -1.0)
        torque = -1.0;
    gripperTorqueActive = true;
    if(torque < 0){
        torqueGripper = (int) (-800.0 * torque);
        torqueGripperCCW1 = false;
        torqueGripperCCW2 = true;
    }
    else{
        torqueGripper = (int) (800.0 * torque);
        torqueGripperCCW1 = true;
        torqueGripperCCW2 = false;
    }
    newGoalGripper = true;
    for(int i = 0; i < 10; i++)
        validateCMD[i] = false;
}

int main(int argc, char ** argv){

    ros::init(argc, argv, "left_arm_node");
    ros::NodeHandle n;
    
    std::string port;
    int baudRate;
    bool bulkEnable = false;
    bool syncWriteEnable = false;
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

    if(ros::param::has("~bulk_enable"))
        ros::param::get("~bulk_enable", bulkEnable);
    
    if(ros::param::has("~sync_write_enable"))
        ros::param::get("~sync_write_enable", syncWriteEnable);

    if(!correctParams){
        std::cerr << "Can not initialized the arm left node, please put correct params to this node, for example." << std::endl;
        std::cerr << "port : tty01" << std::endl;
        std::cerr << "baud : 1000000" << std::endl;
        return -1;
    }

    tf::TransformBroadcaster broadCaster;

    ros::Subscriber subGoalPos = n.subscribe("/hardware/left_arm/goal_pose", 1, callbackArmGoalPose);
    ros::Subscriber subGripperPos = n.subscribe("/hardware/left_arm/goal_gripper", 1, callbackGripperPos);
    ros::Subscriber subGripperTroque = n.subscribe("/hardware/left_arm/torque_gripper", 1, callbackGripperTorque);

    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
    ros::Publisher pubArmPose = n.advertise<std_msgs::Float32MultiArray>("left_arm/current_pose", 1);
    ros::Publisher pubGripper = n.advertise<std_msgs::Float32>("left_arm/current_gripper", 1);
    ros::Publisher pubObjOnHand = n.advertise<std_msgs::Bool>("left_arm/object_on_hand", 1);
    ros::Publisher pubBattery = n.advertise<std_msgs::Float32>("/hardware/robot_state/left_arm_battery", 1);

    ros::Rate rate(20);

    std::vector<int> ids;
    for(int i = 0; i < 9; i++)
        ids.push_back(i);
    DynamixelManager dynamixelManager;
    // dynamixelManager.enableInfoLevelDebug();
    dynamixelManager.init(port, baudRate, bulkEnable, ids, syncWriteEnable);

    uint16_t curr_position[9] = {1543, 1600, 1800, 2100, 2048, 1800, 1050, 2440, 2680};

    //float bitsPerRadian = (4095.0)/((360.0)*(3.141592/180.0));
    float bitsPerRadian = 4095.0/360.0*180.0/M_PI;

    std::string names[9] = {"la_1_joint", "la_2_joint", "la_3_joint", "la_4_joint", "la_5_joint", "la_6_joint", "la_7_joint", "la_grip_left", "la_grip_right"};
    float positions[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    
    sensor_msgs::JointState jointStates;
    jointStates.name.insert(jointStates.name.begin(), names, names + 9);
    jointStates.position.insert(jointStates.position.begin(), positions, positions + 9);

    // Setup features for init the servos of left arm
    for(int i = 0; i < 9; i++){
        dynamixelManager.enableTorque(i); 
        dynamixelManager.setPGain(i, 128);
        dynamixelManager.setIGain(i, 0);
        dynamixelManager.setDGain(i, 32);
        /*dynamixelManager.setPGain(i, 32);
        dynamixelManager.setIGain(i, 0);
        dynamixelManager.setDGain(i, 0);*/
        dynamixelManager.setMaxTorque(i, 1023);
        dynamixelManager.setTorqueLimit(i, 768);
        dynamixelManager.setHighestLimitTemperature(i, 80);
        dynamixelManager.setAlarmShutdown(i, 0b00000100);
    }

    dynamixelManager.setCWAngleLimit(7, 0);
    dynamixelManager.setCCWAngleLimit(7, 4095);
    dynamixelManager.setCWAngleLimit(8, 0);
    dynamixelManager.setCCWAngleLimit(8, 4095);

    dynamixelManager.setMovingSpeed(7, 100);
    dynamixelManager.setMovingSpeed(8, 100);

    goalGripper[0] = zero_gripper[0];
    goalGripper[1] = zero_gripper[1];

    std_msgs::Float32MultiArray msgCurrPose;
    msgCurrPose.data.resize(7);
    std_msgs::Float32 msgCurrGripper;
    std_msgs::Bool msgObjOnHand;
    std_msgs::Float32 msgBattery;

    while(ros::ok()){

        if(newGoalPose){
            std::cout << "left_arm_pose.->send newGoalPose" << std::endl;
            for(int i = 0; i < 7; i++){
                dynamixelManager.setMovingSpeed(i, goalSpeeds[i]);
                dynamixelManager.setGoalPosition(i, goalPos[i]);
            }
            if(syncWriteEnable){
                dynamixelManager.writeSyncGoalPosesData();
                dynamixelManager.writeSyncSpeedsData();
            }
            newGoalPose = false;
        }

        if(newGoalGripper){
            std::cout << "left_arm_node.->Proccessing the new goal gripper." << std::endl; 
            int countValidLimit, countValid = 0;
            if(gripperTorqueActive){
                if(!validateCMD[0])
                    validateCMD[0] = dynamixelManager.setCWAngleLimit(7, 0);
                if(!validateCMD[1])
                    validateCMD[1] = dynamixelManager.setCCWAngleLimit(7, 0);
                if(!validateCMD[2])
                    validateCMD[2] = dynamixelManager.setCWAngleLimit(8, 0);
                if(!validateCMD[3])
                    validateCMD[3] = dynamixelManager.setCCWAngleLimit(8, 0);
                if(!validateCMD[4])
                    validateCMD[4] = dynamixelManager.setTorqueLimit(7, torqueGripper);
                if(!validateCMD[5])
                    validateCMD[5] = dynamixelManager.setTorqueLimit(8, torqueGripper);
                if(!validateCMD[6])
                    validateCMD[6] = dynamixelManager.setTorqueValue(7, speedGripper, torqueGripperCCW1);
                if(!validateCMD[7])
                    validateCMD[7] = dynamixelManager.setTorqueValue(8, speedGripper, torqueGripperCCW2);
                countValidLimit = 8;
            }
            else{
                if(!validateCMD[0])
                    validateCMD[0] = dynamixelManager.setCWAngleLimit(7, 0);
                if(!validateCMD[1])
                    validateCMD[1] = dynamixelManager.setCCWAngleLimit(7, 4095);
                if(!validateCMD[2])
                    validateCMD[2] = dynamixelManager.setCWAngleLimit(8, 0);
                if(!validateCMD[3])
                    validateCMD[3] = dynamixelManager.setCCWAngleLimit(8, 4095);
                if(!validateCMD[4])
                    validateCMD[4] = dynamixelManager.setTorqueLimit(7, 500);
                if(!validateCMD[5])
                    validateCMD[5] = dynamixelManager.setTorqueLimit(8, 500);
                if(!validateCMD[6])
                    validateCMD[6] = dynamixelManager.setMovingSpeed(7, 200);
                if(!validateCMD[7])
                    validateCMD[7] = dynamixelManager.setMovingSpeed(8, 200);
                if(!validateCMD[8])
                    validateCMD[8] = dynamixelManager.setGoalPosition(7, goalGripper[0]);
                if(!validateCMD[9])
                    validateCMD[9] = dynamixelManager.setGoalPosition(8, goalGripper[1]);
                countValidLimit = 10;
            }
            for(int i = 0; i < countValidLimit; i++){
                if(validateCMD[i])
                    countValid++;
            }
    
            //std::cout << "left_arm_node.->CountValid=" << countValid << std::endl;
            attempts++;
            if(attempts > 5 || countValid == countValidLimit){
                newGoalGripper = false;
                attempts = 0;
            }
        }

        if(bulkEnable)
            dynamixelManager.readBulkData();
        for(int i = 0; i < 9; i++)
            dynamixelManager.getPresentPosition(i, curr_position[i]);

        jointStates.header.stamp = ros::Time::now();
        jointStates.position[0] = -((float) (zero_arm[0]-curr_position[0]))/bitsPerRadian;
        jointStates.position[1] = -((float) (zero_arm[1]-curr_position[1]))/bitsPerRadian;
        jointStates.position[2] = -((float) (zero_arm[2]-curr_position[2]))/bitsPerRadian;
        jointStates.position[3] =  ((float) (zero_arm[3]-curr_position[3]))/bitsPerRadian;
        jointStates.position[4] = -((float) (zero_arm[4]-curr_position[4]))/bitsPerRadian;
        jointStates.position[5] = -((float) (zero_arm[5]-curr_position[5]))/bitsPerRadian;
        jointStates.position[6] = -((float) (zero_arm[6]-curr_position[6]))/bitsPerRadian;
        jointStates.position[7] = -((float) (zero_gripper[0]-curr_position[7]))/bitsPerRadian;
        jointStates.position[8] =  ((float) (zero_gripper[1]-curr_position[8]))/bitsPerRadian;
        // std::cout << "left_arm_node.->curr_position[0]:" << curr_position[0] << std::endl;
        
        if(gripperTorqueActive){
            dynamixelManager.getPresentLoad(7, currentLoadD21);
            dynamixelManager.getPresentLoad(8, currentLoadD22);
            if(currentLoadD21 > 1023)
                currentLoadD21 -= 1023;
            if(currentLoadD22 > 1023)
                currentLoadD22 -= 1023;
            currentLoadD21 = (currentLoadD21 + currentLoadD22) / 2.0f;
            if(currentLoadD21 > 200 and jointStates.position[7] > -0.05)
                msgObjOnHand.data = true;
            else
                msgObjOnHand.data = false;
        }

        for(int i = 0; i < 7; i++)
           msgCurrPose.data[i] = jointStates.position[i]; 

        msgCurrGripper.data = jointStates.position[7];

        uint8_t voltage;
        dynamixelManager.getPresentVoltage(2, voltage);
        msgBattery.data = voltage / 10.0;
        pubBattery.publish(msgBattery);

        joint_pub.publish(jointStates);
        pubArmPose.publish(msgCurrPose);
        pubGripper.publish(msgCurrGripper);
        pubObjOnHand.publish(msgObjOnHand);

        rate.sleep();
        ros::spinOnce();
    }

    std::cout << "left_arm_pose.->Shutdowning the left arm node" << std::endl;
    std::cout << "left_arm_pose.->Writing the zero_arm init pose" << std::endl;
    for(int i = 0; i < 6; i++){
        uint16_t zeroPose = (uint16_t) zero_arm[i];
        dynamixelManager.setGoalPosition(i, zeroPose);
        dynamixelManager.setMovingSpeed(i, 30);
    }
    if(syncWriteEnable){
        dynamixelManager.writeSyncGoalPosesData();
        dynamixelManager.writeSyncSpeedsData();
    }
    
    bool validatePosition [7] = {0, 0, 0, 0, 0, 0, 0};
    int countValidate = 0;
    boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
    boost::posix_time::ptime curr = prev;
    do{
        for(int i = 0; i < 6; i++){
            if(!validatePosition[i]){
                unsigned short position;
                if(bulkEnable)
                    dynamixelManager.readBulkData();
                dynamixelManager.getPresentPosition(i, position);
                float error = fabs(position - zero_arm[i]);
                //std::cout << "left_arm_pose.->Moto:" <<  i << ", error: " << error << std::endl;
                if(error < 10){
                    validatePosition[i] = true;
                    countValidate++;
                }
            }
        }
        curr = boost::posix_time::second_clock::local_time();
    }while(countValidate < 6 && (curr - prev).total_milliseconds() < 7500);

    std::cout << "left_arm_node.->The arm have reached the init pose" << std::endl;


    for(int i = 0; i < 9; i++)
        dynamixelManager.disableTorque(i); 

    dynamixelManager.close();

    return 1;
}

