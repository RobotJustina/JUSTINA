#include <ros/ros.h>
#include <vector>
#include <hardware_tools/DynamixelManager.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

bool newGoalPose = true;
bool readSimul = false;
bool simul = false;

int goalPos[2] = {0, 0};
int goalSpeeds[2] = {90, 75};
int PID[2][3] = {{24, 0, 128}, {128, 0, 64}};
int minLimits[2] = {1023, 0};
int maxLimits[2] = {3069, 4095};

float goalPos_simul[2] = {0.0, 0.0};
float goalSpeeds_simul[2] = {0.3, 0.3};

int zero_head[2] = {2040, 2520};
//float offset = -0.07671; // This for help me carry
float offset = -0.04; // This is for p and g 
float offsetReadSimul = -0.04;

void callbackHeadGoalPose(const std_msgs::Float32MultiArray::ConstPtr &msg){
    // std::cout << "head_node.-> Reciving new goal head pose." << std::endl;
    if(!(msg->data.size() == 2))
        std::cout << "Can not process the goal poses for the head" << std::endl;
    else{
        if(!simul ||(!simul && readSimul)){
            float goalPan = msg->data[0];
            float goalTilt = msg->data[1];
            if(goalPan < -1.1)
                goalPan = -1.1;
            if(goalPan > 1.1)
                goalPan = 1.1;
            if(goalTilt < -0.9)
                goalTilt = -0.9;
            if(goalTilt > 0.0)
                goalTilt = 0.0;

            goalPos[0] = int( (goalPan /(360.0/4095.0*M_PI/180.0)) + zero_head[0]);
            goalPos[1] = int( (goalTilt/(360.0/4095.0*M_PI/180.0)) + zero_head[1]);
            //std::cout << "head_node.->goal pose [0]:" << goalPos[0] << ", goal pose [1]:" << goalPos[1] << std::endl;
            //for(int i = 0; i < 2; i++)
                //goalSpeeds[i] = 80;

            if(readSimul){
                goalPos_simul[0] = msg->data[0];
                goalSpeeds_simul[0] = 0.1;
                goalPos_simul[1] = msg->data[1];
                goalSpeeds_simul[1] = 0.1;
            }

            if(goalPos[0] >= minLimits[0] && goalPos[0] <= maxLimits[0] && goalPos[1] >= minLimits[1] && goalPos[1] <= maxLimits[1])
                newGoalPose = true;
        }
        else{
            goalPos_simul[0] = msg->data[0];
            goalSpeeds_simul[0] = 0.1;
            goalPos_simul[1] = msg->data[1];
            goalSpeeds_simul[1] = 0.1;
        }
    }
}

void callback_simulated(const std_msgs::Bool::ConstPtr &msg){
    simul = msg->data;
}

int main(int argc, char ** argv){

    ros::init(argc, argv, "head_node");
    ros::NodeHandle n;
    
    std::string port;
    int baudRate;
    bool bulkEnable = false;
    bool syncWriteEnable = false;
    bool correctParams = false;
    simul = false;

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

    if(ros::param::has("~simul"))
        ros::param::get("~simul", simul);
    
    if(ros::param::has("~read_simul"))
        ros::param::get("~read_simul", readSimul);

    if(!correctParams){
        std::cerr << "Can not initialized the head node, please put correct params to this node, for example." << std::endl;
        std::cerr << "port : tty01" << std::endl;
        std::cerr << "baud : 1000000" << std::endl;
        return -1;
    }

    ros::Subscriber subGoalPos = n.subscribe("/hardware/head/goal_pose", 1, callbackHeadGoalPose);
    ros::Subscriber subSimul = n.subscribe("/simulated", 1, callback_simulated); 
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
    ros::Publisher pubHeadPose = n.advertise<std_msgs::Float32MultiArray>("head/current_pose", 1);
    ros::Publisher pubBattery = n.advertise<std_msgs::Float32>("/hardware/robot_state/head_battery", 1);

    ros::Rate rate(30);

    std::vector<int> ids;
    ids.push_back(0);
    ids.push_back(1);
    DynamixelManager dynamixelManager;
    if(!simul){
        //dynamixelManager.enableInfoLevelDebug();
        dynamixelManager.init(port, baudRate, bulkEnable, ids, syncWriteEnable);
    }

    uint16_t curr_position[2];
    for(int i = 0; i < 2; i++){
        curr_position[i] = zero_head[i];
        goalPos[i] = zero_head[i];
    }

    float bitsPerRadian = 4095.0/360.0*180.0/M_PI;

    std::string names[2] = {"pan_connect", "tilt_connect"};
    float positions[2] = {0, 0};
    
    sensor_msgs::JointState jointStates;
    jointStates.name.insert(jointStates.name.begin(), names, names + 2);
    jointStates.position.insert(jointStates.position.begin(), positions, positions + 2);

    // Setup features for init the servos of left arm
    if(!simul){
        for(int i = 0; i < 2; i++){
            dynamixelManager.enableTorque(i); 
            dynamixelManager.setPGain(i, PID[i][0]);
            dynamixelManager.setIGain(i, PID[i][1]);
            dynamixelManager.setDGain(i, PID[i][2]);
            dynamixelManager.setMaxTorque(i, 1023);
            dynamixelManager.setTorqueLimit(i, 512);
            dynamixelManager.setHighestLimitTemperature(i, 80);
            dynamixelManager.setAlarmShutdown(i, 0b00000100);
            dynamixelManager.setMovingSpeed(i, 100);
            dynamixelManager.setCWAngleLimit(i, minLimits[i]);
            dynamixelManager.setCCWAngleLimit(i, maxLimits[i]);
        }
    }

    std_msgs::Float32MultiArray msgCurrPose;
    msgCurrPose.data.resize(2);
    std_msgs::Float32 msgBattery;
    

    //initialize simulation variables
    float Pos[2] = {0.0, 0.0};
    float deltaPos[2] = {0.0, 0.0};
    for(int i = 0; i < 2; i++){
        goalPos_simul[i] = 0.0;
        goalSpeeds_simul[i] = 0.1;
    }

    while(ros::ok()){
        if(!simul){
            if(newGoalPose){
                //std::cout << "head_pose.->send newGoalPose" << std::endl;
                for(int i = 0; i < 2; i++){
                    dynamixelManager.setMovingSpeed(i, goalSpeeds[i]);
                    dynamixelManager.setGoalPosition(i, goalPos[i]);
                }
                if(syncWriteEnable){
                    dynamixelManager.writeSyncGoalPosesData();
                    dynamixelManager.writeSyncSpeedsData();
                }
                newGoalPose = false;
            }
        }
        if(!simul && !readSimul){
            if(bulkEnable)
                dynamixelManager.readBulkData();
            bool readData = true;
            for(int i = 0; i < 2 && readData; i++)
                readData = dynamixelManager.getPresentPosition(i, curr_position[i]);
            if(!readData)
                std::cout << "head_node.->Read data not found." << std::endl;

            jointStates.header.stamp = ros::Time::now();
            //std::cout << "head_node.->curr pose [0]:" << curr_position[0] << ", curr pose [1]:" << curr_position[1] << std::endl;
            jointStates.position[0] = (- (float) (zero_head[0]-curr_position[0]))/bitsPerRadian;
            jointStates.position[1] = ((float) (zero_head[1]-curr_position[1]))/bitsPerRadian;
            
            msgCurrPose.data[0] = jointStates.position[0]; 
            msgCurrPose.data[1] = -jointStates.position[1]; 
            
            jointStates.position[1] = jointStates.position[1] + offset;

            uint8_t voltage;
            dynamixelManager.getPresentVoltage(1, voltage);
            msgBattery.data = voltage / 10.0;
            pubBattery.publish(msgBattery);

            joint_pub.publish(jointStates);
            pubHeadPose.publish(msgCurrPose);
        }
        else{
            for(int i = 0; i < 2; i++){
                deltaPos[i] = goalPos_simul[i] - Pos[i];
                if(deltaPos[i] > goalSpeeds_simul[i])
                    deltaPos[i] = goalSpeeds_simul[i];
                if(deltaPos[i] < -goalSpeeds_simul[i])
                    deltaPos[i] = -goalSpeeds_simul[i];
                Pos[i] += deltaPos[i];
                if(i == 0)
                    jointStates.position[i] = Pos[i];
                else{ 
                    if(readSimul)
                        jointStates.position[i] = -Pos[i] + offsetReadSimul;
                    else
                        jointStates.position[i] = -Pos[i];
                }
                msgCurrPose.data[i] = Pos[i]; 
            }
                
            jointStates.header.stamp = ros::Time::now();
            joint_pub.publish(jointStates);
            pubHeadPose.publish(msgCurrPose);
            msgBattery.data =  11.6;
            pubBattery.publish(msgBattery);

        }

        rate.sleep();
        ros::spinOnce();
    }

    std::cout << "head_node.->Shutdowning the head node" << std::endl;
    std::cout << "head_node.->Writing the shutdowning pose" << std::endl;
    goalPos[0] = int(zero_head[0]);
    goalPos[1] = int( (-0.9/(360.0/4095.0*M_PI/180.0)) + zero_head[1]);
    for(int i = 0; i < 2; i++){
        dynamixelManager.setGoalPosition(i, goalPos[i]);
        dynamixelManager.setMovingSpeed(i, 100);
    }
    if(syncWriteEnable){
        dynamixelManager.writeSyncGoalPosesData();
        dynamixelManager.writeSyncSpeedsData();
    }
    
    bool validatePosition [2] = {0, 0};
    int countValidate = 0;
    boost::posix_time::ptime prev = boost::posix_time::second_clock::local_time();
    boost::posix_time::ptime curr = prev;
    do{
        for(int i = 0; i < 2; i++){
            if(!validatePosition[i]){
                unsigned short position;
                if(bulkEnable)
                    dynamixelManager.readBulkData();
                dynamixelManager.getPresentPosition(i, position);
                float error = fabs(position - goalPos[i]);
                //std::cout << "left_arm_pose.->Moto:" <<  i << ", error: " << error << std::endl;
                if(error < 10){
                    validatePosition[i] = true;
                    countValidate++;
                }
            }
        }
        curr = boost::posix_time::second_clock::local_time();
    }while(countValidate < 2 && (curr - prev).total_milliseconds() < 7500);

    std::cout << "head_node.->The head have reached the finish pose" << std::endl;

    for(int i = 0; i < 2; i++)
        dynamixelManager.disableTorque(i); 

    dynamixelManager.close();

    return 1;
}

