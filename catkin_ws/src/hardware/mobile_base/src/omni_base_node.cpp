#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <hardware_tools/roboclaw.hpp>
#include <exception>
#include <algorithm>

#define BASE_WIDTH 0.52

bool newData = false;
int noNewDataCounter = 5;
long lastSetSpeedTime;
    
Roboclaw * rcFrontal;
Roboclaw * rcLateral;

uint8_t rcAddressFrontal =  128;
uint8_t rcAddressLateral =  128;
    
float QPPS_LEFT;
float QPPS_RIGHT;
float QPPS_FRONT;
float QPPS_REAR;
    
float leftSpeed;
float rightSpeed;
float frontSpeed;
float rearSpeed;

void checkSpeedRanges(float &sLeft, float &sRight, float &sFront, float &sRear)
{
    float maxValueFrontal = std::max(fabs(sLeft), fabs(sRight));
    float maxValueLateral = std::max(fabs(sFront), fabs(sRear));
    if(maxValueFrontal > 1.0)
    {
        sLeft /= maxValueFrontal;
        sRight /= maxValueFrontal;
    }
    if(maxValueLateral > 1.0)
    {
        sFront /= maxValueLateral;
        sRear /= maxValueLateral;
    }
}

void callbackCmdVel(const geometry_msgs::Twist::ConstPtr &msg)
{
    std::cout << "Reciving cmd vel." << std::endl;
    lastSetSpeedTime = ros::Time::now().toSec();
    float linearX   = msg->linear.x;
    float linearY   = msg->linear.y;
    float angularZ  = msg->angular.z;

    leftSpeed     = linearX - angularZ * BASE_WIDTH / 2.0;
    rightSpeed    = linearX + angularZ * BASE_WIDTH / 2.0;
    frontSpeed    = linearY + angularZ * BASE_WIDTH / 2.0;
    rearSpeed     = linearY - angularZ * BASE_WIDTH / 2.0;
    
    checkSpeedRanges(leftSpeed, rightSpeed, frontSpeed, rearSpeed);
    
    newData = true;
    noNewDataCounter = 5.0;

}

int main(int argc, char ** argv)
{
    std::cout << "Mobile base with roboclaw implemented in c++ by Reynaldo" << std::endl;
    ros::init(argc, argv, "omni_base_node");
    ros::NodeHandle nh;
    ros::Rate rate(30);

    std::string portFrontal = "/dev/ttyACM4";
    std::string portLateral = "/dev/ttyACM5";
    bool simul = false;
    bool correctParams = false;
    
    if(ros::param::has("~port1")){
        ros::param::get("~port1", portFrontal);
        correctParams = true;
    }
    if(ros::param::has("~port2")){
        ros::param::get("~port2", portLateral);
        correctParams = true;
    }

    if(ros::param::has("~simul"))
        ros::param::get("~simul", simul);

    if(!correctParams){
        std::cerr << "Can not initialized the arm left node, please put correct params to this node, for example." << std::endl;
        std::cerr << "port : tty01" << std::endl;
        std::cerr << "baud : 1000000" << std::endl;
        return -1;
    }

    int baudRateFrontal = 115200;
    int baudRateLateral = 115200;

    std::cout << "Port frontal:" << portFrontal << std::endl;
    std::cout << "Port lateral:" << portLateral << std::endl;

    char frontalVersion;
    char lateralVersion;

    float posPIDLeftF[3] = {0.0, 0.0, 0.0}; 
    uint32_t posPIDLeftI[4] = {0, 0, 0, 0}; 
    float posPIDRightF[3] = {0.0, 0.0, 0.0}; 
    uint32_t posPIDRightI[4] = {0, 0, 0, 0}; 
    float posPIDFrontF[3] = {0.0, 0.0, 0.0}; 
    uint32_t posPIDFrontI[4] = {0, 0, 0, 0}; 
    float posPIDRearF[3] = {0.0, 0.0, 0.0}; 
    uint32_t posPIDRearI[4] = {0, 0, 0, 0}; 
    
    float velPIDLeftF[4] = {0.0, 0.0, 0.0, 0.0}; 
    uint32_t velPIDLeftI;
    float velPIDRightF[4] = {0.0, 0.0, 0.0, 0.0}; 
    uint32_t velPIDRightI;
    float velPIDFrontF[4] = {0.0, 0.0, 0.0, 0.0}; 
    uint32_t velPIDFrontI;
    float velPIDRearF[4] = {0.0, 0.0, 0.0, 0.0}; 
    uint32_t velPIDRearI;

    if(!simul)
    {
        if(rcAddressFrontal > 0x87 or rcAddressFrontal < 0x80)
        { 
            ROS_INFO("Address frontal out of range");
            ros::shutdown();
        }
        if(rcAddressLateral > 0x87 or rcAddressLateral < 0x80)
        { 
            ROS_INFO("Address lateral out of range");
            ros::shutdown();
        }

        try
        {
            rcFrontal = new Roboclaw(portFrontal, baudRateFrontal); 
        }
        catch(std::exception &e)
        {
            ROS_FATAL("Could not connect to frontal roboclaw");
            ROS_FATAL("Error: %s", e.what());
        }
        
        try
        {
            rcLateral = new Roboclaw(portLateral, baudRateLateral);
        }
        catch(std::exception &e)
        {
            ROS_FATAL("Could not connect to lateral roboclaw");
            ROS_FATAL("Error: %s", e.what());
        }

        try
        {
            frontalVersion = rcFrontal->ReadVersion(rcAddressFrontal, &frontalVersion);
        }
        catch(std::exception &e)
        {
            ROS_WARN("Problem getting roboclaw frontal version");
            ROS_DEBUG("%s", e.what());
        }
        try
        {
            lateralVersion = rcFrontal->ReadVersion(rcAddressFrontal, &lateralVersion);
        }
        catch(std::exception &e)
        {
            ROS_WARN("Problem getting roboclaw lateral version");
            ROS_DEBUG("%s", e.what());
        }

        if(!frontalVersion)
        {
            ROS_WARN("Could not get version from frontal roboclaw");
        }
        else
        {
            ROS_DEBUG("%c", frontalVersion);
        }
        if(!lateralVersion)
        {
            ROS_WARN("Could not get version from lateral roboclaw");
        }
        else
        {
            ROS_DEBUG("%c", lateralVersion);
        }

        try
        {
            rcFrontal->SpeedM1M2(rcAddressFrontal, 0, 0);
            rcFrontal->ResetEncoders(rcAddressFrontal);
            rcLateral->SpeedM1M2(rcAddressLateral, 0, 0);         
            rcLateral->ResetEncoders(rcAddressLateral);
            
            rcFrontal->ReadM1PositionPID(rcAddressFrontal, posPIDLeftF[0], posPIDLeftF[1], posPIDLeftF[2], posPIDLeftI[0], posPIDLeftI[1], posPIDLeftI[2], posPIDLeftI[3]);

            rcFrontal->ReadM2PositionPID(rcAddressFrontal, posPIDRightF[0], posPIDRightF[1], posPIDRightF[2], posPIDRightI[0], posPIDRightI[1], posPIDRightI[2], posPIDRightI[3]);

            rcLateral->ReadM1PositionPID(rcAddressLateral, posPIDFrontF[0], posPIDFrontF[1], posPIDFrontF[2], posPIDFrontI[0], posPIDFrontI[1], posPIDFrontI[2], posPIDFrontI[3]);

            rcLateral->ReadM2PositionPID(rcAddressLateral, posPIDRearF[0], posPIDRearF[1], posPIDRearF[2], posPIDRearI[0], posPIDRearI[1], posPIDRearI[2], posPIDRearI[3]);

            rcFrontal->ReadM1VelocityPID(rcAddressFrontal, velPIDLeftF[0], velPIDLeftF[1], velPIDLeftF[2], velPIDLeftI);

            rcFrontal->ReadM2VelocityPID(rcAddressFrontal, velPIDRightF[0], velPIDRightF[1], velPIDRightF[2], velPIDRightI);

            rcLateral->ReadM1VelocityPID(rcAddressLateral, velPIDFrontF[0], velPIDFrontF[1], velPIDFrontF[2], velPIDFrontI);

            rcLateral->ReadM2VelocityPID(rcAddressLateral, velPIDRearF[0], velPIDRearF[1], velPIDRearF[2], velPIDRearI);

            std::cout << "MobileBase.->Position PID constants: Success P I D MaxI Deadzone MinPos MaxPos" << std::endl;
            std::cout << "MobileBase.->Left Motor: ";
            for (int i = 0; i < 3; i++) 
                std::cout << posPIDLeftF[i] << "   ";
            for (int i = 0; i < 4; i++) 
                std::cout << posPIDLeftI[i] << "   ";
            std::cout << std::endl;

            std::cout << "MobileBase.->Right Motor: ";
            for (int i = 0; i < 3; i++) 
                std::cout << posPIDRightF[i] << "   ";
            for (int i = 0; i < 4; i++) 
                std::cout << posPIDRightI[i] << "   ";
            std::cout << std::endl;

            std::cout << "MobileBase.->Front Motor: ";
            for (int i = 0; i < 3; i++) 
                std::cout << posPIDFrontF[i] << "   ";
            for (int i = 0; i < 4; i++) 
                std::cout << posPIDFrontI[i] << "   ";
            std::cout << std::endl;

            std::cout << "MobileBase.->Rear Motor: ";
            for (int i = 0; i < 3; i++) 
                std::cout << posPIDRearF[i] << "   ";
            for (int i = 0; i < 4; i++) 
                std::cout << posPIDRearI[i] << "   ";
            std::cout << std::endl;

            std::cout << "MobileBase.->Velocity PID constants: Success P I D MaxI Deadzone MinPos MaxPos" << std::endl;
            std::cout << "MobileBase.->Left Motor: ";
            for (int i = 0; i < 3; i++) 
                std::cout << velPIDLeftF[i] << "   ";
            std::cout << velPIDLeftI << "   ";
            std::cout << std::endl;

            std::cout << "MobileBase.->Right Motor: ";
            for (int i = 0; i < 3; i++) 
                std::cout << velPIDRightF[i] << "   ";
            std::cout << velPIDRightI << "   ";
            std::cout << std::endl;

            std::cout << "MobileBase.->Front Motor: ";
            for (int i = 0; i < 3; i++) 
                std::cout << velPIDFrontF[i] << "   ";
            std::cout << velPIDFrontI << "   ";
            std::cout << std::endl;

            std::cout << "MobileBase.->Rear Motor: ";
            for (int i = 0; i < 3; i++) 
                std::cout << velPIDRearF[i] << "   ";
            std::cout << velPIDRearI << "   ";
            std::cout << std::endl;

            QPPS_LEFT = velPIDLeftI;
            QPPS_RIGHT = velPIDRightI;
            QPPS_FRONT = velPIDFrontI;
            QPPS_REAR = velPIDRearI;

            if (QPPS_LEFT != QPPS_RIGHT || QPPS_REAR != QPPS_FRONT)
                std::cout << "MobileBase.->WARNING: Motors have different max speeds!! Stall condition may occur." << std::endl; 
            /*
               if self.rc_frontal.ReadPWMMode(self.rc_address_frontal)[1] == 1)
               print "MobileBase.->PWM Mode for left and right motors: Sign magnitude"
else:
print "MobileBase.->PWM Mode for left and right motors: Locked antiphase"
if self.rc_lateral.ReadPWMMode(self.rc_address_lateral)[1] == 1:
print "MobileBase.->PWM Mode for front and rear motors: Sign magnitude"
else:
print "MobileBase.->PWM Mode for front and rear motors: Locked antiphase"
*/

        }
        catch(std::exception &e)
        {
            ROS_INFO("Can not send init speed to roboclaw");
            ROS_DEBUG("%s", e.what());
        }

    }

    ros::Subscriber subCmdVel = nh.subscribe("/hardware/mobile_base/cmd_vel", 1, callbackCmdVel);

    while(ros::ok())
    {
        if(newData)
        {
            leftSpeed   = (leftSpeed    * QPPS_LEFT * 16.0 / 35.0);
            rightSpeed  = (rightSpeed   * QPPS_RIGHT * 16.0 / 35.0);
            frontSpeed  = -(frontSpeed   * QPPS_FRONT * 16.0 / 35.0);
            rearSpeed   = -(rearSpeed    * QPPS_REAR * 16.0 / 35.0);

            try
            {
                rcFrontal->SpeedM1M2(rcAddressFrontal, leftSpeed, rightSpeed); 
            }
            catch(std::exception &e)
            {
                ROS_ERROR("Mobile base.-> Error while writing speeds to roboclaw frontal:");
                ROS_DEBUG("%s", e.what());
            }
            try
            {
                rcLateral->SpeedM1M2(rcAddressLateral, frontSpeed, rearSpeed); 
            }
            catch(std::exception &e)
            {
                ROS_ERROR("Mobile base.-> Error while writing speeds to roboclaw frontal:");
                ROS_DEBUG("%s", e.what());
            }
            newData = false;
        }
        rate.sleep();
        ros::spinOnce();
    }

    delete rcFrontal;
    delete rcLateral;
}
