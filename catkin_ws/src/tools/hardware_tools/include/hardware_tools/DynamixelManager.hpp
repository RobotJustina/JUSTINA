#pragma once

#include <iostream>
#include <vector>
#include <algorithm>
#include <dynamixel_sdk/dynamixel_sdk.h>

#define MODEL_NUMBER    0
#define FIRMWARE_VERSION    2
#define ID  3
#define BAUD_RATE   4
#define RETURN_DELAY_TIME   5
#define CW_ANGLE_LIMIT  6
#define CCW_ANGLE_LIMIT 8
#define HIGHEST_LIMIT_TEMP  11
#define LOWEST_LIMIT_VOLT   12
#define HIGHEST_LIMIT_VOLT  13
#define MAX_TORQUE  14
#define STATUS_RETURN_LEVEL 16
#define ALARM_LED   17
#define ALARM_SHUTDOWN  18
#define TORQUE_ENABLE   24
#define LED 25
#define D_GAIN  26
#define I_GAIN  27
#define P_GAIN  28
#define GOAL_POSITION   30
#define MOVING_SPEED    32
#define TORQUE_LIMIT    34
#define PRESENT_POSITION    36
#define PRESENT_SPEED   38
#define PRESENT_LOAD    40
#define PRESENT_VOLTAGE 42
#define PRESENT_TEMPERATURE 43
#define REGISTERED_INSTRUCTION  44
#define MOVING  46
#define LOCK    47
#define PUNCH   48
#define CURRENT 68
#define PROTOCOL_VERSION    1.0

class DynamixelManager{
    public:
        DynamixelManager(std::string portName, int baudRate, bool enableBulkRead = false, std::vector<int> ids = std::vector<int>(), bool enableSyncWrite = false);
        DynamixelManager();
        ~DynamixelManager();
        void init(std::string portName, int baudRate, bool enableBulkRead = false, std::vector<int> ids = std::vector<int>(), bool enableSyncWrite = false);
        void close();
        bool readBulkData();
        bool writeSyncGoalPosesData();
        bool writeSyncSpeedsData();
        bool getPresentPosition(int id, unsigned short &position);
        bool enableTorque(int id);
        bool disableTorque(int id);
        bool setMovingSpeed(int id, uint16_t speed);
        bool getMovingSpeed(int id, uint16_t &speed);
        bool setGoalPosition(int id, uint16_t goalPosition);
        bool getGoalPosition(int id, uint16_t &goalPosition);
        
        bool getCWAngleLimit(int id, uint16_t &angleLimit);
        bool setCWAngleLimit(int id, uint16_t angleLimit);
        bool getCCWAngleLimit(int id, uint16_t &angleLimit);
        bool setCCWAngleLimit(int id, uint16_t angleLimit);
        bool setHighestLimitTemperature(int id, uint8_t highestLimitTemp);
        
        bool setTorqueValue(int id, uint16_t torque, bool directionTurn);
        bool setMaxTorque(int id, uint16_t maxTorque);
        bool getMaxTorque(int id, uint16_t &maxTorque);
        bool getTorqueLimit(int id, uint16_t &torqueLimit);
        bool setTorqueLimit(int id, uint16_t torqueLimit);
        
        bool getPresentVoltage(int id, uint8_t &presentVoltage);
        bool getPresentLoad(int id, uint16_t &presentLoad);
        
        bool setDGain(int id, uint8_t dGain);
        bool setIGain(int id, uint8_t iGain);
        bool setPGain(int id, uint8_t pGain);
        
        bool setAlarmShutdown(int id, uint8_t alarmShutdown);
        void enableInfoLevelDebug(){
            this->infoLevelDebug = true;
        }
        void disableInfoLevelDebug(){
            this->infoLevelDebug = false;
        }

    private:
        std::string portName;
        int baudRate;
        dynamixel::PortHandler *portHandler;
        dynamixel::PacketHandler *packetHandler;
        dynamixel::GroupBulkRead * groupBulkRead;
        dynamixel::GroupSyncWrite * groupSyncWriteGoalPos;
        dynamixel::GroupSyncWrite * groupSyncWriteSpeeds;
        bool infoLevelDebug;
        bool enableBulkRead;
        bool enableSyncWrite;
        std::vector<int> idsBulkRead;
};
