#include "hardware_tools/DynamixelManager.hpp"

DynamixelManager::DynamixelManager(std::string portName, int baudRate, bool enableBulkRead, std::vector<int> ids, bool enableSyncWrite){
    this->portName = portName;
    this->baudRate = baudRate;
    this->enableBulkRead = enableBulkRead;
    this->enableSyncWrite = enableSyncWrite;
    init(portName, baudRate, enableBulkRead, ids, enableSyncWrite);
    infoLevelDebug = false;
    std::cout << "Enable sync write: " << enableSyncWrite << std::endl;
}

DynamixelManager::DynamixelManager(){
	infoLevelDebug = false;
}

DynamixelManager::~DynamixelManager(){
    delete portHandler;
    delete packetHandler;
    delete groupBulkRead;
    delete groupSyncWriteGoalPos;
}

void DynamixelManager::init(std::string portName, int baudRate, bool enableBulkRead, std::vector<int> ids, bool enableSyncWrite){
    this->enableBulkRead = enableBulkRead;
    this->enableSyncWrite = enableSyncWrite;
    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux
    this->portHandler = dynamixel::PortHandler::getPortHandler(portName.c_str());

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler
    this->packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if(enableBulkRead)
        this->groupBulkRead = new dynamixel::GroupBulkRead(portHandler, packetHandler);

    if(enableSyncWrite)
      {
	this->groupSyncWriteGoalPos = new dynamixel::GroupSyncWrite(portHandler, packetHandler, GOAL_POSITION, 2);
	this->groupSyncWriteSpeeds  = new dynamixel::GroupSyncWrite(portHandler, packetHandler, MOVING_SPEED, 2);
      }

    // Open port
    if(portHandler->openPort()){
        std::cout << "Succeeded to open the port!" << std::endl;
    }
    else{
        std::cerr << "Failed to open the port!" << std::endl;
        throw;
    }

    if(portHandler->setBaudRate(baudRate)){
        std::cout << "Succeeded to change the baund rate!" << std::endl;
    }
    else{
        std::cout << "Failed to change the baund rate!" << std::endl;
        throw;
    }

    if(enableBulkRead && ids.size() > 0){
        uint8_t dxl_error = 0;
        int dxl_addparam_result = COMM_TX_FAIL;
        for(int i = 0; i < ids.size(); i++){
            dxl_addparam_result = groupBulkRead->addParam(i, PRESENT_POSITION, 2);
            if (dxl_addparam_result != true){
                fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed", i);
            }
            else
                idsBulkRead.push_back(i);
        }
    }
    else if(enableBulkRead){
        std::cout << "Can not set bulk mode because nos ids have found to initi" << std::endl;
        throw;
    }
}

void DynamixelManager::close(){
    portHandler->closePort();
}

bool DynamixelManager::readBulkData(){
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;
    dxl_comm_result = groupBulkRead->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS){
        if(infoLevelDebug)
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        if (dxl_error != 0){
            printf("Error reading bulk data: %s\n", packetHandler->getRxPacketError(dxl_error));
            return false;
        }
    }

    return true;
}
        
bool DynamixelManager::writeSyncGoalPosesData(){
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;
    // Syncwrite goal position
    dxl_comm_result = groupSyncWriteGoalPos->txPacket();
    if (dxl_comm_result != COMM_SUCCESS){
        if(infoLevelDebug)
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        if (dxl_error != 0){
            printf("Error writing sync data: %s\n", packetHandler->getRxPacketError(dxl_error));
            return false;
        }
    }

    // Clear syncwrite parameter storage
    groupSyncWriteGoalPos->clearParam();
}

bool DynamixelManager::writeSyncSpeedsData(){
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;
    // Syncwrite goal position
    dxl_comm_result = groupSyncWriteSpeeds->txPacket();
    if (dxl_comm_result != COMM_SUCCESS){
        if(infoLevelDebug)
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        if (dxl_error != 0){
            printf("Error writing sync data: %s\n", packetHandler->getRxPacketError(dxl_error));
            return false;
        }
    }

    // Clear syncwrite parameter storage
    groupSyncWriteSpeeds->clearParam();
}

bool DynamixelManager::getPresentPosition(int id, unsigned short &position){
    uint8_t dxl_error = 0;
    bool bulkRead = false;
    if(enableBulkRead){
        std::vector<int>::iterator it = std::find(idsBulkRead.begin(), idsBulkRead.end(), id);
        if (it != idsBulkRead.end())
            bulkRead = true;
    }
    if(!bulkRead){
        int dxl_comm_result = COMM_TX_FAIL;
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, PRESENT_POSITION, &position, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS){
            if(infoLevelDebug)
                printf("DX_ID: %d, DX_REG: %d, READ, %s\n", id, PRESENT_POSITION, packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0){
            printf("DX_ID: %d, DX_REG: %d, READ, %s\n", id, PRESENT_POSITION, packetHandler->getRxPacketError(dxl_error));
            return false;
        }
    }
    else{
        int dxl_getdata_result = groupBulkRead->isAvailable(id, PRESENT_POSITION, 2);
        if (dxl_getdata_result != true){
            fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed\n", id);
            return false;
        }
        position = groupBulkRead->getData(id, PRESENT_POSITION, 2);
    }
    return true;
}

bool DynamixelManager::enableTorque(int id){
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, TORQUE_ENABLE, 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
        if(infoLevelDebug)
   			printf("DX_ID: %d, DX_REG: %d, WRITE, %s\n", id, TORQUE_ENABLE, packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0){
        printf("DX_ID: %d, DX_REG: %d, WRITE, %s\n", id, TORQUE_ENABLE, packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    return true;
}

bool DynamixelManager::disableTorque(int id){
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, TORQUE_ENABLE, 0, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
    	if(infoLevelDebug)
   			printf("DX_ID: %d, DX_REG: %d, WRITE, %s\n", id, TORQUE_ENABLE, packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0){
        printf("DX_ID: %d, DX_REG: %d, WRITE, %s\n", id, TORQUE_ENABLE, packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    return true;
}

bool DynamixelManager::setMovingSpeed(int id, uint16_t speed){
    if(!enableSyncWrite){
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, MOVING_SPEED, speed, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS){
            if(infoLevelDebug)
                printf("DX_ID: %d, DX_REG: %d, WRITE, %s\n", id, MOVING_SPEED, packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0){
            printf("DX_ID: %d, DX_REG: %d, WRITE, %s\n", id, MOVING_SPEED, packetHandler->getRxPacketError(dxl_error));
            return false;
        }
        return true;
    }
    else{
        bool dxl_addparam_result = false;
        // Allocate goal position value into byte array
        uint8_t param[2] = {0, 0}; 
        param[0] = DXL_LOBYTE(speed);
        param[1] = DXL_HIBYTE(speed);
        // Add Dynamixel#1 goal position value to the Syncwrite storage
        dxl_addparam_result = groupSyncWriteSpeeds->addParam(id, param);
        if (dxl_addparam_result != true){
            fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", id);
            return false;
        }
    }
}


bool DynamixelManager::getMovingSpeed(int id, uint16_t &speed){
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, MOVING_SPEED, &speed, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
    	if(infoLevelDebug)
   			printf("DX_ID: %d, DX_REG: %d, READ, %s\n", id, MOVING_SPEED, packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0){
        printf("DX_ID: %d, DX_REG: %d, READ, %s\n", id, MOVING_SPEED, packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    return true;
}

bool DynamixelManager::setGoalPosition(int id, uint16_t goalPosition){
    if(!enableSyncWrite){
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, GOAL_POSITION, goalPosition, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS){
            if(infoLevelDebug)
                printf("DX_ID: %d, DX_REG: %d, WRITE, %s\n", id, GOAL_POSITION, packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0){
            printf("DX_ID: %d, DX_REG: %d, WRITE, %s\n", id, GOAL_POSITION, packetHandler->getRxPacketError(dxl_error));
            return false;
        }
    }
    else{
        bool dxl_addparam_result = false;
        // Allocate goal position value into byte array
        uint8_t param[2] = {0, 0}; 
        param[0] = DXL_LOBYTE(goalPosition);
        param[1] = DXL_HIBYTE(goalPosition);
        // Add Dynamixel#1 goal position value to the Syncwrite storage
        dxl_addparam_result = groupSyncWriteGoalPos->addParam(id, param);
        if (dxl_addparam_result != true){
            fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", id);
            return false;
        }
    }
    return true;
}

bool DynamixelManager::getGoalPosition(int id, uint16_t &goalPosition){
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, GOAL_POSITION, &goalPosition, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
        if(infoLevelDebug)
   			printf("DX_ID: %d, DX_REG: %d, READ, %s\n", id, GOAL_POSITION, packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0){
        printf("DX_ID: %d, DX_REG: %d, READ, %s\n", id, GOAL_POSITION, packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    return true;
}

bool DynamixelManager::getCWAngleLimit(int id, uint16_t &angleLimit){
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, CW_ANGLE_LIMIT, &angleLimit, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
    	if(infoLevelDebug)
   			printf("DX_ID: %d, DX_REG: %d, READ, %s\n", id, CW_ANGLE_LIMIT, packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0){
        printf("DX_ID: %d, DX_REG: %d, READ, %s\n", id, CW_ANGLE_LIMIT, packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    return true;
}

bool DynamixelManager::setCWAngleLimit(int id, uint16_t angleLimit){
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, CW_ANGLE_LIMIT, angleLimit, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
    	if(infoLevelDebug)
   			printf("DX_ID: %d, DX_REG: %d, WRITE, %s\n", id, CW_ANGLE_LIMIT, packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0){
        printf("DX_ID: %d, DX_REG: %d, WRITE, %s\n", id, CW_ANGLE_LIMIT, packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    return true;
}

bool DynamixelManager::getCCWAngleLimit(int id, uint16_t &angleLimit){
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, CCW_ANGLE_LIMIT, &angleLimit, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
    	if(infoLevelDebug)
   			printf("DX_ID: %d, DX_REG: %d, READ, %s\n", id, CCW_ANGLE_LIMIT, packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0){
        printf("DX_ID: %d, DX_REG: %d, READ, %s\n", id, CCW_ANGLE_LIMIT, packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    return true;
}

bool DynamixelManager::setCCWAngleLimit(int id, uint16_t angleLimit){
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, CCW_ANGLE_LIMIT, angleLimit, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
    	if(infoLevelDebug)
   			printf("DX_ID: %d, DX_REG: %d, WRITE, %s\n", id, CCW_ANGLE_LIMIT, packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0){
        printf("DX_ID: %d, DX_REG: %d, WRITE, %s\n", id, CCW_ANGLE_LIMIT, packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    return true;
}

bool DynamixelManager::setHighestLimitTemperature(int id, uint8_t highestLimitTemp){
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, HIGHEST_LIMIT_TEMP, highestLimitTemp, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
    	if(infoLevelDebug)
   			printf("DX_ID: %d, DX_REG: %d, WRITE, %s\n", id, HIGHEST_LIMIT_TEMP, packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0){
        printf("DX_ID: %d, DX_REG: %d, WRITE, %s\n", id, HIGHEST_LIMIT_TEMP, packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    return true;
}

bool DynamixelManager::setTorqueValue(int id, uint16_t torque, bool directionTurn){
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    if(directionTurn)
       torque = torque + 1024;
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, MOVING_SPEED, torque, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
    	if(infoLevelDebug)
   			printf("DX_ID: %d, DX_REG: %d, WRITE, %s\n", id, MOVING_SPEED, packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0){
        printf("DX_ID: %d, DX_REG: %d, WRITE, %s\n", id, MOVING_SPEED, packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    return true;
}

bool DynamixelManager::setMaxTorque(int id, uint16_t maxTorque){
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, MAX_TORQUE, maxTorque, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
    	if(infoLevelDebug)
   			printf("DX_ID: %d, DX_REG: %d, WRITE, %s\n", id, MAX_TORQUE, packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0){
        printf("DX_ID: %d, DX_REG: %d, WRITE, %s\n", id, MAX_TORQUE, packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    return true;
}

bool DynamixelManager::getMaxTorque(int id, uint16_t &maxTorque){
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, MAX_TORQUE, &maxTorque, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
    	if(infoLevelDebug)
   			printf("DX_ID: %d, DX_REG: %d, READ, %s\n", id, MAX_TORQUE, packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0){
        printf("DX_ID: %d, DX_REG: %d, READ, %s\n", id, MAX_TORQUE, packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    return true;
}

bool DynamixelManager::getTorqueLimit(int id, uint16_t &torqueLimit){
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, TORQUE_LIMIT, &torqueLimit, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
    	if(infoLevelDebug)
   			printf("DX_ID: %d, DX_REG: %d, READ, %s\n", id, TORQUE_LIMIT, packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0){
        printf("DX_ID: %d, DX_REG: %d, READ, %s\n", id, TORQUE_LIMIT, packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    return true;
}

bool DynamixelManager::setTorqueLimit(int id, uint16_t torqueLimit){
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, TORQUE_LIMIT, torqueLimit, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
    	if(infoLevelDebug)
   			printf("DX_ID: %d, DX_REG: %d, WRITE, %s\n", id, TORQUE_LIMIT, packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0){
        printf("DX_ID: %d, DX_REG: %d, WRITE, %s\n", id, TORQUE_LIMIT, packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    return true;
}


bool DynamixelManager::getPresentVoltage(int id, uint8_t &presentVoltage){
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, id, PRESENT_VOLTAGE, &presentVoltage, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
    	if(infoLevelDebug)
   			printf("DX_ID: %d, DX_REG: %d, READ, %s\n", id, PRESENT_VOLTAGE, packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0){
        printf("DX_ID: %d, DX_REG: %d, READ, %s\n", id, PRESENT_VOLTAGE, packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    return true;
}

bool DynamixelManager::getPresentLoad(int id, uint16_t &presentLoad){
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, PRESENT_LOAD, &presentLoad, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
    	if(infoLevelDebug)
   			printf("DX_ID: %d, DX_REG: %d, READ, %s\n", id, PRESENT_LOAD, packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0){
        printf("DX_ID: %d, DX_REG: %d, READ, %s\n", id, PRESENT_LOAD, packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    return true;
}

bool DynamixelManager::setDGain(int id, uint8_t dGain){
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, D_GAIN, dGain, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
    	if(infoLevelDebug)
   			printf("DX_ID: %d, DX_REG: %d, WRITE, %s\n", id, D_GAIN, packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0){
        printf("DX_ID: %d, DX_REG: %d, WRITE, %s\n", id, D_GAIN, packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    return true;
}

bool DynamixelManager::setIGain(int id, uint8_t iGain){
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, I_GAIN, iGain, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
    	if(infoLevelDebug)
   			printf("DX_ID: %d, DX_REG: %d, WRITE, %s\n", id, I_GAIN, packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0){
        printf("DX_ID: %d, DX_REG: %d, WRITE, %s\n", id, I_GAIN, packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    return true;
}

bool DynamixelManager::setPGain(int id, uint8_t pGain){
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, P_GAIN, pGain, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
    	if(infoLevelDebug)
   			printf("DX_ID: %d, DX_REG: %d, WRITE, %s\n", id, P_GAIN, packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0){
        printf("DX_ID: %d, DX_REG: %d, WRITE, %s\n", id, P_GAIN, packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    return true;
}

bool DynamixelManager::setAlarmShutdown(int id, uint8_t alarmShutdown){
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ALARM_SHUTDOWN, alarmShutdown, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
    	if(infoLevelDebug)
   			printf("DX_ID: %d, DX_REG: %d, WRITE, %s\n", id, ALARM_SHUTDOWN, packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0){
        printf("DX_ID: %d, DX_REG: %d, WRITE, %s\n", id, ALARM_SHUTDOWN, packetHandler->getRxPacketError(dxl_error));
        return false;
    }
    return true;
}
