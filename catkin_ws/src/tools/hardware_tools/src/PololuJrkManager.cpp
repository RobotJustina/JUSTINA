#include <hardware_tools/PololuJrkManager.hpp>

PololuJrkManager::PololuJrkManager(){
}

PololuJrkManager::PololuJrkManager(std::string port){
    this->init(port);
}

void PololuJrkManager::init(std::string port){
    this->port = port;
    this->dev = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (this->dev == -1){
        perror(port.c_str());
    }
}

int PololuJrkManager::getFeedback(int fd){
    return jrkGetVariable(dev, 0xA5);
}

int PololuJrkManager::getScaledFeedback(int fd){
    return jrkGetVariable(dev, 0xA7);
}

bool PololuJrkManager::setTarget(int fd, unsigned int target){
    unsigned char command[] = {0xC0 + (target & 0x1F), (target >> 5) & 0x7F};
    if (write(dev, command, sizeof(command)) == -1){
        perror("error writing");
        return false;
    }
    return true;
}

int PololuJrkManager::getTarget(int fb){
    return jrkGetVariable(dev, 0xA3);
}

int PololuJrkManager::getErrorFlagsHalting(int fb){
    return jrkWrite(dev, 0xB3);
}

bool PololuJrkManager::jrkWrite(int fd, unsigned char command){
    if(write(fd, &command, 1) == -1){
        perror("error writing");
        return false;
    }
    tcflush(fd, TCOFLUSH);
    return true;
}

int PololuJrkManager::jrkGetVariable(int fd, unsigned char command){
    if(write(fd, &command, 1) == -1){
        perror("error writing");
        return -1;
    }

    tcflush(fd, TCOFLUSH);

    unsigned char response[2];
    if(read(fd,response,2) != 2){
        perror("error reading");
        return -1;
    }
    
    tcflush(fd, TCOFLUSH);

    return response[0] + 256*response[1];   
}

float PololuJrkManager::getPositionFromFeedback(int feedback){
    return 0.0076491823 * feedback - 0.5021359358;
}

int PololuJrkManager::getFeedbackFromPosition(float position){
    return 130.657375698 * position + 66.7352975312;
}
