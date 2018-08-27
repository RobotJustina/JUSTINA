#include <hardware_tools/JrkManager.hpp>

#define WRITE7BITDATA(byte) ((byte)&0x7F)

//#define DEBUG 0

#ifdef DEBUG
#include <cstdio>
#endif

JrkManager::JrkManager(const std::string port/*=""*/, uint32_t baudrate/*=9600*/, uint32_t timeout/*=0*/)
    : _serial(new serial::Serial(port, baudrate, serial::Timeout::simpleTimeout(timeout))) {};

JrkManager::JrkManager(serial::Serial* serial) : _serial(serial){};

JrkManager::~JrkManager(){};

void JrkManager::reset(serial::Serial* serial){
    _serial.reset(serial);
}

void JrkManager::sendBaudRateIndication(){
    writeByte(baudRateIndication);
}

void JrkManager::motorOff(){
    writeByte(motorOffCommand);
}

void JrkManager::setTarget(uint16_t target){
    // split target bytes
    uint8_t lowerByte = target & 0x1F;
    uint8_t upperByte = (target >> 5) & 0x7F;

#ifdef DEBUG
    printf(">> setTarget: %04d\n", target);
#endif

    uint8_t write_buffer[2];
    write_buffer[0] = setTargetCommand + lowerByte;
    write_buffer[1] = WRITE7BITDATA(upperByte);
    writeBuffer(write_buffer, 2);
}

uint16_t JrkManager::get(uint8_t command_byte){
    writeByte(command_byte);

    return read16BitData();
}

void JrkManager::writeByte(uint8_t dataByte){
#ifdef DEBUG
    printf(">> writeByte: 0x%02x\n", dataByte);
#endif
    uint8_t write_buffer[1];
    write_buffer[0] = dataByte;
    writeBuffer(write_buffer, 1);
}

void JrkManager::write7BitData(uint8_t data){
    writeByte(data & 0x7F);
}

void JrkManager::writeBuffer(uint8_t * write_buffer, uint8_t size){
    _serial->write(write_buffer, size);
    _serial->flushOutput();
}

uint16_t JrkManager::read16BitData(){
    size_t bytes;
    uint8_t read_buffer[8];
    bytes = _serial->read(read_buffer, 2);
    if (bytes < 2)
        throw JrkTimeout();

    uint16_t value = (read_buffer[1] << 8) | (read_buffer[0] & 0xFF);

#ifdef DEBUG
    printf("<< read16BitData: %04d\n", value);
#endif

    return value;
}
