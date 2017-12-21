#ifndef POLOLU_JRK_MANAGER_H
#define POLOLU_JRK_MANAGER_H

#include <array>
#include <exception>
#include <memory>
#include <string>
#include <sstream>
#include <vector>

#include "serial/serial.h"

class JrkManager{
    public:
        JrkManager(const std::string port="", uint32_t baudrate=9600, uint32_t timeout=0);
        JrkManager(serial::Serial* serial);
        virtual ~JrkManager();

        void reset(serial::Serial* serial);

        void sendBaudRateIndication();
        void motorOff();
        void setTarget(uint16_t target);
        uint16_t get(uint8_t command_byte);
        uint16_t getFeedback() {return get(getScaledFeedbackCommand); };
        uint16_t getErrorsHalting() {return get(getErrorsHaltingCommand); };
        uint16_t getErrors() {return get(getErrorsCommand); };

        static const uint8_t baudRateIndication = 0xAA;

        static const uint8_t motorOffCommand = 0xFF;
        static const uint8_t setTargetCommand = 0xC0;
        static const uint8_t getTargetCommand = 0xA3;
        static const uint8_t getFeedbackCommand = 0xA5;
        static const uint8_t getScaledFeedbackCommand = 0xA7;
        static const uint8_t getErrorsHaltingCommand = 0xB3;
        static const uint8_t getErrorsCommand = 0xB5;

    protected:
        void writeBuffer(uint8_t * write_buffer, uint8_t size);
        void writeByte(uint8_t dataByte);
        void write7BitData(uint8_t data);
        uint16_t read16BitData();

    private:
        std::unique_ptr<serial::Serial> _serial;
};

class JrkTimeout : public std::exception {};

const std::array<std::string, 12> JrkErrorStrings = {
    "awaiting command",
    "no power",
    "motor driver error",
    "input invalid",
    "input disconnect",
    "feedback disconnect",
    "maximum current exceeded",
    "serial signal error",
    "serial rx buffer full",
    "serial crc error",
    "serial protocol error",
    "serial tiomeout error"
};

std::string readErrorFlags(uint16_t errorBits)
{
    std::stringstream errors;

    for (size_t i = 0; i < JrkErrorStrings.size(); i++)
    {
        if (errorBits & (1 << i))
        {
            errors << JrkErrorStrings[i] << " ";
        }
    }

    return errors.str();
}

#endif // POLOLU_JRK_MANAGER_H
