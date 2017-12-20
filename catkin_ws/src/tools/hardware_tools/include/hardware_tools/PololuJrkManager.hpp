#include <iostream>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

class PololuJrkManager{

    public:
        PololuJrkManager();
        PololuJrkManager(std::string port);
        void init(std::string port);
        int getFeedback(int fd);
        int getScaledFeedback(int fd);
        bool setTarget(int fd, unsigned int target);
        int getTarget(int fb);
        int getErrorFlagsHalting(int fb);
        float getPositionFromFeedback(int feedback);
        int getFeedbackFromPosition(float position);

    private:
        std::string port;
        int dev;

        bool jrkWrite(int fd, unsigned char command);
        int jrkGetVariable(int fd, unsigned char command);
};

