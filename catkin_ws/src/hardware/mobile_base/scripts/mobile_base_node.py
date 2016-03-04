#!/usr/bin/env python
import serial, time, sys, math
import rospy
import Roboclaw
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import tf

def printHelp():
    print "MOBILE BASE BY MARCOSOFT. Options:"
    print "\t --port \t Serial port name. If not provided, the default value is \"/dev/ttyACM0\""

def callbackSpeeds(msg):
    global leftSpeed
    global rightSpeed
    global newSpeedData
    #Speeds are assumed to come in float in [-1,1] for each tire. The values need to be transformed to values in [0,127]
    #A float value of -1, indicates the maximum speed backwards
    #Similar for +1
    tempLeftSpeed = msg.data[0] * 127
    tempRightSpeed = msg.data[1] * 127
    leftSpeed = int(tempLeftSpeed)
    rightSpeed = int(tempRightSpeed)
    newSpeedData = True

def main(portName, simulated):
    print "INITIALIZING MOBILE BASE BY MARCOSOFT..."
    ###Connection with ROS
    rospy.init_node("mobile_base")
    pubOdometry = rospy.Publisher("/hardware/mobile_base/odometry", Odometry, queue_size = 1)
    subSpeeds = rospy.Subscriber("/hardware/mobile_base/speeds", Float32MultiArray, callbackSpeeds)
    rate = rospy.Rate(10)
    ###Communication with the Roboclaw
    print "MobileBase.-> Trying to open serial port on \"" + portName + "\""
    Roboclaw.Open(portName, 38400)
    address = 0x80
    print "MobileBase.-> Serial port openned on \"" + portName + "\" at 38400 bps (Y)"
    ###Variables for setting tire speeds
    global leftSpeed
    global rightSpeed
    global newSpeedData
    newSpeedData = False
    speedCounter = 5
    while not rospy.is_shutdown():
        if newSpeedData:
            newSpeedData = False
            speedCounter = 5
            if leftSpeed >= 0:
                Roboclaw.DriveForwardM2(address, leftSpeed)
            else:
                Roboclaw.DriveBackwardsM2(address, -leftSpeed)
            if rightSpeed >= 0:
                Roboclaw.DriveForwardM1(address, rightSpeed)
            else:
                Roboclaw.DriveBackwardsM1(address, -rightSpeed)
        else:
            speedCounter -= 1
            if speedCounter == 0:
                Roboclaw.DriveForwardM1(address, 0)
                Roboclaw.DriveForwardM2(address, 0)
            if speedCounter < -1:
                speedCounter = -1
        encoderLeft = -Roboclaw.ReadQEncoderM2(address)
        encoderRight = -Roboclaw.ReadQEncoderM1(address) #The negative sign is just because it is the way the encoders are wired to the roboclaw
        #print "Encoders: " + str(encoderLeft) + "  " + str(encoderRight)
        ##Odometry and transformations
        rate.sleep()

if __name__ == '__main__':
    try:
        if "--help" in sys.argv:
            printHelp()
        elif "-h" in sys.argv:
            printHelp()
        else:
            portName = "/dev/ttyACM0"
            simulated = False
            if "--port" in sys.argv:
                portName = sys.argv[sys.argv.index("--port") + 1]
            if "--simul" in sys.argv:
                simulated = True
            main(portName, simulated)
    except rospy.ROSInterruptException:
        pass
