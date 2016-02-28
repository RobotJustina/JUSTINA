#!/usr/bin/env python
import serial, time, sys, math
import rospy
import Roboclaw
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry

def printHelp():
    print "MOBILE BASE BY MARCOSOFT. Options:"
    print "\t --port \t Serial port name. If not specified, name is taken from RosParamServer"
    print "\t --simul\t If the parameter is present, mobile base will turn into simulation mode."

def callbackSpeeds(msg):
    global leftSpeed
    global rightSpeed
    global newSpeedData
    leftSpeed = msg.data[0]
    rightSpeed = msg.data[1]
    newSpeedData = True

def main(portName, simulated):
    print "INITIALIZING MOBILE BASE BY MARCOSOFT..."
    rospy.init_node("mobile_base")
    pubOdometry = rospy.Publisher("/hardware/mobile_base/odometry", Odometry, queue_size = 1)
    subSpeeds = rospy.Subscriber("/hardware/mobile_base/speeds", Float32MultiArray, callbackSpeeds)
    rate = rospy.Rate(10)
    print "MobileBase.-> Trying to open serial port on \"" + portName + "\""
    Roboclaw.Open(portName, 38400)
    address = 0x80
    print "MobileBase.-> Serial port openned on \"" + portName + "\" at 38400 bps (Y)"
    global leftSpeed
    global rightSpeed
    global newSpeedData
    speedCounter = 5
    while not rospy.is_shutdown():
        if newSpeedData:
            newSpeedData = False
            speedCounter = 5
            if leftSpeed >= 0:
                Roboclaw.DriveForwardM1(address, leftSpeed)
            else:
                Roboclaw.DriveBackwardsM1(address, -leftSpeed)
            if rightSpeed >= 0:
                Roboclaw.DriveForwardM2(address, rightSpeed)
            else:
                Roboclaw.DriveBackwardsM2(address, -rightSpeed)
        else:
            speedCounter -= 1
            if speedCounter == 0:
                Roboclaw.DriveForwardM1(address, 0)
                Roboclaw.DriveForwardM2(address, 0)
            if speedCounter < -1:
                speedCounter = -1
        encoderLeft = Roboclaw.ReadQEncoderM1(address)
        encoderRight = Roboclaw.ReadQEncoderM2(address)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        if "--help" in sys.argv:
            printHelp()
        elif "-h" in sys.argv:
            printHelp()
        else:
            portName = ""
            simulated = False
            if "--port" in sys.argv:
                portName = sys.argv[sys.argv.index("--port") + 1]
            if "--simul" in sys.argv:
                simulated = True
            main(portName, simulated)
    except rospy.ROSInterruptException:
        pass
