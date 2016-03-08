#!/usr/bin/env python
import serial, time, sys, math
import rospy
import Roboclaw
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf

def printHelp():
    print "MOBILE BASE BY MARCOSOFT. Options:"
    print "\t --port \t Serial port name. If not provided, the default value is \"/dev/ttyACM0\""
    print "\t --simul\t Simulation mode."

def callbackSpeeds(msg):
    global leftSpeed
    global rightSpeed
    global newSpeedData
    #Speeds are assumed to come in float in [-1,1] for each tire. The values need to be transformed to values in [0,127]
    #A float value of -1, indicates the maximum speed backwards
    #Similar for +1
    leftSpeed = msg.data[0]
    rightSpeed = msg.data[1]
    newSpeedData = True

def calculateOdometry(currentPos, leftEnc, rightEnc):
    leftEnc = leftEnc * 0.39/980 #From ticks to meters
    rightEnc = rightEnc * 0.39/980
    deltaTheta = (rightEnc - leftEnc)/0.48 #0.48 is the robot diameter
    if math.fabs(deltaTheta) >= 0.0001:
        rg = (leftEnc + rightEnc)/(2*deltaTheta)
        deltaX = rg*math.sin(deltaTheta)
        deltaY = rg*(1-math.cos(deltaTheta))
    else:
        deltaX = (leftEnc + rightEnc)/2
        deltaY = 0
    currentPos[0] += deltaX * math.cos(currentPos[2]) - deltaY * math.sin(currentPos[2])
    currentPos[1] += deltaX * math.sin(currentPos[2]) + deltaY * math.cos(currentPos[2])
    currentPos[2] += deltaTheta
    return currentPos
    

def main(portName, simulated):
    print "INITIALIZING MOBILE BASE BY MARCOSOFT..."
    ###Connection with ROS
    rospy.init_node("mobile_base")
    pubOdometry = rospy.Publisher("mobile_base/odometry", Odometry, queue_size = 1)
    subSpeeds = rospy.Subscriber("mobile_base/speeds", Float32MultiArray, callbackSpeeds)
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)
    ###Communication with the Roboclaw
    if not simulated:
        print "MobileBase.-> Trying to open serial port on \"" + portName + "\""
        Roboclaw.Open(portName, 38400)
        address = 0x80
        print "MobileBase.-> Serial port openned on \"" + portName + "\" at 38400 bps (Y)"
    ###Variables for setting tire speeds
    global leftSpeed
    global rightSpeed
    global newSpeedData
    leftSpeed = 0
    rightSpeed = 0
    newSpeedData = False
    speedCounter = 5
    ###Variables for odometry
    robotPos = [0, 0, 0]
    while not rospy.is_shutdown():
        if newSpeedData:
            newSpeedData = False
            speedCounter = 5
            if not simulated:
                leftSpeed = int(leftSpeed*127)
                rightSpeed = int(rightSpeed*127)
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
                if not simulated:
                    Roboclaw.DriveForwardM1(address, 0)
                    Roboclaw.DriveForwardM2(address, 0)
                else:
                    leftSpeed = 0
                    rightSpeed = 0
            if speedCounter < -1:
                speedCounter = -1
        if not simulated:
            encoderLeft = -Roboclaw.ReadQEncoderM2(address)
            encoderRight = -Roboclaw.ReadQEncoderM1(address) #The negative sign is just because it is the way the encoders are wired to the roboclaw
            Roboclaw.ResetQuadratureEncoders(address)
        else:
            encoderLeft = leftSpeed * 0.1 * 980 / 0.39
            encoderRight = rightSpeed * 0.1 * 980 / 0.39
        ###Odometry calculation
        robotPos = calculateOdometry(robotPos, encoderLeft, encoderRight)
        #print "Encoders: " + str(encoderLeft) + "  " + str(encoderRight)
        ##Odometry and transformations
        ts = TransformStamped()
        ts.header.stamp = rospy.Time.now()
        ts.header.frame_id = "odom"
        ts.child_frame_id = "base_link"
        ts.transform.translation.x = robotPos[0]
        ts.transform.translation.y = robotPos[1]
        ts.transform.translation.z = 0
        ts.transform.rotation = tf.transformations.quaternion_from_euler(0, 0, robotPos[2])
        br.sendTransform((robotPos[0], robotPos[1], 0), ts.transform.rotation, rospy.Time.now(), ts.child_frame_id, ts.header.frame_id)
        rate.sleep()
    #End of while
    if not simulated:
        Roboclaw.Close()
#end of main()

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
