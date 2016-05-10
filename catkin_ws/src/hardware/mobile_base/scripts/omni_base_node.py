#!/usr/bin/env python
import serial, time, sys, math
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from hardware_tools import Roboclaw
import tf

def printHelp():
    print "MOBILE BASE BY MARCOSOFT. Options:"
    print "\t --port \t Serial port name. If not provided, the default value is \"/dev/ttyACM0\""
    print "\t --simul\t Simulation mode."
    print " - Mobile base can be moved by publishing either mobile_base/cmd_vel or"
    print " - mobile_base/speeds. Speeds must be values in [-1, 1] where a value of 1 "
    print " - represents the maximum speed that each motor can generate."
    print "PLEASE DON'T TRY TO OPERATE JUSTINA IF YOU ARE NOT QUALIFIED ENOUGH."

def callbackStop(msg):
    leftSpeed = 0
    rightSpeed = 0
    newSpeedData = True

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

def callbackCmdVel(msg):
    global leftSpeed    #w1
    global rightSpeed   #w2
    global frontSpeed   #w3
    global backSpeed    #w4
    global newSpeedData
    L = 0.25 #Robot diameter/2
    leftSpeed = -msg.linear.x + msg.linear.y + L*msg.angular.z
    frontSpeed = -msg.linear.x - msg.linear.y - L*msg.angular.z
    rightSpeed = msg.linear.x - msg.linear.y + L*msg.angular.z
    backSpeed = msg.linear.x + msg.linear.y - L*msg.angular.z
    if leftSpeed > 1:
        leftSpeed = 1
    elif leftSpeed < -1:
        leftSpeed = -1

    if rightSpeed > 1:
        rightSpeed = 1
    elif rightSpeed < -1:
        rightSpeed = -1

    if frontSpeed > 1:
        frontSpeed = 1
    elif frontSpeed < -1:
        frontSpeed = -1
        
    if backSpeed > 1:
        backSpeed = 1
    elif backSpeed < -1:
        backSpeed = -1

    m1Speed
        
    newSpeedData = True

def calculateOdometry(currentPos, leftEnc, rightEnc): #Encoder measurements are assumed to be in ticks
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
    

def main(portName1, portName2, simulated):
    print "INITIALIZING MOBILE BASE BY MARCOSOFT..."
    ###Connection with ROS
    rospy.init_node("mobile_base")
    pubOdometry = rospy.Publisher("mobile_base/odometry", Odometry, queue_size = 1)
    pubBattery = rospy.Publisher("robot_state/base_battery", Float32, queue_size = 1)
    subSpeeds = rospy.Subscriber("robot_state/stop", Empty, callbackStop)
    subSpeeds = rospy.Subscriber("mobile_base/speeds", Float32MultiArray, callbackSpeeds)
    subCmdVel = rospy.Subscriber("mobile_base/cmd_vel", Twist, callbackCmdVel)
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)
    ###Communication with the Roboclaw
    if not simulated:
        print "MobileBase.-> Trying to open serial port on \"" + portName1 + "\""
        Roboclaw1.Open(portName1, 38400)
        print "MobileBase.-> Trying to open serial port on \"" + portName2 + "\""
        Roboclaw2.Open(portName2, 38400)
        address = 0x80
        print "MobileBase.-> Serial port openned on \"" + portName1 + "\" at 38400 bps (Y)"
        print "MobileBase.-> Clearing previous encoders readings"
        Roboclaw1.ResetQuadratureEncoders(address)
        Roboclaw2.ResetQuadratureEncoders(address)
    ###Variables for setting tire speeds
    global leftSpeed
    global rightSpeed
    global frontSpeed
    global backSpeed
    global newSpeedData
    leftSpeed = 0
    rightSpeed = 0
    frontSpeed = 0
    backSpeed = 0
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
                frontSpeed = int(frontSpeed*63)
                backSpeed = int(backSpeed*63)
                if leftSpeed >= 0:
                    Roboclaw1.DriveForwardM2(address, leftSpeed)
                else:
                    Roboclaw1.DriveBackwardsM2(address, -leftSpeed)
                if rightSpeed >= 0:
                    Roboclaw1.DriveForwardM1(address, rightSpeed)
                else:
                    Roboclaw1.DriveBackwardsM1(address, -rightSpeed)
                if backSpeed >= 0:
                    Roboclaw2.DriveForwardM1(address, backSpeed)
                else:
                    Roboclaw2.DriveBackwardsM1(address, -backSpeed)
                if frontSpeed >= 0:
                    Roboclaw2.DriveForwardM2(address, frontSpeed)
                else:
                    Roboclaw2.DriveBackwardsM2(address, -frontSpeed)
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
        msgOdom = Odometry()
        msgOdom.header.stamp = rospy.Time.now()
        msgOdom.pose.pose.position.x = robotPos[0]
        msgOdom.pose.pose.position.y = robotPos[1]
        msgOdom.pose.pose.position.z = 0
        msgOdom.pose.pose.orientation.x = 0
        msgOdom.pose.pose.orientation.y = 0
        msgOdom.pose.pose.orientation.z = math.sin(robotPos[2]/2)
        msgOdom.pose.pose.orientation.w = math.cos(robotPos[2]/2)
        pubOdometry.publish(msgOdom)
        ###Reads battery and publishes the corresponding topic
        motorBattery = 11.1
        if not simulated:
            motorBattery = Roboclaw.ReadMainBattVoltage(address)
        msgBattery = Float32()
        msgBattery.data = motorBattery
        pubBattery.publish(msgBattery)
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
            portName1 = "/dev/ttyACM0"
            portName2 = "/dev/ttyACM1"
            simulated = False
            if "--port" in sys.argv:
                portName = sys.argv[sys.argv.index("--port") + 1]
            if "--simul" in sys.argv:
                simulated = True
            main(portName1, portName2, simulated)
    except rospy.ROSInterruptException:
        pass
