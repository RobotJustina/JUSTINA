#!/usr/bin/env python
import serial, time, sys, math
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from hardware_tools import roboclaw_driver as Roboclaw1
from hardware_tools import roboclaw_driver1 as Roboclaw2
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
    global leftSpeed
    global rightSpeed
    global rearSpeed
    global frontSpeed


    leftSpeed = 0
    rightSpeed = 0
    rearSpeed = 0
    frontSpeed = 0
    newSpeedData = True

def callbackSpeeds(msg):
    global leftSpeed
    global rightSpeed
    global frontSpeed   #w3
    global rearSpeed    #w4
    global newSpeedData
    #Speeds are assumed to come in float in [-1,1] for each tire. The values need to be transformed to values in [0,127]
    #A float value of -1, indicates the maximum speed backwards
    #Similar for +1
    leftSpeed = msg.data[0]
    rightSpeed = msg.data[1]
    frontSpeed = (rightSpeed - leftSpeed)/2.0
    rearSpeed = (leftSpeed - rightSpeed)/2.0

    maxValue = 0;
    if(math.fabs(leftSpeed) > maxValue):
        maxValue = math.fabs(leftSpeed);
    if(math.fabs(rightSpeed) > maxValue):
        maxValue = math.fabs(rightSpeed);
    if(math.fabs(frontSpeed) > maxValue):
        maxValue = math.fabs(frontSpeed);
    if(math.fabs(rearSpeed) > maxValue):
        maxValue = math.fabs(rearSpeed);

    if maxValue > 1.0:
        leftSpeed /= maxValue;
        rightSpeed /= maxValue;
        frontSpeed /= maxValue;
        rearSpeed /= maxValue;


    newSpeedData = True

def callbackCmdVel(msg):
    global leftSpeed    #w1
    global rightSpeed   #w2
    global frontSpeed   #w3
    global rearSpeed    #w4
    global newSpeedData

    leftSpeed = msg.linear.x - msg.angular.z * 0.48/2.0
    rightSpeed = msg.linear.x + msg.angular.z * 0.48/2.0
    frontSpeed = msg.linear.y + msg.angular.z * 0.48/2.0
    rearSpeed = msg.linear.y - msg.angular.z * 0.48/2.0

    maxValueFront = 0;
    maxValueLateral = 0;
    if(math.fabs(leftSpeed) > maxValueFront):
        maxValueFront = math.fabs(leftSpeed);
    if(math.fabs(rightSpeed) > maxValueFront):
        maxValueFront = math.fabs(rightSpeed);
    if(math.fabs(frontSpeed) > maxValueLateral):
        maxValueLateral = math.fabs(frontSpeed);
    if(math.fabs(rearSpeed) > maxValueLateral):
        maxValueLateral = math.fabs(rearSpeed);

    if maxValueLateral > 1.0:
        leftSpeed /= maxValueLateral;
        rightSpeed /= maxValueLateral;
        frontSpeed /= maxValueLateral;
        rearSpeed /= maxValueLateral;
        maxValueFront /= maxValueLateral;
    if maxValueFront > 2.0:
        leftSpeed /= maxValueFront;
        rightSpeed /= maxValueFront;
        frontSpeed /= maxValueFront;
        rearSpeed /= maxValueFront;



    #if leftSpeed > 1:
    #    leftSpeed = 1
    #elif leftSpeed < -1:
    #    leftSpeed = -1

    #if rightSpeed > 1:
    #    rightSpeed = 1
    #elif rightSpeed < -1:
    #    rightSpeed = -1

    #if frontSpeed > 1:
    #    frontSpeed = 1
    #elif frontSpeed < -1:
    #    frontSpeed = -1

    #if rearSpeed > 1:
    #    rearSpeed = 1
    #elif rearSpeed < -1:
    #    rearSpeed = -1

    #print "leftSpeed: " + str(leftSpeed) + " rightSpeed: " + str(rightSpeed) + " frontSpeed: " + str(frontSpeed) + " rearSpeed: " + str(rearSpeed)
    newSpeedData = True

def calculateOdometry(currentPos, leftEnc, rightEnc, rearEnc, frontEnc): #Encoder measurements are assumed to be in ticks
    leftEnc = leftEnc / 158891.2 #From ticks to meters
    rightEnc = rightEnc /158891.2
    rearEnc = rearEnc / 336857.5
    frontEnc = frontEnc / 336857.5
    deltaTheta = (rightEnc - leftEnc + frontEnc - rearEnc)/0.48/2.0 #0.48 is the robot diameter

    if math.fabs(deltaTheta) >= 0.00001:
        rgX = (leftEnc + rightEnc)/(2*deltaTheta)
        rgY = (rearEnc + frontEnc)/(2*deltaTheta)
        deltaX = rgX*math.sin(deltaTheta) + rgY*(1-math.cos(deltaTheta))
        deltaY = rgX*(1-math.cos(deltaTheta)) + rgY*math.sin(deltaTheta)
    else:
        deltaX = (leftEnc + rightEnc)/2
        deltaY = (rearEnc + frontEnc)/2
    currentPos[0] += deltaX * math.cos(currentPos[2]) - deltaY * math.sin(currentPos[2])
    currentPos[1] += deltaX * math.sin(currentPos[2]) + deltaY * math.cos(currentPos[2])
    currentPos[2] += deltaTheta
    return currentPos


def main(portName1, portName2, simulated):
    print "MobileBase.->INITIALIZING MOBILE BASE BY MARCOSOFT..."
    #Roboclaw1 = Roboclaw
    #Roboclaw2 = Roboclaw

    ###Connection with ROS
    rospy.init_node("omni_mobile_base")
    pubOdometry = rospy.Publisher("mobile_base/odometry", Odometry, queue_size = 1)
    pubBattery = rospy.Publisher("robot_state/base_battery", Float32, queue_size = 1)

    subStop = rospy.Subscriber("robot_state/stop", Empty, callbackStop)
    subSpeeds = rospy.Subscriber("/hardware/mobile_base/speeds", Float32MultiArray, callbackSpeeds)
    subCmdVel = rospy.Subscriber("/hardware/mobile_base/cmd_vel", Twist, callbackCmdVel)

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(30)
    ###Communication with the Roboclaw
    if not simulated:
        print "MobileBase.-> Trying to open serial port on \"" + portName1 + "\""
        Roboclaw1.Open(portName2, 38400) #ttyACM0  --- M1: front  --- M2: rear #Port names are inverted due to the way motors are wired
        print "MobileBase.-> Trying to open serial port on \"" + portName2 + "\""
        Roboclaw2.Open(portName1, 38400) #ttyACM1  --- M1: right  --- M2: left

        address1 = 0x80
        address2 = 0x80
        print "MobileBase.-> Serial port openned on \"" + portName1 + "\" at 38400 bps (Y)"
        print "MobileBase.-> Serial port openned on \"" + portName2 + "\" at 38400 bps (Y)"
        print "MobileBase.-> Clearing previous encoders readings"
        #print Roboclaw1.ReadEncM1(address1)
        #print Roboclaw2.ReadEncM2(address2)
        Roboclaw1.ResetEncoders(address1)
        Roboclaw2.ResetEncoders(address2)
    ###Variables for setting tire speeds
    global leftSpeed
    global rightSpeed
    global frontSpeed
    global rearSpeed
    global newSpeedData
    leftSpeed = 0
    rightSpeed = 0
    frontSpeed = 0
    rearSpeed = 0
    newSpeedData = False
    speedCounter = 5
    ###Variables for odometry
    robotPos = [0, 0, 0]

    while not rospy.is_shutdown():
        if newSpeedData:
            newSpeedData = False
            speedCounter = 5
            if not simulated:
                leftSpeed = int(leftSpeed*16.0/35.0*110)
                rightSpeed = int(rightSpeed*16.0/35.0*110)
                frontSpeed = int(frontSpeed*110)
                rearSpeed = int(rearSpeed*110)
                #print "lS: " + str(leftSpeed) + " rS: " + str(rightSpeed) + " fS: " + str(frontSpeed) + " rS: " + str(rearSpeed)
                try:
                    if leftSpeed >= 0:
                        Roboclaw2.ForwardM1(address2, leftSpeed)
                    else:
                        Roboclaw2.BackwardM1(address2, -leftSpeed)

                    if rightSpeed >= 0:
                        Roboclaw2.ForwardM2(address2, rightSpeed)
                    else:
                        Roboclaw2.BackwardM2(address2, -rightSpeed)

                    if frontSpeed >= 0:
                        Roboclaw1.BackwardM1(address1, frontSpeed)
                    else:
                        Roboclaw1.ForwardM1(address1, -frontSpeed)

                    if rearSpeed >= 0:
                        Roboclaw1.BackwardM2(address1, rearSpeed)
                    else:
                        Roboclaw1.ForwardM2(address1, -rearSpeed)
                except:
                    print "MobileBase.->Error trying to write speeds :("
                    #Roboclaw1.ForwardM1(address1, 0)
                    #Roboclaw1.ForwardM2(address1, 0)
                    #Roboclaw2.ForwardM1(address2, 0)
                    #Roboclaw2.ForwardM2(address2, 0)
        else:
            speedCounter -= 1
            if speedCounter == 0:
                if not simulated:
                    Roboclaw1.ForwardM1(address1, 0)
                    Roboclaw1.ForwardM2(address1, 0)
                    Roboclaw2.ForwardM1(address2, 0)
                    Roboclaw2.ForwardM2(address2, 0)
                else:
                    leftSpeed = 0
                    rightSpeed = 0
                    frontSpeed = 0
                    rearSpeed = 0

            if speedCounter < -1:
                speedCounter = -1
        if not simulated:
            a1, encoderLeft, a2 = Roboclaw2.ReadEncM1(address2)
            #print Roboclaw2.ReadEncM2(address2)
            #print Roboclaw2.ReadEncM1(address2)
            #print Roboclaw1.ReadEncM1(address1)
            #print Roboclaw1.ReadEncM2(address1)
            b1, encoderRight, b2 = Roboclaw2.ReadEncM2(address2) #The negative sign is just because it is the way the encoders are wired to the roboclaw
            c1, encoderRear, c2 =  Roboclaw1.ReadEncM2(address1)
            d1, encoderFront, d2 = Roboclaw1.ReadEncM1(address1)
            #print "encLeft: " + str(encoderLeft) + " encFront: " + str(encoderFront)
            Roboclaw1.ResetEncoders(address1)
            Roboclaw2.ResetEncoders(address2)
            encoderFront *= -1;
            encoderRear  *= -1;
        else:
            encoderLeft = leftSpeed * 0.05 * 158891.2
            encoderRight = rightSpeed * 0.05 * 158891.2
            encoderFront = frontSpeed * 0.05 * 336857.5
            encoderRear = rearSpeed * 0.05 * 336857.5
        ###Odometry calculation
        robotPos = calculateOdometry(robotPos, encoderLeft, encoderRight, encoderRear, encoderFront)
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
        motorBattery = 12.0
        if not simulated:
            motorBattery = Roboclaw1.ReadMainBatteryVoltage(address1)[1]/10.0 + 0.2 #There is an offset in battery reading
            #print motorBattery
        msgBattery = Float32()
        msgBattery.data = motorBattery
        pubBattery.publish(msgBattery)
        rate.sleep()
    #End of while
    if not simulated:
        Roboclaw1.Close()
        Roboclaw2.Close()
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
            if "--port1" in sys.argv:
                portName1 = sys.argv[sys.argv.index("--port1") + 1]
            if "--port2" in sys.argv:
                portName2 = sys.argv[sys.argv.index("--port2") + 1]
            if "--simul" in sys.argv:
                simulated = True
            main(portName1, portName2, simulated)
    except rospy.ROSInterruptException:
        pass
