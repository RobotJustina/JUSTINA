#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
#from hardware_tools import dynamixel_lib as Dynamixel
from hardware_tools import Dynamixel
import tf


global modeTorque
modeTorque = 2

def callbackTorque(msg):
    global dynMan1
    global modeTorque
    torquePan = 0.0        ## Torque magnitude 
    torquePanCCW = True    ## Turn direction 

    torqueTilt = 0.0
    torqueTiltCCW = True

    if modeTorque != 0:
        ## Change to Torque mode
        dynMan1.SetCWAngleLimit(5, 0)
        dynMan1.SetCCWAngleLimit(5, 0)

        dynMan1.SetCWAngleLimit(1, 0)
        dynMan1.SetCCWAngleLimit(1, 0)

        dynMan1.SetTorqueEnable(5, 0)
        dynMan1.SetTorqueEnable(1, 0)
        
        # dynMan1.GetRegistersValues(5)
        # dynMan1.GetRegistersValues(1)
        print "HardwareHead.->Mode Torque...   "
        modeTorque = 0

    if msg.data[0] < 0:
        torquePan = int(-1*100*msg.data[0])
        torquePanCCW = False
    else:
        torquePan = int(100*msg.data[0])

    if msg.data[1] < 0:
        torqueTilt = int(-1*100*msg.data[1])
        torqueTiltCCW = False
    else:
        torqueTilt = int(100*msg.data[1])

    print "HardwareHead.->Torque.... " + str(torquePan) + "   " + str(torqueTilt)

    ## Send 0-1023 magnitude torque, and the torquePanCCW means the turn direction 
    dynMan1.SetTorqueVale(5, torquePan, torquePanCCW)
    dynMan1.SetTorqueVale(1, torqueTilt, torqueTiltCCW)


def callbackPosHead(msg):
    global goalPan;
    global goalTilt;
    global dynMan1
    global modeTorque

    #if modeTorque != 1:
        ## Change to Position mode
    dynMan1.SetCWAngleLimit(5, 0)
    dynMan1.SetCCWAngleLimit(5, 4095)

    dynMan1.SetCWAngleLimit(1, 0)
    dynMan1.SetCCWAngleLimit(1, 4095)
        
    dynMan1.SetTorqueEnable(5, 1)
    dynMan1.SetTorqueEnable(1, 1)
        
    dynMan1.SetMovingSpeed(5, 90)
    dynMan1.SetMovingSpeed(1, 90)
        
    # print "HardwareHead.->Mode Position...   "
    #modeTorque = 1

    ### Set GoalPosition 
    goalPosPan = msg.data[0]
    goalPosTilt = msg.data[1]

    if goalPosPan < -1.1:
        goalPosPan = -1.1
    if goalPosPan > 1.1:
        goalPosPan = 1.1
    if goalPosTilt < -0.9:
        goalPosTilt = -0.9
    if goalPosTilt > 0:
        goalPosTilt = 0

    goalPan = goalPosPan;
    goalTilt = goalPosTilt;

    # Conversion float to bits
    goalPosTilt = int(( (goalPosTilt)/(360.0/4095.0*3.14159265358979323846/180.0) ) + 2520)
    goalPosPan = int((  (goalPosPan)/(360.0/4095.0*3.14159265358979323846/180.0) ) + 2040 )

    if goalPosTilt >= 0 and goalPosTilt <= 4095 and goalPosPan >= 1023 and goalPosPan <=3069:
        #print "Trying to write servo pos... ->"
        dynMan1.SetGoalPosition(5, goalPosPan)
        dynMan1.SetGoalPosition(1, goalPosTilt)
    #else:
     #   print "HEAD.-> Error: Incorrect goal position.... "

def callbackPosHeadSimul(msg):
    ### Set GoalPosition
    global goalPan
    global goalTilt
    goalPan = msg.data[0]
    goalTilt = msg.data[1]


def printHelp():
    print "HEAD NODE. Options:"
    print "\t --port \t Serial port name. If not provided, the default value is \"/dev/ttyACM0\""
    print "\t --simul\t Simulation mode."
    print " - Head can be moved by publishing either head/goal_pose or"
    print " - head/torque. Torque must be values in [-1, 1] where a value of 1 "
    print " - represents the maximum torque that each motor can generate."
    print " - Positions (both current and goal) are in [rad]"
    print "PLEASE DON'T TRY TO OPERATE JUSTINA IF YOU ARE NOT QUALIFIED ENOUGH."


def main(portName, portBaud):
    print "HardwareHead.->INITIALIZING HEAD NODE..."

    ###Communication with dynamixels:
    global dynMan1
    global goalPan;
    global goalTilt;
    print "HardwareHead.->Trying to open port on " + portName + " at " + str(portBaud)
    dynMan1 = Dynamixel.DynamixelMan(portName, portBaud)
    pan = 0;
    tilt = 0;
    i = 0

    ### Set controller parameters
    dynMan1.SetDGain(1, 25)
    dynMan1.SetPGain(1, 16)
    dynMan1.SetIGain(1, 1)
    
    dynMan1.SetDGain(5, 25)
    dynMan1.SetPGain(5, 16)
    dynMan1.SetIGain(5, 1)


    ### Set servos features
    #dynMan1.SetMaxTorque(1, 1023)
    dynMan1.SetTorqueLimit(1, 512)
    #dynMan1.SetHighestLimitTemperature(1, 80)
    #dynMan1.SetMaxTorque(5, 1023)
    dynMan1.SetTorqueLimit(5, 512)
    #dynMan1.SetHighestLimitTemperature(5, 80)
    
    ###Connection with ROS
    rospy.init_node("head")
    br = tf.TransformBroadcaster()
    jointStates = JointState()
    jointStates.name = ["pan_connect", "tilt_connect"]
    jointStates.position = [0, 0]
    
    ## Subscribers
    subPosition = rospy.Subscriber("/hardware/head/goal_pose", Float32MultiArray, callbackPosHead)
    pubCurrentPose = rospy.Publisher("/hardware/head/current_pose", Float32MultiArray, queue_size = 1);
    #subTorque = rospy.Subscriber("/torque", Float32MultiArray, callbackTorque)
    pubJointStates = rospy.Publisher("/joint_states", JointState, queue_size = 1)
    pubBatery = rospy.Publisher("/hardware/robot_state/head_battery", Float32, queue_size = 1)
    msgCurrentPose = Float32MultiArray()
    msgCurrentPose.data = [0, 0]
    

    dynMan1.SetCWAngleLimit(5, 1023)
    dynMan1.SetCCWAngleLimit(5, 3069)

    dynMan1.SetCWAngleLimit(1, 0)
    dynMan1.SetCCWAngleLimit(1, 4095)
    
    dynMan1.SetGoalPosition(5, 2040)
    dynMan1.SetGoalPosition(1, 2520)
 
    dynMan1.SetTorqueEnable(5, 1)
    dynMan1.SetTorqueEnable(1, 1)
     
    dynMan1.SetMovingSpeed(5, 90)
    dynMan1.SetMovingSpeed(1, 90)
    loop = rospy.Rate(50)

    lastPan = 0.0;
    lastTilt = 0.0;
    goalPan = 0;
    goalTilt = 0;
    speedPan = 0.1 #These values should represent the Dynamixel's moving_speed 
    speedTilt = 0.1
    while not rospy.is_shutdown():
        # Pose in bits
        #panPose = dynMan1.GetPresentPosition(5)
        #tiltPose = dynMan1.GetPresentPosition(1)
        

        # Pose in rad
        #if panPose != None:
        #    pan = (panPose - 1750)*360/4095*3.14159265358979323846/180
        #    if abs(lastPan-pan) > 0.78539816339:
        #        pan = lastPan
        #else:
        #    pan = lastPan

        #if tiltPose != None:
        #    tilt = (tiltPose - 970)*360/4095*3.14159265358979323846/180
        #    if abs(lastTilt-tilt) > 0.78539816339:
        #        tilt = lastTilt
        #else:
        #    tilt = lastTilt
        #SINCE READING IS NOT WORKING, WE ARE FAKING THE REAL SERVO POSE
        deltaPan = goalPan - pan;
        deltaTilt = goalTilt - tilt;
        if deltaPan > speedPan:
            deltaPan = speedPan;
        if deltaPan < -speedPan:
            deltaPan = -speedPan;
        if deltaTilt > speedTilt:
            deltaTilt = speedTilt;
        if deltaTilt < -speedTilt:
            deltaTilt = -speedTilt;
        pan += deltaPan
        tilt += deltaTilt
        
        jointStates.header.stamp = rospy.Time.now()
        jointStates.position[0] = pan
        #jointStates.position[1] = -tilt - 0.08 #goes upwards, but to keep a dextereous system, positive tilt should go downwards
        jointStates.position[1] = -tilt - 0.04#goes upwards, but to keep a dextereous system, positive tilt should go downwards
        pubJointStates.publish(jointStates)  #We substract 0.1 to correct an offset error due to the real head position
        msgCurrentPose.data = [pan, tilt]
        pubCurrentPose.publish(msgCurrentPose)

        if i == 10:
            msgBatery = float(dynMan1.GetPresentVoltage(5)/10.0)
            pubBatery.publish(msgBatery)
            i=0
        i+=1
        
        lastPan = pan
        lastTilt = tilt 
        loop.sleep()

def mainSimul():
    print "INITIALIZING HEAD NODE IN SIMULATION MODE BY MARCOSOFT..."
    ###Connection with ROS
    rospy.init_node("head")
    br = tf.TransformBroadcaster()
    jointStates = JointState()
    jointStates.name = ["pan_connect", "tilt_connect"]
    jointStates.position = [0 ,0]

    subPosition = rospy.Subscriber("head/goal_pose", Float32MultiArray, callbackPosHeadSimul)
    pubHeadPose = rospy.Publisher("head/current_pose", Float32MultiArray, queue_size = 1);
    pubJointStates = rospy.Publisher("/joint_states", JointState, queue_size = 1)
    pubHeadBattery = rospy.Publisher("/hardware/robot_state/head_battery", Float32, queue_size=1)
    
    loop = rospy.Rate(30)

    global goalPan
    global goalTilt
    goalPan = 0
    goalTilt = 0
    pan = 0;
    tilt = 0;
    speedPan = 0.1 #These values should represent the Dynamixel's moving_speed 
    speedTilt = 0.1
    msgCurrentPose = Float32MultiArray()
    msgCurrentPose.data = [0, 0]
    while not rospy.is_shutdown():
        deltaPan = goalPan - pan;
        deltaTilt = goalTilt - tilt;
        if deltaPan > speedPan:
            deltaPan = speedPan;
        if deltaPan < -speedPan:
            deltaPan = -speedPan;
        if deltaTilt > speedTilt:
            deltaTilt = speedTilt;
        if deltaTilt < -speedTilt:
            deltaTilt = -speedTilt;
        pan += deltaPan
        tilt += deltaTilt
        jointStates.header.stamp = rospy.Time.now()
        jointStates.position[0] = pan
        jointStates.position[1] = -tilt #A tilt > 0 goes upwards, but to keep a dextereous system, positive tilt should go downwards
        pubJointStates.publish(jointStates)
        #print "Poses: " + str(panPose) + "   " + str(tiltPose)
        msgCurrentPose.data = [pan, tilt]
        pubHeadPose.publish(msgCurrentPose)
        msgBattery = Float32()
        msgBattery.data = 12.0
        pubHeadBattery.publish(msgBattery);
        loop.sleep()

if __name__ == '__main__':
    try:
        if "--help" in sys.argv:
            printHelp()
        elif "-h" in sys.argv:
            printHelp()
        else:
            portName = "/dev/justinaHead"
            portBaud = 200000
            if "--port" in sys.argv:
                portName = sys.argv[sys.argv.index("--port") + 1]
            if "--baud" in sys.argv:
                portBaud = int(sys.argv[sys.argv.index("--baud") + 1])
            if "--simul" in sys.argv:
                mainSimul()
            else:
                main(portName, portBaud)
    except rospy.ROSInterruptException:
        pass
