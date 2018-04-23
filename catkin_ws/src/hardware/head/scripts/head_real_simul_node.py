#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from hardware_tools import Dynamixel
import tf


global modeTorque
modeTorque = 2
global simul
#dynMan1
simul = False

def callbackPosHead(msg):
    global goalPan
    global goalTilt
    global goalPansimul
    global goalTiltsimul
    global dynMan1
    global simul

    ### Set GoalPosition
    if simul == True:
        goalPansimul = msg.data[0]
        goalTiltsimul = msg.data[1]

    else:
        dynMan1.SetCWAngleLimit(0, 0)
        dynMan1.SetCCWAngleLimit(0, 4095)

        dynMan1.SetCWAngleLimit(1, 0)
        dynMan1.SetCCWAngleLimit(1, 4095)
        
        dynMan1.SetTorqueEnable(0, 1)
        dynMan1.SetTorqueEnable(1, 1)
        
        dynMan1.SetMovingSpeed(0, 90)
        dynMan1.SetMovingSpeed(1, 90)

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

        goalPan = goalPosPan
        goalTilt = goalPosTilt

        # Conversion float to bits
        goalPosTilt = int(( (goalPosTilt)/(360.0/4095.0*3.14159265358979323846/180.0) ) + 2520)
        goalPosPan = int((  (goalPosPan)/(360.0/4095.0*3.14159265358979323846/180.0) ) + 2040 )

        if goalPosTilt >= 0 and goalPosTilt <= 4095 and goalPosPan >= 1023 and goalPosPan <=3069:
            dynMan1.SetGoalPosition(0, goalPosPan)
            dynMan1.SetGoalPosition(1, goalPosTilt)

def callback_simulated(msg):
    global simul
    simul = msg.data 

def print_help():
    print "HEAD NODE. Options:"
    print "\t --port \t Serial port name. If not provided, the default value is \"/dev/ttyACM0\""
    print "\t --simul\t Simulation mode."
    print " - Head can be moved by publishing either head/goal_pose or"
    print " - head/torque. Torque must be values in [-1, 1] where a value of 1 "
    print " - represents the maximum torque that each motor can generate."
    print " - Positions (both current and goal) are in [rad]"
    print "PLEASE DON'T TRY TO OPERATE JUSTINA IF YOU ARE NOT QUALIFIED ENOUGH."

def main():
    print "HardwareHead.->INITIALIZING HEAD NODE..."

    ###Communication with dynamixels:
    global dynMan1
    global goalPan
    global goalTilt
    global goalPansimul
    global goalTiltsimul
    global simul

    pan = 0
    tilt = 0
    pansimul = 0
    tiltsimul = 0
    i = 0
    
    ###Connection with ROS
    rospy.init_node("head")

    if rospy.has_param('~simul'):
        simul = rospy.get_param('~simul')
    else:
        simul = False
    
    if rospy.has_param('~port'):
        portName = rospy.get_param('~port')
    elif not simul:
        print_help();
        sys.exit();
    if rospy.has_param('~baud'):
        portBaud = rospy.get_param('~baud')
    elif not simul:
        print_help();
        sys.exit();

    if simul==False:
        print "HardwareHead.->Trying to open port on " + portName + " at " + str(portBaud)
        dynMan1 = Dynamixel.DynamixelMan(portName, portBaud)
        ### Set controller parameters
        dynMan1.SetDGain(1, 25)
        dynMan1.SetPGain(1, 16)
        dynMan1.SetIGain(1, 1)
        
        dynMan1.SetDGain(0, 25)
        dynMan1.SetPGain(0, 16)
        dynMan1.SetIGain(0, 1)
    
    
        ### Set servos features
        dynMan1.SetTorqueLimit(1, 512)
        dynMan1.SetTorqueLimit(0, 512)

        dynMan1.SetCWAngleLimit(0, 1023)
        dynMan1.SetCCWAngleLimit(0, 3069)

        dynMan1.SetCWAngleLimit(1, 0)
        dynMan1.SetCCWAngleLimit(1, 4095)
    
        dynMan1.SetGoalPosition(0, 2040)
        dynMan1.SetGoalPosition(1, 2520)
 
        dynMan1.SetTorqueEnable(0, 1)
        dynMan1.SetTorqueEnable(1, 1)
     
        dynMan1.SetMovingSpeed(0, 90)
        dynMan1.SetMovingSpeed(1, 90)

    br = tf.TransformBroadcaster()
    jointStates = JointState()
    jointStates.name = ["pan_connect", "tilt_connect"]
    jointStates.position = [0, 0]

    jointStatesSimul = JointState()
    jointStatesSimul.name = ["pan_connect", "tilt_connect"]
    jointStatesSimul.position = [0, 0]
    
    ## Subscribers
    subPosition = rospy.Subscriber("/hardware/head/goal_pose", Float32MultiArray, callbackPosHead)
    subSimul = rospy.Subscriber("/simulated", Bool, callback_simulated)
    pubCurrentPose = rospy.Publisher("/hardware/head/current_pose", Float32MultiArray, queue_size = 1)
    pubJointStates = rospy.Publisher("/joint_states", JointState, queue_size = 1)
    pubBatery = rospy.Publisher("/hardware/robot_state/head_battery", Float32, queue_size = 1)
    msgCurrentPose = Float32MultiArray()
    msgCurrentPose.data = [0, 0]
    msgCurrentPosesimul = Float32MultiArray()
    msgCurrentPosesimul.data = [0, 0]
    
    loop = rospy.Rate(50)

    #lastPan = 0.0
    #lastTilt = 0.0
    goalPan = 0
    goalTilt = 0
    speedPan = 0.1 #These values should represent the Dynamixel's moving_speed 
    speedTilt = 0.1
    goalPansimul = 0
    goalTiltsimul= 0
    speedPansimul = 0.1 #These values should represent the Dynamixel's moving_speed 
    speedTiltsimul = 0.1

    while not rospy.is_shutdown():

        if simul == True:
            deltaPansimul = goalPansimul - pansimul
            deltaTiltsimul = goalTiltsimul - tiltsimul
            if deltaPansimul > speedPansimul:
                deltaPansimul = speedPansimul
            if deltaPansimul < -speedPansimul:
                deltaPansimul = -speedPansimul
            if deltaTiltsimul > speedTiltsimul:
                deltaTiltsimul = speedTiltsimul
            if deltaTiltsimul < -speedTiltsimul:
                deltaTiltsimul = -speedTiltsimul
            pansimul += deltaPansimul
            tiltsimul += deltaTiltsimul

            #print "HardwareHead.->pan " + str(pans)
            #print "HardwareHead.->tilt " + str(tilts)
        
            jointStatesSimul.header.stamp = rospy.Time.now()
            jointStatesSimul.position[0] = pansimul
            jointStatesSimul.position[1] = -tiltsimul

            pubJointStates.publish(jointStatesSimul)  #We substract 0.1 to correct an offset error due to the real head position
            msgCurrentPosesimul.data = [pansimul, tiltsimul]
            pubCurrentPose.publish(msgCurrentPosesimul)

            msgBattery = Float32()
            msgBattery.data = 12.0
            pubBatery.publish(msgBattery)

        else:
            deltaPan = goalPan - pan
            deltaTilt = goalTilt - tilt
            if deltaPan > speedPan:
                deltaPan = speedPan
            if deltaPan < -speedPan:
                deltaPan = -speedPan
            if deltaTilt > speedTilt:
                deltaTilt = speedTilt
            if deltaTilt < -speedTilt:
                deltaTilt = -speedTilt
            pan += deltaPan
            tilt += deltaTilt
            #print "HardwareHead.->pan " + str(pan)
            #print "HardwareHead.->tilt " + str(tilt)
        
            jointStates.header.stamp = rospy.Time.now()
            jointStates.position[0] = pan
            jointStates.position[1] = -tilt - 0.04#goes upwards, but to keep a dextereous system, positive tilt should go downwards
            pubJointStates.publish(jointStates)  #We substract 0.1 to correct an offset error due to the real head position
            msgCurrentPose.data = [pan, tilt]
            pubCurrentPose.publish(msgCurrentPose)

            if i == 10:
                msgBatery = float(dynMan1.GetPresentVoltage(0)/10.0)
                pubBatery.publish(msgBatery)
                i=0
            i+=1
             
        loop.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
