#!/usr/bin/env python
import sys
import rospy
import Dynamixel
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
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
        print "Mode Torque...   "
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

    print "Torque.... " + str(torquePan) + "   " + str(torqueTilt)

    ## Send 0-1023 magnitude torque, and the torquePanCCW means the turn direction 
    dynMan1.SetTorqueVale(5, torquePan, torquePanCCW)
    dynMan1.SetTorqueVale(1, torqueTilt, torqueTiltCCW)





def callbackPosHead(msg):
    global dynMan1
    global modeTorque
    if modeTorque != 1:
        ## Change to Position mode
        dynMan1.SetCWAngleLimit(5, 0)
        dynMan1.SetCCWAngleLimit(5, 1023)

        dynMan1.SetCWAngleLimit(1, 0)
        dynMan1.SetCCWAngleLimit(1, 1023)
        
        dynMan1.SetTorqueEnable(5, 1)
        dynMan1.SetTorqueEnable(1, 1)

        dynMan1.SetMovingSpeed(5, 50)
        dynMan1.SetMovingSpeed(1, 50)
        
        print "Mode Position...   "
        modeTorque = 1

    ### Set GoalPosition 
    goalPosPan = msg.data[0]
    goalPosTilt = msg.data[1]

    # Conversion float to bits
    goalPosTilt = int(( (goalPosTilt)/(300.0/1023.0*3.14159265358979323846/180.0) ) + 674)
    goalPosPan = int((  (goalPosPan)/(300.0/1023.0*3.14159265358979323846/180.0) ) + 512 )

    if goalPosTilt >= 0 and goalPosTilt <= 1023 and goalPosPan >= 0 and goalPosPan <=1023:
        dynMan1.SetGoalPosition(5, goalPosPan)
        dynMan1.SetGoalPosition(1, goalPosTilt)
    else:
        print " Error: Incorrect goal position.... "


def printHelp():
    print "HEAD NODE. Options:"
    print "TODO: Print all argument options"



def main(portName, portBaud):
    global dynMan1
    ###Communication with dynamixels:
    dynMan1 = Dynamixel.DynamixelMan(portName, portBaud)
    dynMan1.SetHighestLimitTemperature(5, 80)
    dynMan1.SetHighestLimitTemperature(1, 80)
    
    # dynMan1.GetRegistersValues(5)
    # dynMan1.GetRegistersValues(1)

    pan = 0;
    tilt = 0;

    print "INITIALIZING HEAD NODE..."
    ###Connection with ROS
    rospy.init_node("head")
    br = tf.TransformBroadcaster()
    jointStates = JointState()
    jointStates.name = ["pan_connect", "tilt_connect"]
    jointStates.position = [0, 0]
    
    ## Subscribers
    subPosition = rospy.Subscriber("/goal_pose", Float32MultiArray, callbackPosHead)
    subTorque = rospy.Subscriber("/torque", Float32MultiArray, callbackTorque)

    ## Publishers
    pubJointStates = rospy.Publisher("/joint_states", JointState, queue_size = 1)
    
    loop = rospy.Rate(10)
    bitsPerRadian = (1023)/((300)*(3.14159265358979323846/180))

    while not rospy.is_shutdown():
        panPose = float((512-dynMan1.GetPresentPosition(5))/bitsPerRadian)
        tiltPose = float((674-dynMan1.GetPresentPosition(1))/bitsPerRadian)
        
        print "Poses: " + str(panPose) + "   " + str(tiltPose)

        # Pose in bits
        panPose = dynMan1.GetPresentPosition(5)
        tiltPose = dynMan1.GetPresentPosition(1)
        
        # Pose in rad
        pan = (panPose - 512)*300/1023*3.14159265358979323846/180
        tilt = (tiltPose - 674)*300/1023*3.14159265358979323846/180
        
        jointStates.header.stamp = rospy.Time.now()
        jointStates.position[0] = pan
        jointStates.position[1] = -tilt #A tilt > 0 goes upwards, but to keep a dextereous system, positive tilt should go downwards
        pubJointStates.publish(jointStates)
        #print "Poses: " + str(panPose) + "   " + str(tiltPose)
        loop.sleep()

if __name__ == '__main__':
    try:
        if "--help" in sys.argv:
            printHelp()
        elif "-h" in sys.argv:
            printHelp()
        else:
            portName = "/dev/ttyUSB2"
            portBaud = 1000000
            if "--port" in sys.argv:
                portName = sys.argv[sys.argv.index("--port") + 1]
            if "--baud" in sys.argv:
                portBaud = int(sys.argv[sys.argv.index("--baud") + 1])
            main(portName, portBaud)
    except rospy.ROSInterruptException:
        pass
