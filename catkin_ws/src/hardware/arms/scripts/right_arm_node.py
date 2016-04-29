#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from hardware_tools import Dynamixel
import tf


global gripperTorqueActive
global armTorqueActive

gripperTorqueActive = False
armTorqueActive = False


def printHelp():
    print "RIGHT ARM NODE BY MARCOSOfT. Options:"

def printRegisters(portName1, portBaud1):
    dynMan1 = Dynamixel.DynamixelMan(portName1, portBaud1)
    dynMan1.GetRegistersValues(0)
    dynMan1.GetRegistersValues(1)
    dynMan1.GetRegistersValues(2)
    dynMan1.GetRegistersValues(3)
    dynMan1.GetRegistersValues(4)
    dynMan1.GetRegistersValues(5)
    dynMan1.GetRegistersValues(6)

def callbackGripper(msg):
    global dynMan1
    global gripperTorqueActive

    if gripperTorqueActive == False:
        dynMan1.SetTorqueEnable(7, 1)
        dynMan1.SetTorqueEnable(8, 1)

        dynMan1.SetMovingSpeed(7, 50)
        dynMan1.SetMovingSpeed(8, 50)
        gripperTorqueActive = True

    gripperPos = msg.data
    gripperGoal_1 = int((  (gripperPos)/(360.0/4095.0*3.14159265358979323846/180.0) ) + 1400 )
    gripperGoal_2 = int((  (gripperPos)/(360.0/4095.0*3.14159265358979323846/180.0) ) + 1295 )

    dynMan1.SetGoalPosition(7, gripperGoal_1)
    dynMan1.SetGoalPosition(8, gripperGoal_2)


def callbackPos(msg):
    global dynMan1
    global armTorqueActive

    Pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    goalPos = [0, 0, 0, 0, 0, 0, 0, 0, 0]

    if armTorqueActive == False:
        ### Set Servomotors Torque Enable
        for i in range(len(Pos)):
            dynMan1.SetTorqueEnable(i, 1)

        ### Set Servomotors Speeds
        for i in range(len(Pos)):
            dynMan1.SetMovingSpeed(i, 60)

        armTorqueActive = True


    ### Read the data of publisher
    for i in range(len(Pos)):
        Pos[i] = msg.data[i]

    # Conversion float to int for registers
    goalPos[0] = int((-(Pos[0])/(360.0/4095.0*3.14159265358979323846/180.0) ) + 1530 )
    goalPos[1] = int(( (Pos[1])/(360.0/4095.0*3.14159265358979323846/180.0) ) + 2107 )
    goalPos[2] = int((-(Pos[2])/(360.0/4095.0*3.14159265358979323846/180.0) ) + 2048 )
    goalPos[3] = int(( (Pos[3])/(360.0/4095.0*3.14159265358979323846/180.0) ) + 2102 )
    goalPos[4] = int((-(Pos[4])/(360.0/4095.0*3.14159265358979323846/180.0) ) + 2048 )
    goalPos[5] = int(( (Pos[5])/(360.0/4095.0*3.14159265358979323846/180.0) ) + 2068 )
    goalPos[6] = int((-(Pos[6])/(360.0/4095.0*3.14159265358979323846/180.0) ) + 1924 )



    ### Set GoalPosition
    for i in range(len(Pos)):
        dynMan1.SetGoalPosition(i, goalPos[i])

    
def main(portName1, portBaud1):
    print "INITIALIZING RIGHT ARM NODE BY MARCOSOFT..."
    
    ###Communication with dynamixels:
    global dynMan1 
    dynMan1 = Dynamixel.DynamixelMan(portName1, portBaud1)
    msgCurrentPose = Float32MultiArray()
    msgCurrentPose.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
    msgCurrentGripper = Float32()
    msgBatery = Float32()
    msgBatery = 0.0
    curretPos = [0,0,0,0,0,0,0,0]
    bitsPerRadian = (4095)/((360)*(3.141592/180)) 
    i = 0

    ###Connection with ROS
    rospy.init_node("right_arm")
    br = tf.TransformBroadcaster()
    jointStates = JointState()
    jointStates.name = ["ra_1_joint", "ra_2_joint", "ra_3_joint", "ra_4_joint", "ra_5_joint", "ra_6_joint", "ra_7_joint"]
    jointStates.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    subPos = rospy.Subscriber("/hardware/right_arm/goal_pose", Float32MultiArray, callbackPos)
    pubJointStates = rospy.Publisher("/joint_states", JointState, queue_size = 1)
    pubArmPose = rospy.Publisher("right_arm/current_pose", Float32MultiArray, queue_size = 1)
    pubGripper = rospy.Publisher("right_arm/current_gripper", Float32, queue_size = 1)
    pubBatery = rospy.Publisher("/hardware/robot_state/right_arm_battery", Float32, queue_size = 1)
    

    ### Set controller parameters
    for i in range(0, 7):
        dynMan1.SetDGain(i, 25)
        dynMan1.SetPGain(i, 16)
        dynMan1.SetIGain(i, 1)


    ### Set servos features
    for i in range(0, 7):
        dynMan1.SetMaxTorque(i, 1024)
        dynMan1.SetTorqueLimit(i, 512)
        dynMan1.SetHighestLimitTemperature(i, 80)

    

    loop = rospy.Rate(10)

    while not rospy.is_shutdown():

        pos0 = float( (1530-dynMan1.GetPresentPosition(0))/bitsPerRadian)
        pos1 = float(-(2107-dynMan1.GetPresentPosition(1))/bitsPerRadian)
        pos2 = float(-(2048-dynMan1.GetPresentPosition(2))/bitsPerRadian)
        pos3 = float(-(2102-dynMan1.GetPresentPosition(3))/bitsPerRadian)
        pos4 = float(-(2048-dynMan1.GetPresentPosition(4))/bitsPerRadian)
        pos5 = float(-(2068-dynMan1.GetPresentPosition(5))/bitsPerRadian)
        pos6 = float(-(1924-dynMan1.GetPresentPosition(6))/bitsPerRadian)
        posD21 = float((1400-dynMan1.GetPresentPosition(7))/bitsPerRadian)
        posD22 = float((1295-dynMan1.GetPresentPosition(8))/bitsPerRadian)
        
        jointStates.header.stamp = rospy.Time.now()
        jointStates.position[0] = pos0
        jointStates.position[1] = pos1
        jointStates.position[2] = pos2
        jointStates.position[3] = pos3
        jointStates.position[4] = pos4
        jointStates.position[5] = pos5
        jointStates.position[6] = pos6
        msgCurrentPose.data[0] = pos0
        msgCurrentPose.data[1] = pos1
        msgCurrentPose.data[2] = pos2
        msgCurrentPose.data[3] = pos3
        msgCurrentPose.data[4] = pos4
        msgCurrentPose.data[5] = pos5
        msgCurrentPose.data[6] = pos6
        msgCurrentGripper.data = posD21
        pubJointStates.publish(jointStates)
        pubArmPose.publish(msgCurrentPose)
        pubGripper.publish(msgCurrentGripper)

        if i == 20:
            msgBatery = float(dynMan1.GetPresentVoltage(0)/10)
            print "Batery voljate: " + str(msgBatery)
            pubBatery.publish(msgBatery)
            i=0
        i+=1
        loop.sleep()

if __name__ == '__main__':
    try:
        if "--help" in sys.argv:
            printHelp()
        elif "-h" in sys.argv:
            printHelp()
        if "--registers" in sys.argv:
            printRegisters("/dev/ttyUSB1", 57600)
        else:
            portName1 = "/dev/ttyUSB1"
            portBaud1 = 57600
            if "--port1" in sys.argv:
                portName1 = sys.argv[sys.argv.index("--port1") + 1]
            if "--port2" in sys.argv:
                portName2 = sys.argv[sys.argv.index("--port2") + 1]
            if "--baud1" in sys.argv:
                portBaud1 = int(sys.argv[sys.argv.index("--baud1") + 1])
            if "--baud2" in sys.argv:
                portBaud2 = int(sys.argv[sys.argv.index("--baud2") + 1])
            main(portName1, portBaud1)
    except rospy.ROSInterruptException:
        pass
