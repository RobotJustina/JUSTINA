#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from hardware_tools import Dynamixel
import tf

global armTorqueActive
global gripperTorqueActive
global torqueMode

gripperTorqueActive = False 
armTorqueActive = False
torqueMode = 2

def printRegisters(portName1, portBaud1):
    dynMan1 = Dynamixel.DynamixelMan(portName1, portBaud1)   
    dynMan1.GetRegistersValues(0)
    dynMan1.GetRegistersValues(1)
    dynMan1.GetRegistersValues(2)
    dynMan1.GetRegistersValues(3)
    dynMan1.GetRegistersValues(4)
    dynMan1.GetRegistersValues(5)
    dynMan1.GetRegistersValues(6)

def printHelp():
    print "LEFT ARM NODE BY MARCOSOfT. Options:"

def callbackTorqueGripper(msg):
    global dynMan1
    global torqueMode
    global torqueGripper
    presentLoad = 0
    
    torqueGripper = 0.0          ## Torque magnitude 
    torqueGripperCCW1 = True     ## Turn direction 
    torqueGripperCCW2 = False

    #Torque Mode = 0
    if torqueMode != 0:
        ### set torque mode...
        dynMan1.SetCWAngleLimit(7, 0)
        dynMan1.SetCCWAngleLimit(7, 0)

        dynMan1.SetCWAngleLimit(8, 0)
        dynMan1.SetCCWAngleLimit(8, 0)

        dynMan1.SetTorqueEnable(7, 1)
        dynMan1.SetTorqueEnable(8, 1) 
        torqueMode = 0
        print "Left gripper on torque mode... "


    if msg.data < 0:
        torqueGripper = int(-1*300*msg.data)
        torqueGripperCCW1 = False
        torqueGripperCCW2 = True
    else:
        torqueGripper = int(300*msg.data)
        torqueGripperCCW1 = True
        torqueGripperCCW2 = False

    #dynMan1.SetAlarmShutdown(7, 0b00000100)
    #dynMan1.SetAlarmShutdown(8, 0b00000100)

    dynMan1.SetTorqueVale(7, 100, torqueGripperCCW1)
    dynMan1.SetTorqueVale(8, 100, torqueGripperCCW2)

    #presentLoad = dynMan1.GetPresentLoad(7)
    #print "hardware-> left_arm gripper_presentLoad: " + str(presentLoad)
    #if presentLoad > 1023:
    #    presentLoad -= 1023

    #while presentLoad < 20:
    #    presentLoad = dynMan1.GetPresentLoad(7)
    #    if presentLoad > 1023:
    #        presentLoad -= 1023
    #    print "hardware->left_arm gripper_presentLoad: " + str(presentLoad)
    #    time.sleep(0.03)

    #dynMan1.SetTorqueVale(7, 0, torqueGripperCCW1)
    #dynMan1.SetTorqueVale(8, 0, torqueGripperCCW2)


def callbackGripper(msg):
    global dynMan1
    global gripperTorqueActive
    global torqueMode
    print "Gripper position mode, open...."

    #TorqueMode = 1 means position control  
    if torqueMode != 1:
        ### set position mode...
        dynMan1.SetCWAngleLimit(7, 0)
        dynMan1.SetCCWAngleLimit(7, 4095)
        dynMan1.SetCWAngleLimit(8, 0)
        dynMan1.SetCCWAngleLimit(8, 4095)
        dynMan1.SetMovingSpeed(7, 100)
        dynMan1.SetMovingSpeed(8, 100)
        dynMan1.SetTorqueEnable(7, 1)
        dynMan1.SetTorqueEnable(8, 1)
        torqueMode = 1
        print "Left gripper on position mode... "

    if gripperTorqueActive == False:
        dynMan1.SetTorqueEnable(7, 1)
        dynMan1.SetTorqueEnable(8, 1)

        dynMan1.SetMovingSpeed(7, 50)
        dynMan1.SetMovingSpeed(8, 50)
        gripperTorqueActive = True
        print "Left gripper active....  "

    gripperPos = msg.data
    gripperGoal_1 = int((  (gripperPos)/(360.0/4095.0*3.14159265358979323846/180.0) ) + 2487 )
    gripperGoal_2 = int(( -(gripperPos)/(360.0/4095.0*3.14159265358979323846/180.0) ) + 2741 )

    #print "gripper1: " + str(gripperGoal_1) + " gripper2: " +str(gripperGoal_2)
    dynMan1.SetGoalPosition(7, gripperGoal_1)
    dynMan1.SetGoalPosition(8, gripperGoal_2)


def callbackPos(msg):
    global dynMan1
    global armTorqueActive
    global goalPos
    global speedsGoal
    global newGoalPose
    global poseForFake
    global speedForFake

    Pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    speedsGoal = [0, 0, 0, 0, 0, 0, 0]
    goalPos = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    
    if armTorqueActive == False:
        ### Set Servomotors Torque Enable
        for i in range(len(Pos)):
            dynMan1.SetTorqueEnable(i, 1)
        ### Set Servomotors Speeds
        for i in range(len(Pos)):
            dynMan1.SetMovingSpeed(i, 60)
        armTorqueActive = True

    if len(msg.data) == 7: 
        ### Read the data of publisher
        for i in range(len(Pos)):
            Pos[i] = msg.data[i]
            speedsGoal[i] = 50
    elif len(msg.data) == 14:
        for i in range(len(Pos)):
            Pos[i] = msg.data[i]
            speedsGoal[i] = int(msg.data[i+7]*1023)
    for i in range(len(speedsGoal)):
        if speedsGoal[i] < 0:
            speedsGoal[i] = 0
        if speedsGoal[i] > 1023:
            speedsGoal[i] = 1023

    poseForFake = [Pos[0], Pos[1], Pos[2], Pos[3], Pos[4], Pos[5], Pos[6]]
    speedForFake = [0,0,0,0,0,0,0]
    for i in range(7):
        speedForFake[i] = speedsGoal[i]/1023.0*0.5
    # Conversion float to int for registers
    goalPos[0] = int((Pos[0]/(360.0/4095.0*3.14159265358979323846/180.0) ) + 2042 )
    goalPos[1] = int((Pos[1]/(360.0/4095.0*3.14159265358979323846/180.0) ) + 1603 )
    goalPos[2] = int((Pos[2]/(360.0/4095.0*3.14159265358979323846/180.0) ) + 1769 )
    goalPos[3] = int((Pos[3]/(360.0/4095.0*3.14159265358979323846/180.0) ) + 1983 )
    goalPos[4] = int((Pos[4]/(360.0/4095.0*3.14159265358979323846/180.0) ) + 2048 )
    goalPos[5] = int(-(Pos[5]/(360.0/4095.0*3.14159265358979323846/180.0) ) + 1795 )
    goalPos[6] = int((Pos[6]/(360.0/4095.0*3.14159265358979323846/180.0) ) + 3028 )
    newGoalPose = True

    #if len(msg.data) == 7: 
        ### Set GoalPosition
    #    for i in range(len(Pos)):
    #        dynMan1.SetMovingSpeed(i, 50)
    #        dynMan1.SetGoalPosition(i, goalPos[i])
    #elif len(msg.data) == 14:
    #    for i in range(len(Pos)):
    #        dynMan1.SetMovingSpeed(i, speedsGoal[i])
    #        dynMan1.SetGoalPosition(i, goalPos[i])


def main(portName1, portBaud1):
    print "INITIALIZING LEFT ARM NODE BY MARCOSOFT..."
    
    ###Communication with dynamixels:
    global dynMan1
    print "LEFT ARM.->Trying to open port " + portName1 + " at " + str(portBaud1)
    dynMan1 = Dynamixel.DynamixelMan(portName1, portBaud1)
    msgCurrentPose = Float32MultiArray()
    msgCurrentPose.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
    msgCurrentGripper = Float32()
    msgBatery = Float32()
    msgBatery = 0.0
    curretPos = [0,0,0,0,0,0,0,0]
    bitsPerRadian = (4095)/((360)*(3.141592/180)) 
    i = 0

    ### Set controller parameters 
    #dynMan1.SetCWComplianceSlope(0, 32)
    #dynMan1.SetCCWComplianceSlope(0, 32)
    #dynMan1.SetCWComplianceSlope(1, 32)
    #dynMan1.SetCCWComplianceSlope(1, 32)
    #for i in range(9):
    #    dynMan1.SetBaudrate(i, 1000000)

    for i in range(9):
        dynMan1.SetDGain(i, 25)
        dynMan1.SetPGain(i, 16)
        dynMan1.SetIGain(i, 6)

    ### Set servos features
    for i in range(9):
        dynMan1.SetMaxTorque(i, 1023)
        dynMan1.SetTorqueLimit(i, 768)
        dynMan1.SetHighestLimitTemperature(i, 80)
        dynMan1.SetAlarmShutdown(i, 0b00000100)

    dynMan1.SetCWAngleLimit(7, 0)
    dynMan1.SetCCWAngleLimit(7, 4095)
    dynMan1.SetCWAngleLimit(8, 0)
    dynMan1.SetCCWAngleLimit(8, 4095)
    dynMan1.SetMovingSpeed(7, 100)
    dynMan1.SetMovingSpeed(8, 100)
    dynMan1.SetGoalPosition(7, 2487)
    dynMan1.SetGoalPosition(8, 2741)
    
    ###Connection with ROS
    rospy.init_node("left_arm")
    br = tf.TransformBroadcaster()
    jointStates = JointState()
    jointStates.name = ["la_1_joint", "la_2_joint", "la_3_joint", "la_4_joint", "la_5_joint", "la_6_joint", "la_7_joint", "la_grip_left", "la_grip_right"]
    jointStates.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    subPos = rospy.Subscriber("/hardware/left_arm/goal_pose", Float32MultiArray, callbackPos)
    subGripper = rospy.Subscriber("/hardware/left_arm/goal_gripper", Float32, callbackGripper)
    subTorqueGripper = rospy.Subscriber("/hardware/left_arm/torque_gripper", Float32, callbackTorqueGripper)

    pubJointStates = rospy.Publisher("/joint_states", JointState, queue_size = 1)
    pubArmPose = rospy.Publisher("left_arm/current_pose", Float32MultiArray, queue_size = 1)
    pubGripper = rospy.Publisher("left_arm/current_gripper", Float32, queue_size = 1)
    pubBatery = rospy.Publisher("/hardware/robot_state/left_arm_battery", Float32, queue_size = 1)
    
    dynMan1.SetGoalPosition(0, 2042)
    dynMan1.SetGoalPosition(1, 1603)
    dynMan1.SetGoalPosition(2, 1769)
    dynMan1.SetGoalPosition(3, 1983)
    dynMan1.SetGoalPosition(4, 2048)
    dynMan1.SetGoalPosition(5, 1795)
    dynMan1.SetGoalPosition(6, 3028)
    for i in range(0, 8):
        dynMan1.SetTorqueEnable(i, 1)
    
    loop = rospy.Rate(5)
    bitValues = [0,0,0,0,0,0,0,0,0]
    lastValues = [0,0,0,0,0,0,0,0,0]

    global goalPos
    global speedsGoal
    global newGoalPose
    global torqueMode
    global torqueGripper
    goalPos = [0,0,0,0,0,0,0]
    speedsGoal=[0, 0,0,0,0,0,0]
    newGoalPose = False
    torqueGripper = 0
    gripperCounter = 0

    global poseForFake
    global speedForFake
    poseForFake = [0,0,0,0,0,0,0]
    speedForFake = [0,0,0,0,0,0,0]
    currentFakePose = [0,0,0,0,0,0,0]
    deltaFakePose = [0,0,0,0,0,0,0]

    while not rospy.is_shutdown():
        if newGoalPose:
            newGoalPose = False
            for i in range(7):
                dynMan1.SetTorqueLimit(i, 1023)
                dynMan1.SetTorqueEnable(i, True)
                dynMan1.SetMovingSpeed(i, speedsGoal[i])
                dynMan1.SetGoalPosition(i, goalPos[i])

        #bitValues[7]= dynMan1.GetPresentPosition(7)
        #bitValues[8]= dynMan1.GetPresentPosition(8)
        for i in range(7):
            deltaFakePose[i] = poseForFake[i] - currentFakePose[i]
            if deltaFakePose[i] > speedForFake[i]:
                deltaFakePose[i] = speedForFake[i];
            if deltaFakePose[i] < -speedForFake[i]:
                deltaFakePose[i] = -speedForFake[i]
            currentFakePose[i] += deltaFakePose[i]
        #for i in range(9):
        #    bitValues[i] = dynMan1.GetPresentPosition(i)
        #    if(bitValues[i] == 0):
        #        bitValues[i] = lastValues[i]
        #    else:
        #        lastValues[i] = bitValues[i]

        presentLoad= 500#= dynMan1.GetPresentLoad(7)
        if presentLoad > 1023:
            presentLoad -= 1023
        if  torqueMode == 0:
            #print "L_Current load: " + str(presentLoad) + " L_torqueGripper: " + str(torqueGripper)
            if presentLoad > torqueGripper:
                gripperCounter += 1
            else:
                gripperCounter = 0
            if gripperCounter > 10:
                gripperCounter = 0
                dynMan1.SetMovingSpeed(7, 0)
                dynMan1.SetMovingSpeed(8, 0)

        pos0 = currentFakePose[0]
        pos1 = currentFakePose[1]
        pos2 = currentFakePose[2]
        pos3 = currentFakePose[3]
        pos4 = currentFakePose[4]
        pos5 = currentFakePose[5]
        pos6 = currentFakePose[6]
        #pos0 = float( (1542-bitValues[0])/bitsPerRadian)
        #pos1 = float(-(2111-bitValues[1])/bitsPerRadian)
        #pos2 = float(-(1893-bitValues[2])/bitsPerRadian)
        #pos3 = float(-(2102-bitValues[3])/bitsPerRadian)
        #pos4 = float(-(2083-bitValues[4])/bitsPerRadian)
        #pos5 = float(-(2084-bitValues[5])/bitsPerRadian)
        #pos6 = float(-(1922-bitValues[6])/bitsPerRadian)
        posD21 = float((1200-bitValues[7])/bitsPerRadian)
        posD22 = float(-(395-bitValues[8])/bitsPerRadian)
        
        
        jointStates.header.stamp = rospy.Time.now()
        jointStates.position[0] = pos0
        jointStates.position[1] = pos1
        jointStates.position[2] = pos2
        jointStates.position[3] = pos3
        jointStates.position[4] = pos4
        jointStates.position[5] = pos5
        jointStates.position[6] = pos6
        jointStates.position[7] = posD21
        jointStates.position[8] = posD22
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
            msgBatery = float(dynMan1.GetPresentVoltage(2)/10.0)
            pubBatery.publish(msgBatery)
            i=0
        i+=1
        loop.sleep()

if __name__ == '__main__':
    try:
        portName1 = "/dev/ttyUSB0"
        portBaud1 = 115200
        if "--port1" in sys.argv:
            portName1 = sys.argv[sys.argv.index("--port1") + 1]
        if "--port2" in sys.argv:
            portName2 = sys.argv[sys.argv.index("--port2") + 1]
        if "--baud1" in sys.argv:
            portBaud1 = int(sys.argv[sys.argv.index("--baud1") + 1])
        if "--baud2" in sys.argv:
            portBaud2 = int(sys.argv[sys.argv.index("--baud2") + 1])
        if "--help" in sys.argv:
            printHelp()
        elif "-h" in sys.argv:
            printHelp()
        if "--registers" in sys.argv:
            printRegisters(portName1, portBaud1)
        else:
            main(portName1, portBaud1)
    except rospy.ROSInterruptException:
        pass
