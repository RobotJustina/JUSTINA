#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
#from hardware_tools import Dynamixel
from hardware_tools import dynamixel_lib as Dynamixel
import tf

global gripperTorqueActive
global armTorqueActive
global gripperTorqueLimit

zero_arm =[2056, 1600, 1800, 2100, 2000, 1800, 1050]
zero_gripper=[2440, 2680]

gripperTorqueActive = False
armTorqueActive = False
torqueGripper = 500
speedGripper = 200

def printRegisters(portName1, portBaud1):
    global dynMan1
    dynMan1 = Dynamixel.DynamixelMan(portName1, portBaud1)
    for i in range(9):
        dynMan1.GetRegistersValues(i)

def printHelp():
    print "JustinaHardwareLeftArm.->LEFT ARM NODE BY MARCOSOfT. Options:"


def callbackGripperTorque(msg):
    global dynMan1
    global torqueGripper
    global speedGripper
    global torqueGripperCCW1
    global torqueGripperCCW2
    global gripperTorqueActive
    global gripperTorqueLimit
    global attemps

    torqueGripperCCW1 = True     ## Turn direction
    torqueGripperCCW2 = False

    if msg.data > 1.0 :
        msg.data = 1;
    if msg.data < -1.0:
        msg.data = -1;

    if msg.data < 0:
        torqueGripper = int(-1*800*msg.data)
        torqueGripperCCW1 = False
        torqueGripperCCW2 = True
    else:
        torqueGripper = int(800*msg.data)
        torqueGripperCCW1 = True
        torqueGripperCCW2 = False

    speedGripper = 200

    ##### Flag to active gripper on torque mode ######
    gripperTorqueActive = True
    attemps = 0

    print "JustinaHardwareLeftArm.->Left gripper  - Open gripper"


def callbackGripperPos(msg):
    global dynMan1
    global gripperGoal_1
    global gripperGoal_2
    global gripperTorqueActive
    global torqueMode
    global attemps

    gripperPos = msg.data
    gripperGoal_1 = int((  (gripperPos)/(360.0/4095.0*3.14159265358979323846/180.0) ) + zero_gripper[0] )
    gripperGoal_2 = int(( -(gripperPos)/(360.0/4095.0*3.14159265358979323846/180.0) ) + zero_gripper[1] )

    ##### Flag to active gripper on position mode ######
    gripperTorqueActive = False
    attemps = 0

    print "JustinaHardwareLeftArm.->Left gripper  - Close gripper"


def callbackArmPos(msg):
    global dynMan1
    global armTorqueActive
    global goalPos
    global speedsGoal
    global newGoalPose

    Pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    goalPos = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    speedsGoal = [0, 0, 0, 0, 0, 0, 0]

    if armTorqueActive == False:
        ### Set Servomotors Torque Enable
        for i in range(len(Pos)):
            dynMan1.SetTorqueEnable(i, 1)
        ### Set Servomotors Speeds
        for i in range(len(Pos)):
            dynMan1.SetMovingSpeed(i, 40)
        armTorqueActive = True

    if len(msg.data) == 7:
        ### Read the data of publisher
        for i in range(len(Pos)):
            Pos[i] = msg.data[i]
            speedsGoal[i] = 40
    elif len(msg.data) == 14:
        for i in range(len(Pos)):
            Pos[i] = msg.data[i]
            speedsGoal[i] = int(msg.data[i+7]*1023)
        for i in range(len(speedsGoal)):
            if speedsGoal[i] < 0:
                speedsGoal[i] = 0
            if speedsGoal[i] > 1023:
                speedsGoal[i] = 1023


    # Conversion float to int for registers
    goalPos[0] = int((Pos[0]/(360.0/4095.0*3.14159265358979323846/180.0) )  + zero_arm[0] )
    goalPos[1] = int((Pos[1]/(360.0/4095.0*3.14159265358979323846/180.0) )  + zero_arm[1] )
    goalPos[2] = int((Pos[2]/(360.0/4095.0*3.14159265358979323846/180.0) )  + zero_arm[2] )
    goalPos[3] = int(-(Pos[3]/(360.0/4095.0*3.14159265358979323846/180.0) ) + zero_arm[3] )
    goalPos[4] = int((Pos[4]/(360.0/4095.0*3.14159265358979323846/180.0) )  + zero_arm[4] )
    goalPos[5] = int((Pos[5]/(360.0/4095.0*3.14159265358979323846/180.0) )  + zero_arm[5] )
    goalPos[6] = int((Pos[6]/(360.0/4095.0*3.14159265358979323846/180.0) )  + zero_arm[6] )

    ##### Flag to refresh arm_position ######
    newGoalPose = True


def main(portName1, portBaud1):
    print "JustinaHardwareLeftArm.->INITIALIZING LEFT ARM NODE BY MARCOSOFT..."

    global objOnHand
    global speedGripper
    global torqueGripper
    global gripperGoal_1
    global gripperGoal_2
    global torqueGripperCCW1
    global torqueGripperCCW2
    global gripperTorqueLimit
    global gripperTorqueActive

    global goalPos
    global torqueMode
    global speedsGoal
    global newGoalPose
    global armTorqueActive

    global attemps


    ###Communication with dynamixels:
    global dynMan1
    print "JustinaHardwareLeftArm.->Trying to open port " + portName1 + " at " + str(portBaud1)
    dynMan1 = Dynamixel.DynamixelMan(portName1, portBaud1)

    msgCurrentPose = Float32MultiArray()
    msgCurrentGripper = Float32()
    msgBatery = Float32()
    msgObjOnHand = Bool()

    msgCurrentPose.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    msgCurrentGripper.data = 0.0
    msgBatery = 0.0
    msgObjOnHand = False

    curretPos = [0,0,0,0,0,0,0,0]
    bitsPerRadian = (4095)/((360)*(3.141592/180))

    bitValues = [0,0,0,0,0,0,0,0,0,0,0]
    lastValues = [0,0,0,0,0,0,0,0,0,0,0]

    goalPos = [0,0,0,0,0,0,0]
    speedsGoal=[0, 0,0,0,0,0,0]

    newGoalPose = False
    objOnHand = False
    torqueGripper = 0
    currentLoad_D21 = 0
    currentLoad_D22 = 0
    posD21 = 0.0
    posD22 = 0.0
    gripperGoal_1 = zero_gripper[0]
    gripperGoal_2 = zero_gripper[1]

    attemps = 0
    i = 0


    ##############################
    ###  Connection with ROS  ####

    rospy.init_node("left_arm")
    br = tf.TransformBroadcaster()
    jointStates = JointState()
    jointStates.name = ["la_1_joint", "la_2_joint", "la_3_joint", "la_4_joint", "la_5_joint", "la_6_joint", "la_7_joint", "la_grip_left", "la_grip_right"]
    jointStates.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    subPos = rospy.Subscriber("/hardware/left_arm/goal_pose", Float32MultiArray, callbackArmPos)
    subGripper = rospy.Subscriber("/hardware/left_arm/goal_gripper", Float32, callbackGripperPos)
    subTorqueGripper = rospy.Subscriber("/hardware/left_arm/torque_gripper", Float32, callbackGripperTorque)

    pubJointStates = rospy.Publisher("/joint_states", JointState, queue_size = 1)
    pubArmPose = rospy.Publisher("left_arm/current_pose", Float32MultiArray, queue_size = 1)
    pubGripper = rospy.Publisher("left_arm/current_gripper", Float32, queue_size = 1)
    pubObjOnHand = rospy.Publisher("left_arm/object_on_hand", Bool, queue_size = 1)
    pubBatery = rospy.Publisher("/hardware/robot_state/left_arm_battery", Float32, queue_size = 1)


    #####################
    ##    Dynamixel   ##
    for i in range(9):
        #dynMan1.SetDGain(i, 25)
        #dynMan1.SetPGain(i, 16)
        #dynMan1.SetIGain(i, 6)
        dynMan1.SetDGain(i, 0)
        dynMan1.SetPGain(i, 32)
        dynMan1.SetIGain(i, 0)

    ### Set servos features
    for i in range(9):
        dynMan1.SetMaxTorque(i, 1023)
        dynMan1.SetTorqueLimit(i, 768)
        dynMan1.SetHighestLimitTemperature(i, 80)
        dynMan1.SetAlarmShutdown(i, 0b00000100)

    #Dynamixel gripper on position Mode...
    dynMan1.SetCWAngleLimit(7, 0)
    dynMan1.SetCCWAngleLimit(7, 4095)
    dynMan1.SetCWAngleLimit(8, 0)
    dynMan1.SetCCWAngleLimit(8, 4095)

    dynMan1.SetMovingSpeed(7, 100)
    dynMan1.SetMovingSpeed(8, 100)

    # Set torque_active for each servo
    for i in range(7):
        dynMan1.SetTorqueEnable(i, 1)

    dynMan1.SetTorqueEnable(7, 1)
    dynMan1.SetTorqueEnable(8, 1)

    # Set initial pos for each servo
    #for i in range(6):
    #    dynMan1.SetGoalPosition(i, zero_arm[i])

    #dynMan1.SetGoalPosition(6, 2130)
    #dynMan1.SetGoalPosition(7, zero_gripper[0])
    #dynMan1.SetGoalPosition(8, zero_gripper[1])

    loop = rospy.Rate(30)


    while not rospy.is_shutdown():

        #### Refresh arm_position ####
        if newGoalPose:
            newGoalPose = False
            for i in range(7):
                dynMan1.SetTorqueLimit(i, 768)
                dynMan1.SetTorqueEnable(i, True)
                dynMan1.SetMovingSpeed(i, speedsGoal[i])
                dynMan1.SetGoalPosition(i, goalPos[i])
                rospy.sleep(0.05)

        #### Refresh gripper_pos ####
        if attemps < 50:
            if gripperTorqueActive :
                dynMan1.SetCWAngleLimit(7, 0)
                dynMan1.SetCCWAngleLimit(7, 0)
                dynMan1.SetCWAngleLimit(8, 0)
                dynMan1.SetCCWAngleLimit(8, 0)
                dynMan1.SetTorqueLimit(7, torqueGripper)
                dynMan1.SetTorqueLimit(8, torqueGripper)
                dynMan1.SetTorqueVale(7, speedGripper, torqueGripperCCW1)
                dynMan1.SetTorqueVale(8, speedGripper, torqueGripperCCW2)
                currentLoad_D21 = dynMan1.GetPresentLoad(7)
                currentLoad_D22 = dynMan1.GetPresentLoad(8)
            else:
                dynMan1.SetCWAngleLimit(7, 0)
                dynMan1.SetCCWAngleLimit(7, 4095)
                dynMan1.SetCWAngleLimit(8, 0)
                dynMan1.SetCCWAngleLimit(8, 4095)
                dynMan1.SetTorqueLimit(7, 500)
                dynMan1.SetTorqueLimit(8, 500)
                dynMan1.SetMovingSpeed(7, 200)
                dynMan1.SetMovingSpeed(8, 200)
                dynMan1.SetGoalPosition(7, gripperGoal_1)
                dynMan1.SetGoalPosition(8, gripperGoal_2)
                objOnHand = False
            ## This counter is reseated in the callback
            attemps += 1

        #### Refresh arms_position's readings #####
        for i in range(9):
            bitValues[i] = dynMan1.GetPresentPosition(i)
            if(bitValues[i] == 0):
                bitValues[i] = lastValues[i]
            else:
                lastValues[i] = bitValues[i]

        # CurrentLoad > 1023 means the oposite sense of load
        if currentLoad_D21 > 1023:
            currentLoad_D21 -= 1023

        if currentLoad_D22 > 1023:
            currentLoad_D22 -= 1023

        if gripperTorqueActive:
            currentLoad_D21 = (currentLoad_D21 + currentLoad_D22)/2
            if currentLoad_D21 > 200 and posD21 > -0.05 :
                objOnHand = True
            else:
                objOnHand = False
        #print "Load_left_gripper:  " + str(currentLoad_D21)
        #print "pos_left_gripper:  " + str(posD21)

        pos0 = float(-(zero_arm[0]-bitValues[0])/bitsPerRadian)
        pos1 = float(-(zero_arm[1]-bitValues[1])/bitsPerRadian)
        pos2 = float(-(zero_arm[2]-bitValues[2])/bitsPerRadian)
        pos3 = float( (zero_arm[3]-bitValues[3])/bitsPerRadian)
        pos4 = float(-(zero_arm[4]-bitValues[4])/bitsPerRadian)
        pos5 = float(-(zero_arm[5]-bitValues[5])/bitsPerRadian)
        pos6 = float(-(zero_arm[6]-bitValues[6])/bitsPerRadian)
        posD21 = float(-(zero_gripper[0]-bitValues[7])/bitsPerRadian)
        posD22 = float( (zero_gripper[1]-bitValues[8])/bitsPerRadian)

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
        msgObjOnHand = objOnHand
        pubJointStates.publish(jointStates)
        pubArmPose.publish(msgCurrentPose)
        pubGripper.publish(msgCurrentGripper)
        pubObjOnHand.publish(msgObjOnHand)

        if i == 20:
            msgBatery = float(dynMan1.GetPresentVoltage(2)/10.0)
            pubBatery.publish(msgBatery)
            i=0
        i+=1
        loop.sleep()

    dynMan1.SetTorqueDisable(0)
    dynMan1.SetTorqueDisable(1)
    dynMan1.SetTorqueDisable(2)
    dynMan1.SetTorqueDisable(3)
    dynMan1.SetTorqueDisable(4)
    dynMan1.SetTorqueDisable(5)
    dynMan1.SetTorqueDisable(6)
    dynMan1.Close()

if __name__ == '__main__':
    try:
        portName1 = "/dev/justinaLeftArm"
        portBaud1 = 1000000
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
