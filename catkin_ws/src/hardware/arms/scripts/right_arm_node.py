#!/usr/bin/env python
import sys
import rospy
import Dynamixel
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import tf

def printHelp():
    print "RIGHT ARM NODE BY MARCOSOfT. Options:"

def callbackPos(msg):
    global dynMan1

    Pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    goalPos = [0, 0, 0, 0, 0, 0, 0, 0, 0]

    ### Read the data of publisher
    for i in range(len(pos)):
        Pos[i] = msg.data[i]

    # Pos0 = msg.data[0]
    # Pos1 = msg.data[1]
    # Pos2 = msg.data[2]
    # Pos3 = msg.data[3]
    # Pos4 = msg.data[4]
    # Pos5 = msg.data[5]
    # Pos6 = msg.data[6] 
    # Pos7 = msg.data[7]
    # Pos8 = msg.data[8]

    # Conversion float to int for registers
    goalPos[0] = int((  (Pos[0])/(360.0/4095.0*3.14159265358979323846/180.0) ) + 1530 )
    goalPos[1] = int((  (Pos[1])/(360.0/4095.0*3.14159265358979323846/180.0) ) + 2107 )
    goalPos[2] = int((  (Pos[2])/(360.0/4095.0*3.14159265358979323846/180.0) ) + 2048 )
    goalPos[3] = int((  (Pos[3])/(360.0/4095.0*3.14159265358979323846/180.0) ) + 2102 )
    goalPos[4] = int((  (Pos[4])/(360.0/4095.0*3.14159265358979323846/180.0) ) + 2048 )
    goalPos[5] = int((  (Pos[5])/(360.0/4095.0*3.14159265358979323846/180.0) ) + 2068 )
    goalPos[6] = int((  (Pos[6])/(360.0/4095.0*3.14159265358979323846/180.0) ) + 1924 )
    goalPos[7] = int((  (Pos[7])/(360.0/4095.0*3.14159265358979323846/180.0) ) + 1400 )
    goalPos[8] = int((  (Pos[8])/(360.0/4095.0*3.14159265358979323846/180.0) ) + 1295 )


    ### Set Servomotors Torque Enable
    for i in range(len(pos)):
        dynMan1.SetTorqueEnable(i, 1)
    # dynMan1.SetTorqueEnable(0, 1)
    # dynMan1.SetTorqueEnable(1, 1)
    # dynMan1.SetTorqueEnable(2, 1)
    # dynMan1.SetTorqueEnable(3, 1)
    # dynMan1.SetTorqueEnable(4, 1)
    # dynMan1.SetTorqueEnable(5, 1)
    # dynMan1.SetTorqueEnable(6, 1)
    # dynMan2.SetTorqueEnable(7, 1)
    # dynMan2.SetTorqueEnable(8, 1)

    ### Set Servomotors Speeds
    for i in range(len(pos)):
        dynMan1.SetMovingSpeed(i, 100)
    # dynMan1.SetMovingSpeed(0, 100)
    # dynMan1.SetMovingSpeed(1, 100)
    # dynMan1.SetMovingSpeed(2, 100)
    # dynMan1.SetMovingSpeed(3, 100)
    # dynMan1.SetMovingSpeed(4, 100)
    # dynMan1.SetMovingSpeed(5, 100)
    # dynMan1.SetMovingSpeed(6, 100)
    # dynMan2.SetMovingSpeed(7, 100)
    # dynMan2.SetMovingSpeed(107, 100)

    ### Set GoalPosition
    for i in range(len(pos)):
        dynMan1.SetGoalPosition(i, goalPos[i])
    # dynMan1.SetGoalPosition(0, goalPos0)
    # dynMan1.SetGoalPosition(1, goalPos1)
    # dynMan1.SetGoalPosition(2, goalPos2)
    # dynMan1.SetGoalPosition(3, goalPos3)
    # dynMan1.SetGoalPosition(4, goalPos4)
    # dynMan1.SetGoalPosition(5, goalPos5)
    # dynMan1.SetGoalPosition(6, goalPos6)
    # dynMan2.SetGoalPosition(7, goalPos5)
    # dynMan2.SetGoalPosition(107, goalPos6)
    
def main(portName1, portBaud1, portName2, portBaud2):
    print "INITIALIZING RIGHT ARM NODE BY MARCOSOFT..."
    ###Connection with ROS
    rospy.init_node("right_arm")
    br = tf.TransformBroadcaster()
    jointStates = JointState()
    jointStates.name = ["la_1_joint", "la_2_joint", "la_3_joint", "la_4_joint", "la_5_joint", "la_6_joint", "la_7_joint"]
    jointStates.position = [0, 0, 0, 0, 0, 0, 0]

    subPos = rospy.Subscriber("right_arm/goal_pose", Float32MultiArray, callbackPos)
    pubJointStates = rospy.Publisher("/joint_states", JointState, queue_size = 1)
    pubArmPose = rospy.Publisher("right_arm/current_pose", Float32MultiArray, queue_size = 1)
    pubGripper = rospy.Publisher("right_arm/current_gripper", Float32, queue_size = 1)
    
    
    loop = rospy.Rate(10)
    ###Communication with dynamixels:
    global dynMan1 
    dynMan1 = Dynamixel.DynamixelMan(portName1, portBaud1)

    msgCurrentPose = Float32MultiArray()
    msgCurrentPose.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
    msgCurrentGripper = Float32()
    bitsPerRadian = (4095)/((360)*(3.141592/180)) 

    for i in range(7):
        dynMan1.Ping(i)

    while not rospy.is_shutdown():
        #print str(dynMan1.GetMovingSpeed(0))
        pos0 = 0#float(-(1530-dynMan1.GetPresentPosition(1))/bitsPerRadian)
        pos1 = 0#float(-(2107-dynMan1.GetPresentPosition(1))/bitsPerRadian)
        pos2 = 0#float(-(2048-dynMan1.GetPresentPosition(2))/bitsPerRadian)
        pos3 = 0#float(-(2102-dynMan1.GetPresentPosition(3))/bitsPerRadian)
        pos4 = 0#float(-(2048-dynMan1.GetPresentPosition(4))/bitsPerRadian)
        pos5 = 0#float( (2068-dynMan1.GetPresentPosition(5))/bitsPerRadian)
        pos6 = 0#float(-(1924-dynMan1.GetPresentPosition(6))/bitsPerRadian)
        posD21 = 0#float((1400-dynMan1.GetPresentPosition(7))/bitsPerRadian)
        posD22 = 0#float((1295-dynMan1.GetPresentPosition(8))/bitsPerRadian)
        
        #print "Poses: " + str(pos0) + "  " + str(pos1) + "  " + str(pos2) + "  " + str(pos3) + "  " + str(pos4) + "  " + str(pos5) + "  " + str(pos6) + "  " + str(posD21) + "  " + str(posD22)
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
        msgCurrentGripper.data = posD22
        pubJointStates.publish(jointStates)
        pubArmPose.publish(msgCurrentPose)
        pubGripper.publish(msgCurrentGripper)
        loop.sleep()

if __name__ == '__main__':
    try:
        if "--help" in sys.argv:
            printHelp()
        elif "-h" in sys.argv:
            printHelp()
        else:
            portName1 = "/dev/ttyUSB0"
            portName2 = "/dev/ttyUSB1"
            portBaud1 = 115200
            portBaud2 = 115200
            if "--port1" in sys.argv:
                portName1 = sys.argv[sys.argv.index("--port1") + 1]
            if "--port2" in sys.argv:
                portName2 = sys.argv[sys.argv.index("--port2") + 1]
            if "--baud1" in sys.argv:
                portBaud1 = int(sys.argv[sys.argv.index("--baud1") + 1])
            if "--baud2" in sys.argv:
                portBaud2 = int(sys.argv[sys.argv.index("--baud2") + 1])
            main(portName1, portBaud1, portName2, portBaud2)
    except rospy.ROSInterruptException:
        pass
