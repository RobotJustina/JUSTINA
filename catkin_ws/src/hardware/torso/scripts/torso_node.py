#!/usr/bin/env python
import sys
import rospy
import math
import time
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import tf
import torso_driver as Torso
torso = None

def printHelp():
    print "YOU DON'T REALLY NEED HELP TO OPERATE TORSO IN SIMULATION MODE"

def callbackGoalPose(msg):
    if len(msg.data) != 3:
        print "Torso.->Invalid number of goal values. "
        return
    global goalSpine
    global goalWaist
    global goalShoulders
    global newGoal
    goalSpine = msg.data[0]
    goalWaist = msg.data[1]
    goalShoulders = msg.data[2]
    newGoal = True;
    msgGoalReached = Bool()
    msgGoalReached.data = False
    pubGoalReached.publish(msgGoalReached)


    dato = msg.data[0]*100
    if dato>45:
        dato=45
    if dato<0:
        dato=0
    torso.columna[0] = int(dato)

    dato = msg.data[1]*180.0/3.1416
    if dato>90:
        dato=90
    if dato<-90:
        dato=-90
    torso.torso[0] = int(dato)

    dato = msg.data[2]*180.0/3.1416
    if dato>20:
        dato=20
    if dato<-20:
        dato=-20
    torso.hombro[0] = int(dato)




def callbackRelPose(msg):
    if len(msg.data) != 3:
        print "Torso.->Invalid number of goal values. "
        return
    global spine
    global waist
    global shoulders
    global goalSpine
    global goalWaist
    global goalShoulders
    global newGoal
    goalSpine = spine + msg.data[0]
    goalWaist = waist + msg.data[1]
    goalShoulders = shoulders + msg.data[2]
    newGoal = True
    msgGoalReached = Bool()
    msgGoalReached.data = False
    pubGoalReached.publish(msgGoalReached)

    dato = goalSpine*100
    if dato>45:
        dato=45
    if dato<0:
        dato=0
    torso.columna[0] = int(dato)

    dato = goalWaist*180.0/3.1416
    if dato>90:
        dato=90
    if dato<-90:
        dato=-90
    torso.torso[0] = int(dato)

    dato = goalShoulders*180.0/3.1416
    if dato>20:
        dato=20
    if dato<-20:
        dato=-20
    torso.hombro[0] = int(dato)
        

def main(portName):
    print "INITIALIZING TORSO NODE BY MARCOSOFT..."
    
    #init torso
    global torso
    torso=Torso.Torso(False)
    torso.SetSerial(portName,115200)
    
    #torso.start()

    ###Connection with ROS
    global pubGoalReached
    rospy.init_node("torso")
    br = tf.TransformBroadcaster()	
    jointStates = JointState()
    jointStates.name = ["spine_connect","waist_connect","shoulders_connect", "shoulders_left_connect", "shoulders_right_connect"]
    jointStates.position = [0.0, 0.0, 0.0, 0.0, 0.0]
    
    rospy.Subscriber("/hardware/torso/goal_pose", Float32MultiArray, callbackGoalPose)
    rospy.Subscriber("/hardware/torso/goal_rel_pose", Float32MultiArray, callbackRelPose)
    pubJointStates = rospy.Publisher("/joint_states", JointState, queue_size = 1)
    pubCurrentPose = rospy.Publisher("/hardware/torso/current_pose", Float32MultiArray, queue_size=1)
    pubGoalReached = rospy.Publisher("/hardware/torso/goal_reached", Bool, queue_size=1)

    loop = rospy.Rate(30)

    global goalSpine
    global goalWaist
    global goalShoulders
    global spine
    global waist
    global shoulders
    global newGoal
    goalSpine = 0.0
    goalWaist = 0.0
    goalShoulders = 0.0
    spine = 0
    waist = 0
    shoulders = 0
    speedSpine = 0.005
    speedWaist = 0.1
    speedShoulders = 0.1
    msgCurrentPose = Float32MultiArray()
    msgGoalReached = Bool()
    msgCurrentPose.data = [0, 0, 0]
    newGoal = False
    
    torso.ComBase()#linea agregada para comunicar con la base
    
    time.sleep(1)
    torso.columna[0]=torso.columna[1]
    torso.columna[2]=True
    torso.torso[2]=False
    torso.hombro[2]=False

    while not rospy.is_shutdown():

        torso.ComBase()#linea agregada para comunicar con la base


        spine=torso.columna[1]/100.0 - 0.2
        waist=0#torso.torso[1]*3.1416/180.0
        shoulders=0#torso.hombro[1]*3.1416/180.0




        jointStates.header.stamp = rospy.Time.now()
        jointStates.position = [spine, waist, shoulders, -shoulders, -shoulders]
        pubJointStates.publish(jointStates)

        msgCurrentPose.data[0] = spine
        msgCurrentPose.data[1] = waist
        msgCurrentPose.data[2] = shoulders
        pubCurrentPose.publish(msgCurrentPose)

        if newGoal and abs(goalSpine - spine) < 0.015 and abs(goalWaist - waist) < 0.05 and abs(goalShoulders - shoulders) < 0.05:
            newGoal = False
            msgGoalReached.data = True
            pubGoalReached.publish(msgGoalReached)
        
        loop.sleep()
    
    torso.columna[2]=False
    torso.torso[2]=False
    torso.hombro[2]=False
    torso.runBase = False

if __name__ == '__main__':
    try:
        portName = "/dev/ttyACM2"
        if "--help" in sys.argv:
            printHelp()
        elif "-h" in sys.argv:
            printHelp()
        elif "--port" in sys.argv:
            portName = sys.argv[sys.argv.index("--port") + 1]
            main(portName)

    except rospy.ROSInterruptException:
        pass
