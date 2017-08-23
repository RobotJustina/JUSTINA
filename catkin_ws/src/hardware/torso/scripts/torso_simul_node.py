#!/usr/bin/env python
import sys
import rospy
import math
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import tf

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
    msg = Bool()
    msg.data = False
    pubGoalReached.publish(msg)

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
    msg = Bool()
    msg.data = False
    pubGoalReached.publish(msg)
        

def main():
    print "INITIALIZING TORSO NODE IN SIMULATION MODE BY MARCOSOFT..."
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
    spine = 0.10
    waist = 0
    shoulders = 0
    goalSpine = spine
    goalWaist = waist
    goalShoulders = shoulders
    
    speedSpine = 0.005
    speedWaist = 0.1
    speedShoulders = 0.1
    msgCurrentPose = Float32MultiArray()
    msgGoalReached = Bool()
    msgCurrentPose.data = [0, 0, 0]
    newGoal = False
    
    while not rospy.is_shutdown():
        deltaSpine = goalSpine - spine;
        deltaWaist = goalWaist - waist;
        deltaShoulders = goalShoulders - shoulders;
        if deltaSpine > speedSpine:
            deltaSpine = speedSpine;
        if deltaSpine < -speedSpine:
            deltaSpine = -speedSpine;
        if deltaWaist > speedWaist:
            deltaWaist = speedWaist;
        if deltaWaist < -speedWaist:
            deltaWaist = -speedWaist;
        if deltaShoulders > speedShoulders:
            deltaShoulders = speedShoulders;
        if deltaShoulders < -speedShoulders:
            deltaShoulders = -speedShoulders;

        spine += deltaSpine
        waist += deltaWaist
        shoulders += deltaShoulders
        jointStates.header.stamp = rospy.Time.now()
        jointStates.position = [spine, waist, shoulders, -shoulders, -shoulders]
        pubJointStates.publish(jointStates)

        msgCurrentPose.data[0] = spine
        msgCurrentPose.data[1] = waist
        msgCurrentPose.data[2] = shoulders
        pubCurrentPose.publish(msgCurrentPose)

        if newGoal and abs(goalSpine - spine) < 0.02 and abs(goalWaist - waist) < 0.05 and abs(goalShoulders - shoulders) < 0.05:
            newGoal = False
            msgGoalReached.data = True
            pubGoalReached.publish(msgGoalReached)
        
        loop.sleep()

if __name__ == '__main__':
    try:
        if "--help" in sys.argv:
            printHelp()
        elif "-h" in sys.argv:
            printHelp()
        else:
            main()
    except rospy.ROSInterruptException:
        pass
