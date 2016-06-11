#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import tf

def printHelp():
    print "YOU DON'T REALLY NEED HELP TO OPERATE TORSO IN SIMULATION MODE"

def callbackGoalSpine(msg):
    global goalSpine
    goalSpine = msg.data 

def callbackGoalWaist(msg):
    global goalWaist
    goalWaist = msg.data

def callbackGoalShoulders(msg):
    global goalShoulders
    goalShoulders = msg.data


def main():
    print "INITIALIZING TORSO NODE IN SIMULATION MODE BY MARCOSOFT..."
    ###Connection with ROS
    rospy.init_node("torso")
    br = tf.TransformBroadcaster()	
    jointStates = JointState()
    jointStates.name = ["spine_connect","waist_connect","shoulders_connect", "shoulders_left_connect", "shoulders_right_connect"]
    jointStates.position = [0.0, 0.0, 0.0, 0.0, 0.0]
    
    subGoalPos = rospy.Subscriber("/hardware/torso/goal_spine", Float32, callbackGoalSpine)
    subGoalPos = rospy.Subscriber("/hardware/torso/goal_waist", Float32, callbackGoalWaist)
    subGoalPos = rospy.Subscriber("/hardware/torso/goal_shoulders", Float32, callbackGoalShoulders)
    pubJointStates = rospy.Publisher("/joint_states", JointState, queue_size = 1)
    pubCurrentSpine = rospy.Publisher("/hardware/torso/current_spine", Float32, queue_size=1)
    pubCurrentWaist = rospy.Publisher("/hardware/torso/current_waist", Float32, queue_size=1)
    pubCurrentShoulders = rospy.Publisher("/hardware/torso/current_shoulders", Float32, queue_size=1)

    loop = rospy.Rate(10)

    global goalSpine
    global goalWaist
    global goalShoulders
    goalSpine = 0.0
    goalWaist = 0.0
    goalShoulders = 0.0
    spine = 0
    waist = 0
    shoulders = 0
    speedSpine = 0.005
    speedWaist = 0.1
    speedShoulders = 0.1
    msgCurrentSpine = Float32()
    msgCurrentWaist = Float32()
    msgCurrentShoulders = Float32()
    
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

        msgCurrentSpine.data = spine
        msgCurrentWaist.data = waist
        msgCurrentShoulders.data = shoulders
        pubCurrentSpine.publish(msgCurrentSpine)
	pubCurrentWaist.publish(msgCurrentWaist)
	pubCurrentShoulders.publish(msgCurrentShoulders)
        
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
