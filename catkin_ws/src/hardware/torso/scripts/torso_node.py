#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import tf

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
    print "INITIALIZING TORSO NODE..."
    global goalSpine
    global goalWaist
    global goalShouldes
    goalSpine = 0.0
    goalWaist = 0.0
    goalShoulders = 0.0
       
    ###Connection with ROS
    rospy.init_node("torso")
    br = tf.TransformBroadcaster()
    msgCurrentSpin = Float32()
    msgCurrentWaist = Float32()
    msgCurrentShoulders = Float32()
	
    jointStates = JointState()
    jointStates.name = ["spine_connect","waist_connect","shoulders_connect"]
    jointStates.position = [0.0,0.0,0.0]
    
    ## Subscribers
    pubJointStates = rospy.Publisher("/joint_states", JointState, queue_size = 1)
    pubCurrentSpine = rospy.Publisher("/hardware/torso/current_spine", Float32, queue_size=1)
    pubCurrentWaist = rospy.Publisher("/hardware/torso/current_waist", Float32, queue_size=1)
    pubCurrentShoulders = rospy.Publisher("/hardware/torso/current_shoulders", Float32, queue_size=1)

    subGoalPos = rospy.Subscriber("/hardware/torso/goal_spine", Float32, callbackGoalSpine)
    subGoalPos = rospy.Subscriber("/hardware/torso/goal_waist", Float32, callbackGoalWaist)
    subGoalPos = rospy.Subscriber("/hardware/torso/goal_shoulders", Float32, callbackGoalShoulders)

    loop = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        jointStates.header.stamp = rospy.Time.now()
        jointStates.position = [goalSpine,goalWaist,goalShoulders]
    
        pubCurrentSpine.publish(msgCurrentSpin)
	pubCurrentWaist.publish(msgCurrentWaist)
	pubCurrentShoulders.publish(msgCurrentShoulders)
	
        pubJointStates.publish(jointStates)
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
