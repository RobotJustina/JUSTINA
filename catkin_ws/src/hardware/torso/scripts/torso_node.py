#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import tf

def callbackGoalPos(msg):
    global goalPos
    goalPos = msg.data 


def main():
    print "INITIALIZING TORSO NODE..."
    global goalPos
    goalPos = 0.0
    
    ###Connection with ROS
    rospy.init_node("torso")
    br = tf.TransformBroadcaster()
    msgCurrentPos = Float32()
    jointStates = JointState()
    jointStates.name = ["torso_connect"]
    jointStates.position = [0.0]
    
    ## Subscribers
    pubJointStates = rospy.Publisher("/joint_states", JointState, queue_size = 1)
    pubCurrentPos = rospy.Publisher("/hardware/torso/current_pose", Float32, queue_size=1)
    subGoalPos = rospy.Subscriber("/hardware/torso/goal_pose", Float32, callbackGoalPos)

   
    loop = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        jointStates.header.stamp = rospy.Time.now()
        jointStates.position = [goalPos]
        pubCurrentPos.publish(msgCurrentPos)
        pubJointStates.publish(jointStates)

        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
