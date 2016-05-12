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
    
    ###Connection with ROS
    rospy.init_node("head")
    br = tf.TransformBroadcaster()
    msgCurrentPos = Float32()
    jointStates = JointState()
    jointStates.name = ["shoulders_connect"]
    jointStates.position = [0.0]
    
    ## Subscribers
    pubJointStates = rospy.Publisher("/joint_states", JointState, queue_size = 1)
    pubCurrentPos = rospy.Publisher("/hardware/torso_current_pose", Float32, queue_size=1)
    subGoalPos = rospy.Subscriber("/hardware/torso_goal_pose", Float32, callbackGoalPos)

   
    loop = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        jointStates.position = [goalPos]
        pubCurrentPos.publish(pos)
        pubJointStates.publish(jointStates)

        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
