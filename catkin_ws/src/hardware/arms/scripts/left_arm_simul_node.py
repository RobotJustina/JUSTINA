#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import tf

def printHelp():
    print "LEFT ARM NODE IN SIMULATION BY MARCOSOfT. Options:"
    
def main():
    print "INITIALIZING LEFT ARM NODE IN SIMULATION BY MARCOSOFT..."
    ###Connection with ROS
    rospy.init_node("left_arm")
    br = tf.TransformBroadcaster()
    loop = rospy.Rate(10)
    jointStates = JointState()
    jointStates.name = ["la_1_joint", "la_2_joint", "la_3_joint", "la_4_joint", "la_5_joint", "la_6_joint", "la_7_joint"]
    jointStates.position = [0, 0, 0, 0, 0, 0, 0]
    pubJointStates = rospy.Publisher("/joint_states", JointState, queue_size = 1)
    while not rospy.is_shutdown():
        pos0 = 0
        pos1 = 0 
        pos2 = 0
        pos3 = 0
        pos4 = 0
        pos5 = 0
        pos6 = 0
        posD21 = 0
        posD22 = 0
        
        jointStates.header.stamp = rospy.Time.now()
        jointStates.position[0] = pos0
        jointStates.position[1] = pos1
        jointStates.position[2] = pos2
        jointStates.position[3] = pos3
        jointStates.position[4] = pos4
        jointStates.position[5] = pos5
        jointStates.position[6] = pos6
        pubJointStates.publish(jointStates)
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
