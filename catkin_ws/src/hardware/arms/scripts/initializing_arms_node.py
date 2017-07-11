#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Float32MultiArray

def main():
    print "JustinaHardwareInitializeArms.->INITIALIZING ARMS NODE"


    global goalPos
    global attemps


    ###Communication with dynamixels:

    msgGoalPoseRA = Float32MultiArray()
    msgGoalPoseLA = Float32MultiArray()

    msgGoalPoseRA.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5707]
    msgGoalPoseLA.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5707]
    attemps = 0
    i = 0


    ##############################
    ###  Connection with ROS  ####

    rospy.init_node("initializing_arms")

    pubGoalPoseRA = rospy.Publisher("/hardware/right_arm/goal_pose", Float32MultiArray, queue_size = 1)
    pubGoalPoseLA = rospy.Publisher("/hardware/left_arm/goal_pose", Float32MultiArray, queue_size = 1)

    loop = rospy.Rate(30)

    rospy.sleep(5)

    for x in range(1,100):
        pubGoalPoseRA.publish(msgGoalPoseRA)
        pubGoalPoseLA.publish(msgGoalPoseLA)
        rospy.sleep(0.05)


if __name__ == '__main__':
    main()