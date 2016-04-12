#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import tf

def printHelp():
    print "RIGHT ARM NODE IN SIMULATION BY MARCOSOfT. Options:"

def callbackPos(msg):
    global goalAngles
    goalAngles = [0, 0, 0, 0, 0, 0, 0]
    for i in range(len(msg.data)):
        goalAngles[i] = msg.data[i]
    
def main():
    print "INITIALIZING RIGHT ARM NODE IN SIMULATION BY MARCOSOFT..."
    ###Connection with ROS
    rospy.init_node("right_arm")
    br = tf.TransformBroadcaster()
    jointStates = JointState()
    jointStates.name = ["ra_1_joint", "ra_2_joint", "ra_3_joint", "ra_4_joint", "ra_5_joint", "ra_6_joint", "ra_7_joint"]
    jointStates.position = [0, 0, 0, 0, 0, 0, 0]

    subPosition = rospy.Subscriber("right_arm/goal_pose", Float32MultiArray, callbackPos)
    pubArmPose = rospy.Publisher("right_arm/current_pose", Float32MultiArray, queue_size = 1)
    pubGripper = rospy.Publisher("right_arm/current_gripper", Float32, queue_size = 1)
    pubJointStates = rospy.Publisher("/joint_states", JointState, queue_size = 1)

    loop = rospy.Rate(10)

    global goalAngles;
    goalAngles = [0, 0, 0, 0, 0, 0, 0]
    angles = [0, 0, 0, 0, 0, 0, 0]
    speeds = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    msgCurrentPose = Float32MultiArray()
    msgCurrentPose.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    msgCurrentGripper = Float32()
    msgCurrentGripper.data = 0.0
    deltaAngles = [0, 0, 0, 0, 0, 0, 0]
    while not rospy.is_shutdown():
        for i in range(len(deltaAngles)):
            deltaAngles[i] = goalAngles[i] - angles[i]
            if deltaAngles[i] > speeds[i]:
                deltaAngles[i] = speeds[i]
            if deltaAngles[i] < -speeds[i]:
                deltaAngles[i] = -speeds[i]
            angles[i] += deltaAngles[i]
            jointStates.position[i] = angles[i]
            msgCurrentPose.data[i] = angles[i]
        
        jointStates.header.stamp = rospy.Time.now()
        pubJointStates.publish(jointStates)
        pubArmPose.publish(msgCurrentPose)
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

