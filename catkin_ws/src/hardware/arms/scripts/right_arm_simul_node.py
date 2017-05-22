#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import tf

def printHelp():
    print "RIGHT ARM NODE IN SIMULATION BY MARCOSOfT"
    print " - This node operates the right arm in a very low level. "
    print " - Right arm can be moved in two modes: by specifying a goal position"
    print " - or by specifying a torque. In both cases, seven values are required."
    print " - Goal position or torque can be specified by publishin the corresponding topic."
    print " - Positions (goal and current) are published in [rad]"
    print " - Torque must be specified in values in [-1, 1] where 1 represents the"
    print " - maximum torque that the corresponding motor can generate."
    print "PLEASE DON'T TRY TO OPERATE JUSTINA IF YOU ARE NOT QUALIFIED ENOUGH."

def callbackTorqueGripper(msg):
    print "Right arm torqueMode gripper"

def callbackPos(msg):
    global goalAngles
    global speeds
    goalAngles = [0, 0, 0, 0, 0, 0, 0]
    speeds = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    if len(msg.data) == 7:
        for i in range(len(msg.data)):
            goalAngles[i] = msg.data[i]
    elif len(msg.data) == 14:
        for i in range(7):
            goalAngles[i] = msg.data[i]
            speeds[i] = msg.data[i+7]

def callbackGripper(msg):
    global goalGripper
    goalGripper = msg.data
    
def main():
    print "INITIALIZING RIGHT ARM NODE IN SIMULATION BY MARCOSOFT..."
    ###Connection with ROS
    rospy.init_node("right_arm")
    br = tf.TransformBroadcaster()
    jointStates = JointState()
    jointStates.name = ["ra_1_joint", "ra_2_joint", "ra_3_joint", "ra_4_joint", "ra_5_joint", "ra_6_joint", "ra_7_joint", "ra_grip_left", "ra_grip_right"]
    jointStates.position = [0, 0, 0, 0, 0, 0, 0, 0, 0]

    subPosition = rospy.Subscriber("right_arm/goal_pose", Float32MultiArray, callbackPos)
    subGripper = rospy.Subscriber("/hardware/right_arm/goal_gripper", Float32, callbackGripper)
    subTorqueGripper = rospy.Subscriber("/hardware/right_arm/torque_gripper", Float32, callbackTorqueGripper)

    pubArmPose = rospy.Publisher("right_arm/current_pose", Float32MultiArray, queue_size = 1)
    pubGripper = rospy.Publisher("right_arm/current_gripper", Float32, queue_size = 1)
    pubJointStates = rospy.Publisher("/joint_states", JointState, queue_size = 1)
    pubArmBattery = rospy.Publisher("/hardware/robot_state/right_arm_battery", Float32, queue_size=1)

    loop = rospy.Rate(10)

    global goalAngles;
    global goalGripper
    global speeds
    goalAngles = [0, 0, 0, 0, 0, 0, 0]
    angles = [0, 0, 0, 0, 0, 0, 0]
    speeds = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    goalGripper = 0
    gripper = 0
    gripperSpeed = 0.1
    msgCurrentPose = Float32MultiArray()
    msgCurrentPose.data = [0, 0, 0, 0, 0, 0, 0]
    msgCurrentGripper = Float32()
    msgCurrentGripper.data = 0
    deltaAngles = [0, 0, 0, 0, 0, 0, 0]
    deltaGripper = 0
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

        deltaGripper = goalGripper - gripper
        if deltaGripper > gripperSpeed:
            deltaGripper = gripperSpeed
        if deltaGripper < -gripperSpeed:
            deltaGripper = -gripperSpeed
        gripper += deltaGripper
        jointStates.position[7] = gripper
        jointStates.position[8] = gripper 
        msgCurrentGripper.data = gripper
        
        jointStates.header.stamp = rospy.Time.now()
        pubJointStates.publish(jointStates)
        pubArmPose.publish(msgCurrentPose)
        pubGripper.publish(msgCurrentGripper);
        msgBattery = Float32()
        msgBattery.data = 11.6
        pubArmBattery.publish(msgBattery);
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

