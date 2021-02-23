#!/usr/bin/env python

import rospy
import math

from std_msgs.msg import Float64, Float32MultiArray, Float32
from math import sin,cos,atan2,sqrt,fabs


def callbackHeadPose(data):
    global tilt, pan
    pan = data.data[0]
    tilt = data.data[1]
    if tilt < 0.0 :
        tilt = -1*tilt

def callbackTorsoPose(data):
    global torso
    torso = data.data[0] + 0.2

def callbackLaPose(data):
    global la1, la2, la3, la4, la5, la6, la7
    la1 = data.data[0]
    la2 = data.data[1]
    la3 = data.data[2]
    la4 = data.data[3]
    la5 = data.data[4]
    la6 = data.data[5]
    la7 = data.data[6]

def callbackRaPose(data):
    global ra1, ra2, ra3, ra4, ra5, ra6, ra7
    ra1 = data.data[0]
    ra2 = data.data[1]
    ra3 = data.data[2]
    ra4 = data.data[3]
    ra5 = data.data[4]
    ra6 = data.data[5]
    ra7 = data.data[6]

def callbackLaGrip(data):
    global lag
    lag = data.data

def callbackRaGrip(data):
    global rag
    rag = data.data

#Define a RRBot joint positions publisher for joint controllers.
def justina_joint_positions_publisher():
        global tilt, pan
        global torso
        global la1, la2, la3, la4, la5, la6, la7
        global ra1, ra2, ra3, ra4, ra5, ra6, ra7
        global lag, rag
        
        pan = 0.0
        tilt = 0.0
        torso = 0.0
        
        la1 = 0.0
        la2 = 0.0
        la3 = 0.0
        la4 = 0.0
        la5 = 0.0
        la6 = 0.0
        la7 = 0.0
        
        ra1 = 0.0
        ra2 = 0.0
        ra3 = 0.0
        ra4 = 0.0
        ra5 = 0.0
        ra6 = 0.0
        ra7 = 0.0

        lag = 0.0
        rag = 0.0

	#Initiate node for controlling joint1 and joint2 positions.
	rospy.init_node('rrrbot_joint_positions_node', anonymous=True)

	#Define publishers for each joint position controller commands.
	pubHeadPan = rospy.Publisher('/justina/joint6_position_controller/command', Float64, queue_size=10)
	pubHeadTilt = rospy.Publisher('/justina/joint7_position_controller/command', Float64, queue_size=10)
        rospy.Subscriber("/hardware/head/goal_pose",Float32MultiArray, callbackHeadPose)
        
	pubTorso = rospy.Publisher('/justina/joint5_position_controller/command', Float64, queue_size=10)
        rospy.Subscriber("/hardware/torso/goal_pose",Float32MultiArray, callbackTorsoPose)


        pubLa1 = rospy.Publisher('/justina/la_1_controller/command', Float64, queue_size=10)
        pubLa2 = rospy.Publisher('/justina/la_2_controller/command', Float64, queue_size=10)
        pubLa3 = rospy.Publisher('/justina/la_3_controller/command', Float64, queue_size=10)
        pubLa4 = rospy.Publisher('/justina/la_4_controller/command', Float64, queue_size=10)
        pubLa5 = rospy.Publisher('/justina/la_5_controller/command', Float64, queue_size=10)
        pubLa6 = rospy.Publisher('/justina/la_6_controller/command', Float64, queue_size=10)
        pubLa7 = rospy.Publisher('/justina/la_7_controller/command', Float64, queue_size=10)
        rospy.Subscriber("/manipulation/manip_pln/la_goto_angles",Float32MultiArray, callbackLaPose)
	
        pubLaGripLeft = rospy.Publisher('/justina/la_grip_left_controller/command', Float64, queue_size=10)
        pubLaGripRight = rospy.Publisher('/justina/la_grip_right_controller/command', Float64, queue_size=10)
        rospy.Subscriber("/hardware/left_arm/goal_gripper", Float32, callbackLaGrip)
        
        pubRa1 = rospy.Publisher('/justina/ra_1_controller/command', Float64, queue_size=10)
        pubRa2 = rospy.Publisher('/justina/ra_2_controller/command', Float64, queue_size=10)
        pubRa3 = rospy.Publisher('/justina/ra_3_controller/command', Float64, queue_size=10)
        pubRa4 = rospy.Publisher('/justina/ra_4_controller/command', Float64, queue_size=10)
        pubRa5 = rospy.Publisher('/justina/ra_5_controller/command', Float64, queue_size=10)
        pubRa6 = rospy.Publisher('/justina/ra_6_controller/command', Float64, queue_size=10)
        pubRa7 = rospy.Publisher('/justina/ra_7_controller/command', Float64, queue_size=10)
        rospy.Subscriber("/manipulation/manip_pln/ra_goto_angles",Float32MultiArray, callbackRaPose)

        pubRaGripLeft = rospy.Publisher('/justina/ra_grip_left_controller/command', Float64, queue_size=10)
        pubRaGripRight = rospy.Publisher('/justina/ra_grip_right_controller/command', Float64, queue_size=10)
        rospy.Subscriber("/hardware/right_arm/goal_gripper", Float32, callbackRaGrip)


	rate = rospy.Rate(100) #100 Hz

	#While loop to have joints follow a certain position, while rospy is not shutdown.
	i = 0
	while not rospy.is_shutdown():


		pubHeadPan.publish(pan)
		pubHeadTilt.publish(tilt)
		
                pubTorso.publish(torso)
		
                pubLa1.publish(la1)
                pubLa2.publish(la2)
                pubLa3.publish(la3)
                pubLa4.publish(la4)
                pubLa5.publish(la5)
                pubLa6.publish(la6)
                pubLa7.publish(la7)

                pubRa1.publish(ra1)
                pubRa2.publish(ra2)
                pubRa3.publish(ra3)
                pubRa4.publish(ra4)
                pubRa5.publish(ra5)
                pubRa6.publish(ra6)
                pubRa7.publish(ra7)

                pubLaGripLeft.publish(lag)
                pubLaGripRight.publish(lag)
                pubRaGripLeft.publish(rag)
                pubRaGripRight.publish(rag)

		rate.sleep() #sleep for rest of rospy.Rate(100)


#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':
	try: justina_joint_positions_publisher()
	except rospy.ROSInterruptException: pass
