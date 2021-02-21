#!/usr/bin/env python

import rospy
import math

from std_msgs.msg import Float64, Float32MultiArray
from math import sin,cos,atan2,sqrt,fabs


def callbackHeadPose(data):
    global tilt, pan
    pan = data.data[0]
    tilt = data.data[1]
    if tilt < 0.0 :
        tilt = -1*tilt

def callbackLaPose(data):
    global la1, la2, la3, la4, la5, la5, la7
    la1 = data.data[0]
    la2 = data.data[1]
    la3 = data.data[2]
    la4 = data.data[3]
    la5 = data.data[4]
    la6 = data.data[5]
    la7 = data.data[6]
    
#Define a RRBot joint positions publisher for joint controllers.
def justina_joint_positions_publisher():
        global tilt, pan
        global la1, la2, la3, la4, la5, la5, la7

        pan = 0.0
        tilt = 0.0
        la1 = 0.0
        la2 = 0.0
        la3 = 0.0
        la4 = 0.0
        la5 = 0.0
        la6 = 0.0
        la7 = 0.0

	#Initiate node for controlling joint1 and joint2 positions.
	rospy.init_node('rrrbot_joint_positions_node', anonymous=True)

	#Define publishers for each joint position controller commands.
	pubHeadPan = rospy.Publisher('/justina/joint6_position_controller/command', Float64, queue_size=10)
	pubHeadTilt = rospy.Publisher('/justina/joint7_position_controller/command', Float64, queue_size=10)
        rospy.Subscriber("/hardware/head/goal_pose",Float32MultiArray, callbackHeadPose)


        pubLa1 = rospy.Publisher('/justina/la_1_controller/command', Float64, queue_size=10)
        pubLa2 = rospy.Publisher('/justina/la_2_controller/command', Float64, queue_size=10)
        pubLa3 = rospy.Publisher('/justina/la_3_controller/command', Float64, queue_size=10)
        pubLa4 = rospy.Publisher('/justina/la_4_controller/command', Float64, queue_size=10)
        pubLa5 = rospy.Publisher('/justina/la_5_controller/command', Float64, queue_size=10)
        pubLa6 = rospy.Publisher('/justina/la_6_controller/command', Float64, queue_size=10)
        pubLa7 = rospy.Publisher('/justina/la_7_controller/command', Float64, queue_size=10)
        rospy.Subscriber("/manipulation/manip_pln/la_goto_angles",Float32MultiArray, callbackLaPose)

	rate = rospy.Rate(100) #100 Hz

	#While loop to have joints follow a certain position, while rospy is not shutdown.
	i = 0
	while not rospy.is_shutdown():


		pubHeadPan.publish(pan)
		pubHeadTilt.publish(tilt)
		
                pubLa1.publish(la1)
                pubLa2.publish(la2)
                pubLa3.publish(la3)
                pubLa4.publish(la4)
                pubLa5.publish(la5)
                pubLa6.publish(la6)
                pubLa7.publish(la7)

		rate.sleep() #sleep for rest of rospy.Rate(100)


#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':
	try: justina_joint_positions_publisher()
	except rospy.ROSInterruptException: pass
