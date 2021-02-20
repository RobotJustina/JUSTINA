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
    
#Define a RRBot joint positions publisher for joint controllers.
def justina_joint_positions_publisher():
        global tilt, pan

        pan = 0.0
        tilt = 0.0

	#Initiate node for controlling joint1 and joint2 positions.
	rospy.init_node('rrrbot_joint_positions_node', anonymous=True)

	#Define publishers for each joint position controller commands.
	pubHeadPan = rospy.Publisher('/justina/joint6_position_controller/command', Float64, queue_size=10)
	pubHeadTilt = rospy.Publisher('/justina/joint7_position_controller/command', Float64, queue_size=10)
	pub3 = rospy.Publisher('/justina/joint3_position_controller/command', Float64, queue_size=10)
        rospy.Subscriber("/hardware/head/goal_pose",Float32MultiArray, callbackHeadPose)

	rate = rospy.Rate(100) #100 Hz

	#While loop to have joints follow a certain position, while rospy is not shutdown.
	i = 0
	while not rospy.is_shutdown():


		pubHeadPan.publish(pan)
		pubHeadTilt.publish(tilt)

		rate.sleep() #sleep for rest of rospy.Rate(100)


#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':
	try: justina_joint_positions_publisher()
	except rospy.ROSInterruptException: pass
