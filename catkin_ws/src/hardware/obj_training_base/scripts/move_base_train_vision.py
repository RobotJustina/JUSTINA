#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sys
#sys.path.append('../../../tools/hardware_tools/src/hardware_tools/')
#sys.path.append("/home/biorobotica/JUSTINA/catkin_ws/src/tools/hardware_tools/src/hardware_tools/")
#sys.path.append("/home/fede/JUSTINA/catkin_ws/src/tools/hardware_tools/src/hardware_tools/")

import Dynamixel
import time

PORT = "/dev/justinaObjTrainBase"
#PORT = "/dev/ttyUSB0"
BAUDRATE = 1000000
DELAY_SAFE_DATA = 0.2
DELAY_FOR_MOVE = 2
MAX_ENCODER = 4096
BASE_MOTOR_SPEED = 60
MOTOR_ID = 1
INC_POS = 400
TEST_POSE1 = 1000
TEST_POSE2 = 0



class Base:
	def __init__(self):
		self.motor = Dynamixel.DynamixelMan(PORT,BAUDRATE)
		time.sleep(DELAY_SAFE_DATA)
		self.motor.SetMovingSpeed(MOTOR_ID, BASE_MOTOR_SPEED)

	def move_base(self):
		time.sleep(DELAY_SAFE_DATA)
		actualPos = self.motor.GetPresentPosition(MOTOR_ID)
		time.sleep(DELAY_SAFE_DATA)
		self.motor.SetGoalPosition( MOTOR_ID, (actualPos + INC_POS) % MAX_ENCODER )
		time.sleep(DELAY_FOR_MOVE)

	def test_motor_base (self):
		self.motor.SetGoalPosition(MOTOR_ID,TEST_POSE1)
		time.sleep(DELAY_FOR_MOVE)
		self.motor.SetGoalPosition(MOTOR_ID,TEST_POSE2)
		time.sleep(DELAY_FOR_MOVE)

	def get_base_pose(self):
		time.sleep(DELAY_SAFE_DATA)
		return self.motor.GetPresentPosition(MOTOR_ID)
		time.sleep(DELAY_SAFE_DATA)

base = Base() 

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    rospy.loginfo(base.get_base_pose())
    base.move_base()

    
def listener():
    rospy.init_node('move_base_train_obj_node', anonymous=True)
    rospy.Subscriber("/hardware/obj_train_base", String, callback)  
    base.test_motor_base()
    rospy.spin()

if __name__ == '__main__':
    listener()


