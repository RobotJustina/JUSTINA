#!/usr/bin/env python

from tuning import Tuning
from std_msgs.msg import Int16, Bool
import rospy
import usb.core
import usb.util
import time


class Tuning_respeaker(object):
	VENDOR_ID = 0x2886
	PRODUCT_ID = 0x0018

	def __init__ (self):
		self.dev = usb.core.find(idVendor=self.VENDOR_ID,idProduct=self.PRODUCT_ID)
		if not self.dev:
			raise RuntimeError("Failed to find Respeaker device")
		rospy.loginfo("Initializing Respeaker device")

	def set_vad(self):
		mic_tuning = Tuning(self.dev)
		mic_tuning.set_vad_threshold(50)

	def set_agc(self):
		mic_tuning = Tuning(self.dev)
		mic_tuning.set_automatic_gain_control(0)	

	def set_agc_gain(self):
		mic_tuning = Tuning(self.dev)
		mic_tuning.set_Current_AGC_gain_factor(10)	

	def set_agc_power_level(self):
		mic_tuning = Tuning(self.dev)
		mic_tuning.set_AGC_power_level(0.00000001)


	def get_doa_value(self):
		if self.dev:
			mic_tuning = Tuning(self.dev)
			doa_val = mic_tuning.direction
		return doa_val

	def get_vad_value(self):
		if self.dev:
			mic_tuning = Tuning(self.dev)
			if mic_tuning.is_voice() == 1:
				bool_vad = True
			else:
				bool_vad = False
		return bool_vad

def doa_publish():
	tuner_resp = Tuning_respeaker()
	tuner_resp.set_vad()
	tuner_resp.set_agc()
        tuner_resp.set_agc_gain()
        tuner_resp.set_agc_power_level()
	pub_doa = rospy.Publisher('sound_direction', Int16, queue_size=1)
	pub_vad = rospy.Publisher('vad_sound', Bool, queue_size=1)
	rospy.init_node('doa_test', anonymous=True)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():

		doa_value = tuner_resp.get_doa_value()
		vad_value = tuner_resp.get_vad_value()
		pub_doa.publish(doa_value)
		pub_vad.publish(vad_value)
		if (vad_value == True):
			rospy.loginfo(doa_value)
		#rospy.loginfo(vad_value)
		rate.sleep()

if __name__ == '__main__':
	try:
		doa_publish()
	except rospy.ROSInterruptException:
		pass
