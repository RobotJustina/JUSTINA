#!/usr/bin/env python
from tuning import Tuning
from usb_pixel_ring_v2 import PixelRing
from std_msgs.msg import Int8, Bool
import rospy
import usb.core
import usb.util
import time
import pyaudio
import numpy as np
import scipy.signal
import time

class Tuning_respeaker(object):
	VENDOR_ID = 0x2886
	PRODUCT_ID = 0x0018

	def __init__ (self):
		self.dev = usb.core.find(idVendor=self.VENDOR_ID,idProduct=self.PRODUCT_ID)
		if not self.dev:
			raise RuntimeError("Failed to find Respeaker device")
		pixel_ring = PixelRing(self.dev)
		rospy.loginfo("Initializing Respeaker device")
		#pixel_ring.think()
		#time.sleep(2)
		#pixel_ring.set_color(False,200,200,1)

	def set_vad(self):
		mic_tuning = Tuning(self.dev)
		mic_tuning.set_vad_threshold(65)
		

	def get_doa_value(self):
		if self.dev:
			mic_tuning = Tuning(self.dev)
			#pixel_ring = PixelRing(self.dev)
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
	pub_doa = rospy.Publisher('sound_direction', Int8, queue_size=1)
	pub_vad = rospy.Publisher('is_voice', Bool, queue_size=1)
	rospy.init_node('doa_test', anonymous=True)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():

		doa_value = tuner_resp.get_doa_value()
		vad_value = tuner_resp.get_vad_value()
		#rospy.loginfo(doa_value)
		if (vad_value == True):
			rospy.loginfo(doa_value)
		pub_doa.publish(doa_value)
		pub_vad.publish(vad_value)
		rate.sleep()

if __name__ == '__main__':
	try:
		doa_publish()
	except rospy.ROSInterruptException:
		pass
