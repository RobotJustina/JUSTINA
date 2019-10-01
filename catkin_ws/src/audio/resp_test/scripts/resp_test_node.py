#!/usr/bin/env python

from tuning import Tuning
from std_msgs.msg import Float32, Bool
import rospy
import usb.core
import usb.util
import time
import angles
import math

try:
    from pixel_ring import usb_pixel_ring_v2
except IOError as e:
    print e
    raise RuntimeError("Check the device is connected and recognized")

class Tuning_respeaker(object):
	VENDOR_ID = 0x2886
	PRODUCT_ID = 0x0018

	def __init__ (self):
		self.TIMEOUT = 100000
		self.dev = usb.core.find(idVendor=self.VENDOR_ID,idProduct=self.PRODUCT_ID)
		if not self.dev:
			raise RuntimeError("Failed to find Respeaker device")
		rospy.loginfo("Initializing Respeaker device")
		self.pixel_ring = usb_pixel_ring_v2.PixelRing(self.dev)
		self.set_led_think()
		time.sleep(5)  # it will take 5 seconds to re-recognize as audio device
		self.set_led_trace()
		rospy.loginfo("Respeaker device initialized (Version: %s)" % self.version)
		self._set_vad = rospy.get_param("~vad", 70)
		self._agc_gain_factor = rospy.get_param("~agc_factor", 10)
		self.mic_tuning = Tuning(self.dev)

	def set_vad(self):
		self.mic_tuning.set_vad_threshold(self._set_vad)
	#Disable AGC default
	def set_agc(self):
		self.mic_tuning.set_automatic_gain_control(0)	

	def set_agc_gain(self):
		self.mic_tuning.set_Current_AGC_gain_factor(self._agc_gain_factor)	

	def set_agc_power_level(self):
		self.mic_tuning.set_AGC_power_level(0.00000001)

	def set_led_think(self):
		self.pixel_ring.set_brightness(10)
		self.pixel_ring.think()

	def set_led_trace(self):
		self.pixel_ring.set_brightness(20)
		self.pixel_ring.trace()

	@property
	def version(self):
		return self.dev.ctrl_transfer(
            usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, 0x80, 0, 1, self.TIMEOUT)[0]
		
	def get_doa_value(self):
		if self.dev:
			doa_rad = math.radians(self.mic_tuning.direction - 180.0)
			doa_rad = angles.shortest_angular_distance(doa_rad, math.radians(90.0))
			doa = math.degrees(doa_rad)
		return doa

	def get_vad_value(self):
		if self.dev:
			if self.mic_tuning.is_voice() == 1:
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
	pub_doa = rospy.Publisher('sound_direction', Float32, queue_size=1)
	pub_vad = rospy.Publisher('vad_sound', Bool, queue_size=1)
	rospy.init_node('doa_test', anonymous=True)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():

		doa_value = tuner_resp.get_doa_value()
		vad_value = tuner_resp.get_vad_value()
		pub_vad.publish(vad_value)
		if (vad_value == True):
			rospy.loginfo(doa_value)
			pub_doa.publish(doa_value)
		#rospy.loginfo(vad_value)
		rate.sleep()

if __name__ == '__main__':
	try:
		doa_publish()
	except rospy.ROSInterruptException:
		pass
