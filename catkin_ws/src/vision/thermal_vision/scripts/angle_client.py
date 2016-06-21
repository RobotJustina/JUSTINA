#!/usr/bin/env python

import rospy
from vision_msgs.srv import GetThermalAngle

def main():
	
	rospy.wait_for_service('thermal_angle')
	
	try:
		thermal_angle = rospy.ServiceProxy('thermal_angle', GetThermalAngle)
		resp1 = thermal_angle()
		print 'angle = %.2f'%resp1.th_angle
	
	except rospy.ServiceException, e:
		print "Service call failled: %s"%e

if __name__ == '__main__':
	main()