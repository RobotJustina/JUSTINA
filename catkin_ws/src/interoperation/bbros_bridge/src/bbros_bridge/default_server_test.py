#!/usr/bin/env python
import rospy
from bbros_bridge.srv import *

def handle_default_server(req):
	print 'Parameters received: ' + str(req.params)

	#Parse req.params here

	#Do all the math you need with the parameters here

	#Create the response to send and store it in responseToSend
	responseToSend = 'Hello from ROS'
	print 'Sending response: ' + responseToSend

	return Default_BB_ROS_BridgeResponse(responseToSend)

def default_server():
	rospy.init_node('default_server')
	s = rospy.Service('default_server', Default_BB_ROS_Bridge, handle_default_server)
	print "Listening calls from BB..."
	rospy.spin()

if __name__ == "__main__":
	default_server()
