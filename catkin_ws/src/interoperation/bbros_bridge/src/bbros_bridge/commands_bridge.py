import rospy
from pyrobotics import BB
from pyrobotics.messages import Command, Response
from bbros_bridge.srv import *	#import all the services from bbros_bridge

class ROS2BB_CommandsCalls:
	"""Class to bridge the BB Commands to allow ROS Calls
	"""
	def __init__(self, commandName, messageType):
		"""Constructor
		Creates a new ROS2BB_CommandsCalls Object.
		Receives:
			commandName: Name of the command to bridge.
		"""
		self.commandName = commandName
		self.messageType = messageType
		#create a ros service associated with the bb command to bridge
		rospy.Service(self.commandName, self.messageType, self.commandCaller)

	def commandCaller(self, req):
		"""
		Handle the ros service corresponding to the BB command to call
		"""
		#send the parameters to BB directly
		rospy.logdebug('Sending parameters "' + str(req.parameters) + '" to command "' + str(self.commandName) + '" with timeout ' + str(req.timeout))
		commandResponse = BB.SendAndWait(Command(self.commandName, req.parameters), req.timeout)

		ret = Default_ROS_BB_BridgeResponse()
		ret.success = False
		ret.response = ''
		if commandResponse and commandResponse.successful:
			#return the command response
			ret.success = True
			ret.response = str(commandResponse.params)
			rospy.logdebug('Response received from command "' + str(self.commandName) + '":  ' + str(commandResponse.params))

		return ret

