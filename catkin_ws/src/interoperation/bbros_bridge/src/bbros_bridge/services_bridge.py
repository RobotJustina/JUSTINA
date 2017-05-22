import rospy
from pyrobotics import BB
from pyrobotics.messages import Command, Response
from bbros_bridge.srv import *	#import all the services from bbros_bridge

class BB2ROS_DefaultServicesCallers:
	"""Class to bridge the ROS Topics to allow calls from BB
	"""
	@staticmethod
	def default_caller(command):
		"""
		BB Command callback which:
			1. Attends a BB Command Invocation.
			2. Parses the parameters list comming from BlackBoard to the Default_BB_ROS_Bridge format.
			3. Invoke the correponding service (which has the same name of the bridged command) with the parsed parameters and waits for his response.
			4. Send the response received from the ROS service and send it to Blackboard.
		Receives:
			command: A command (pyrobotics) object 
		Return:
			The response to the command caller
		"""
		rospy.logdebug('Call to command "' + str(command) + '" received from BB.')

		#send directly the parameters receved from BB to the ros service
		try:
			rospy.wait_for_service(command.name, timeout=4)
			default_service = rospy.ServiceProxy(command.name, Default_BB_ROS_Bridge)
			resp1 = default_service(command.params)
			return Response.FromCommandObject(command, True, str(resp1.response))
		except rospy.ServiceException:
			rospy.logfatal('Call to service: "' + str(command.name) + '" failed.')
		except rospy.ROSException:
			rospy.logfatal('Timeout exceeded while waiting for the service ' + command.name + ' to be available.' )

		#send the response to BB
		return Response.FromCommandObject(command, False, '')


