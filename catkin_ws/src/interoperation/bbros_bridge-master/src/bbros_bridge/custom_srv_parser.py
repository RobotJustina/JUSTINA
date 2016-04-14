import rospy
import StringIO
from pyrobotics import BB
from pyrobotics.messages import Command, Response

#TODO: change the import to the motion planner package
from bbros_bridge.srv import *	#import all the services from bbros_bridge

class BB2ROS_HeadServicesCallers:
	"""Class to bridge the HEAD PACKAGE ROS Services to allow calls from BB
	"""

	@staticmethod
	def hd_lookat_caller(command):
		"""
		BB Command callback which:
			1. Attends a BB Command Invocation.
			2. Parses the parameters list comming from BlackBoard to the hd_lookat format.
			3. Invoke the correponding service (which has the same name of the bridged command) with the parsed parameters and waits for his response.
			4. Send the response received from the ROS service and send it to Blackboard.
		Receives:
			command: A command (pyrobotics) object 
		Return:
			The response to the command caller
		"""
		rospy.logdebug('Call to service: "'+ str(command.name) + '" received from BB.')
		try:
			#parse the parameters comming from BB to the service specific request format
			head_pan = float(command.params.split(' ')[0])
			head_tilt = float(command.params.split(' ')[1])

			#send the parameters to the ROS service
			rospy.wait_for_service(command.name, timeout=4)	#wait for the service to be available
			hd_lookat_service = rospy.ServiceProxy(command.name, hd_lookat)
			service_response = hd_lookat_service(head_pan, head_tilt)
			#parse the response obtained from the ros service
			command_response = str(service_response.currentPan) + str(service_response.currentTilt) + str(service_response.errors)
			return Response.FromCommandObject(command, True, command_response)
		except rospy.ServiceException, e:
			rospy.logfatal('Service call ' + command.name + ' failed: ' + str(e))
		except rospy.ROSException:
			rospy.logfatal('Timeout exceeded while waiting for the service ' + command.name + ' to be available.' )
		except:
			rospy.logfatal('An error ocurred when trying to parse the params: "' + command.params + '" for the ' + command.name + ' service')

		#send the response to BB
		return Response.FromCommandObject(command, False, '')

	@staticmethod
	def hd_torque_caller(command):
		"""
		BB Command callback which:
			1. Attends a BB Command Invocation.
			2. Parses the parameters list comming from BlackBoard to the hd_torque format.
			3. Invoke the correponding service (which has the same name of the bridged command) with the parsed parameters and waits for his response.
			4. Send the response received from the ROS service and send it to Blackboard.
		Receives:
			command: A command (pyrobotics) object 
		Return:
			The response to the command caller
		"""
		rospy.logdebug('Call to service: "'+ str(command.name) + '" received from BB.')
		try:
			#parse the parameters comming from BB to the service specific request format
			enable = str(command.params).lower()

			#send the parameters to the ROS service
			rospy.wait_for_service(command.name, timeout=4)	#wait for the service to be available
			hd_lookat_service = rospy.ServiceProxy(command.name, hd_lookat)
			service_response = hd_lookat_service(head_pan, head_tilt)
			#parse the response obtained from the ros service
			command_response = str(service_response.currentPan) + str(service_response.currentTilt) + str(service_response.errors)
			return Response.FromCommandObject(command, True, command_response)
		except rospy.ServiceException, e:
			rospy.logfatal('Service call ' + command.name + ' failed: ' + str(e))
		except rospy.ROSException:
			rospy.logfatal('Timeout exceeded while waiting for the service ' + command.name + ' to be available.' )
		except:
			rospy.logfatal('An error ocurred when trying to parse the params: "' + command.params + '" for the ' + command.name + ' service')

		#send the response to BB
		return Response.FromCommandObject(command, False, '')
