import bridge_utils
import rospy
import time
from pyrobotics import BB
from pyrobotics.messages import Command, Response
from pyrobotics.shared_variables import SharedVarTypes

class ROS2BBPublisher:
	"""Class to bridge the ROS topics to BB shared vars
	"""
	def __init__(self, rosTopicName, rosMsgType):
		"""Constructor
		Creates a new instance of ROS2BBPublisher
		Receives:
			rosTopicName: the name of the ROS topic to bridge
			rosMsgType: the ROS msg type of the topic
		"""
		self.rosTopicName = rosTopicName
		#build the bb shared var name
		self.bbVarName = rosTopicName.replace("/","")
		#try to get the ros msg type
		try:
			self.rosMsgType = bridge_utils.MSGTYPEALIAS_TO_MSGTYPESYSTEM[rosMsgType]
		except KeyError:
			self.rosMsgType = bridge_utils.MSGTYPEALIAS_TO_MSGTYPESYSTEM['default']
			rospy.logerr('An error ocurred when trying to get the ROS Msg System Type for the representation: "' + str(rosMsgType) + 
									'". Dictionary: ' + str(bridge_utils.MSGTYPEALIAS_TO_MSGTYPESYSTEM) + '. Try to bridge topic "' + str(rosTopicName) + '" will be bridged with default type.')
		except:
			self.rosMsgType = bridge_utils.MSGTYPEALIAS_TO_MSGTYPESYSTEM['default']
			rospy.logfatal('An unexpected error ocurred when trying to get the ROS Msg System Type for the representation: "' + str(rosMsgType) + 
									'". Dictionary: ' + str(bridge_utils.MSGTYPEALIAS_TO_MSGTYPESYSTEM) + '. The topic "' + str(rosTopicName) + '" will be bridged with default type.')

		try:
			#get the corresponding BB SV type
			self.bbVarType = bridge_utils.ROS2BB_TYPE_MAP[self.rosMsgType][0]
		except KeyError:
			#get the corresponding BB SV type
			self.bbVarType = bridge_utils.ROS2BB_TYPE_MAP['default'][0]
			rospy.logerr('The ROS MSG Type "' + str(self.rosMsgType) + '" doesn\'t have a valid BB representation. Dictionary: ' + str(bridge_utils.ROS2BB_TYPE_MAP) + 
									'. The topic "' + str(self.rosTopicName) + '" will be bridged with the default type')
		except:
			self.bbVarType = bridge_utils.BB_STRING
			rospy.logfatal('An unexpected error ocurred when trying to bridge the "' + str(self.rosTopicName) + '" ROS topic. Trying to use the default BB  type: BB_STRING.')

		#try to create the corresponding BB SV 
		if self.createBBSV(self.bbVarName, self.bbVarType):
			#var created, subscribe to the ros topic to update the BB SV
			rospy.Subscriber(self.rosTopicName, self.rosMsgType, self.bridgeROSTopicData)

	def bridgeROSTopicData(self, topicData):
		rospy.logdebug('ROS Topic "' + str(self.rosTopicName) + '" updated from ROS. New value "' + str(topicData) + '"')
		#try to parse the ros topic data to a BB SV valid data
		dicSuccess = False
		try:
			bbSharedVarValue = bridge_utils.ROS2BB_TYPE_MAP[self.rosMsgType][1](topicData)
			dicSuccess = True
		except KeyError:
			rospy.logfatal('Error ocurred when updating the Shared Var "' + 
										str(self.bbVarName) + '" from ROS. The "' + str(self.rosMsgType) + 
										'" ROS MSG type doesn\'t have an associate function to parse a value. No value updated.')
		except:
			rospy.logfatal('An unexpected error ocurred when trying to update the Shared Var "' + str(self.bbVarName) + '" from ROS. No value updated.')
		if dicSuccess:
			if BB.WriteSharedVar(self.bbVarType, self.bbVarName, bbSharedVarValue):
				rospy.loginfo('BB shared Var "' + str(self.bbVarName) + '" updated from ROS.')
			else:
				rospy.logwarn('BB Shared Var "' + str(self.bbVarName) + '" not updated from ROS.')

	def createBBSV(self, sharedVarName, sharedVarType):
		#try to create a shared var
		SVcreated = False
		maxAttempts = 3
		for i in range(maxAttempts):
			if BB.CreateSharedVar(sharedVarType, sharedVarName):
				SVcreated = True
				break
		#verify if the SV was created
		if SVcreated:
			rospy.logdebug('Shared var "' + str(sharedVarName) + '" of type "' + str(sharedVarType) + '" created.')
		else:
			rospy.logfatal('Shared var "' + str(sharedVarName) + '" of type "' + str(sharedVarType) + '" cannot be created.')

		return SVcreated

