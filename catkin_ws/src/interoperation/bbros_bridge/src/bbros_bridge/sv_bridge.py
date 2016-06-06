import bridge_utils
import rospy
from pyrobotics import BB
from pyrobotics.messages import Command, Response
from pyrobotics.shared_variables import SharedVarTypes


class BB2RosPublisher:
	"""Class to bridge the BB Shared Vars to ROS Topics
	"""
	def __init__(self, sharedVarName, bbVarType):
		"""Constructor
		Creates a new BB2ROSPublisher Object.
		Receives:
			shredVarName: Name of the shared var to bridge.
			bbVarType: The type of the var.
		"""
		self.sharedVarName = sharedVarName
		try:
			#get the msg data type for ros publisher and subscriber
			self.bbVarType = bbVarType
			self.rosMsgType = bridge_utils.BB2ROS_TYPE_MAP[self.bbVarType][1]
			#create the corresponding ros publisher to update the ROS topic when updated the BB shared var
			self.rosPublisher = rospy.Publisher(self.sharedVarName, self.rosMsgType, queue_size=1000)
			rospy.logdebug('ROS publisher "' + str(self.rosMsgType) +'" for "' + str(sharedVarName) + '" created.')
		except:
			rospy.logwarn('No ROS msg type associated with "' + str(bbVarType) + '" BB SV type. Using default association.')
			#get the default msg data type for ros publisher and subscriber
			self.bbVarType = 'default'
			self.rosMsgType = bridge_utils.BB2ROS_TYPE_MAP[self.bbVarType][1]
			#create the corresponding ros publisher to update the ROS topic when updated the BB shared var
			self.rosPublisher = rospy.Publisher(self.sharedVarName, self.rosMsgType, queue_size=1000)
			rospy.logdebug('ROS publisher "' + str(self.rosMsgType) +'" for "' + str(sharedVarName) + '" created.')

		#create a subscriber to read the values of the BB shared var
		BB.SubscribeToSharedVar(self.sharedVarName, self.bridge_SV_data)
		#read the value of the shared var from BB and publish it to ROS
		self.pass_var_value()

	def bridge_SV_data(self, sharedVar):
		"""bridge_SV_data
		Callback for the BB shared var update. Read the new value of the BB shared var and and publish it on a ROS topic.
		Receives:
			sharedVar: a BB shared var object.
		"""
		rospy.logdebug('BB Shared Var "' + str(self.sharedVarName) + '" updated from BlackBoard. New value "' + str(sharedVar.data) + '"')
		try:
			rosTopicValue = bridge_utils.BB2ROS_TYPE_MAP[self.bbVarType][2](sharedVar.data)
			self.rosPublisher.publish(rosTopicValue)
			rospy.loginfo('Ros Topic "' + str(self.sharedVarName) + '" updated from BlackBoard.')
		except KeyError:
			rospy.logfatal('Error ocurred when updating the Topic "' + 
										str(self.sharedVarName) + '" from BlackBoard. The "' + str(self.bbVarType) + 
										'" BB SV type doesn\'t have an associate function to parse a value. No value updated.')
		except:
			rospy.logfatal('An unexpected error ocurred when trying to update the Topic "' + str(self.sharedVarName) + '" from BlackBoard. No value updated.')

	def pass_var_value(self):
		"""passVarValue
		Get the value from the BB shared var and copy it to the ROS topic.
		"""
		#try to read the shared var maxAttempt times 
		maxAttempts = 3
		sharedVarReaded = False
		for currentAttempt in range(maxAttempts):
			commandResponse = BB.SendAndWait(Command('read_var',self.sharedVarName), 2000)
			if commandResponse and commandResponse.successful:
				if commandResponse.data != None:
					#the variable was read, publish its value on his corresponding ROS topic
					rosValue = str(commandResponse.data)
					try:
						rosValue = bridge_utils.BB2ROS_TYPE_MAP[self.bbVarType][2](commandResponse.data)
						self.rosPublisher.publish(rosValue)
						rospy.loginfo('BB SharedVariable "' + str(self.sharedVarName) + '" value was published in the "' + str(self.sharedVarName) + '" ROS Topic');
					except KeyError:
						rospy.logfatal('The "' + str(self.bbVarType) + '" BB SV type doesn\'t have an associate function to parse a value. No value published.')
					except:
						rospy.logfatal('An unexpected error ocurrs when trying to parse a "' + str(self.bbVarType) + '" BB SV type. No value published.')
					sharedVarReaded = True
				break

		if not(sharedVarReaded):
			#the variable was not read from BB, publish a default value in its corresponding ROS topic
			try:
				defaultValue = bridge_utils.BB2ROS_TYPE_MAP[self.bbVarType][3]()
				self.rosPublisher.publish(defaultValue)
				rospy.logwarn('BB SharedVariable "' + str(self.sharedVarName) + '" was not read. ROS Topic: "' + str(self.sharedVarName) + '" initialized with default: "' + str(defaultValue + '"'))
			except KeyError:
				rospy.logfatal('No default ROS data defined for "' + str(self.bbVarType) + '" SV type. No data published on "' + str(self.sharedVarName) + '" topic.')
			except:
				rospy.logfatal('An unexpected error ocurred during getting the ROS default value for ' + str(self.bbVarType) + ' SV type. No data published on "' + str(self.sharedVarName) + '" topic.')

		return sharedVarReaded


