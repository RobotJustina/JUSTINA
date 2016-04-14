#!/usr/bin/env python
from topics_bridge import *
from  sv_bridge import *
from commands_bridge import *
from services_bridge import *
import rospy
import rosgraph.masterapi
from pyrobotics import BB
from pyrobotics.messages import Command, Response

#Read the list of shared vars from blackboard and convert it as a list of string
def getBBSharedVars():
	#initialize a dictionary and a list to parse and store the variables and its types
	bbVarsDictionary = {}
	sharedVarList = []

	#try to read the shared var 'vars' maxAttempt times 
	maxAttempts = 3
	sharedVarReaded = False
	for currentAttempt in range(maxAttempts):
		commandResponse = BB.SendAndWait(Command('read_var','vars'), 5000)
		if commandResponse and commandResponse.successful:
			#the variable list was read, create a list of var-vartype
			sharedVarList = str(commandResponse.data).split(' ')
			sharedVarReaded = True
			break

	#Parse the variable list, the format of the response is: varType varName varType varName ... 
	if sharedVarReaded:
		#rospy.logdebug('BB SharedVariables List: ' + str(sharedVarList))
		index = 0
		while index < len(sharedVarList) and index+1 < len(sharedVarList):
			#add the varname-vartype relation to the dictionary 
			bbVarsDictionary[sharedVarList[index+1]] = sharedVarList[index]
			index += 2

	#Return the shared vars dictionary
	return bbVarsDictionary

def getROSTopicList():
	rosTopicsDictionary = {}

	#get the topic list from ROS: the format is [[topicName, topicType], [topicName, topicType], .....]
	master = rosgraph.masterapi.Master('/rostopic')
	topicList = master.getPublishedTopics('/')

	try:
		for topic in topicList:
			if topic[0] in rosTopicsDictionary.keys():
				rospy.logerr('The topic : "' + topic[0] + '" is duplicated in ROS Topic list: ' + str(topicList))
			else:
				rosTopicsDictionary[topic[0]] = topic[1]
	except:
		rospy.logfatal('An error ocurred when trying to parse the ROS Topic List: ' + str(topicList))
		
	return rosTopicsDictionary

def bridge_BB2ROS_SharedVars(bbVarsToBridge, rosTopicsDictionary, bb2rosPublishersDictionary):
	#for each listed shared var  to bridge, create a ros topic with the same name
	for varName in bbVarsToBridge.keys():
		rosTopicName = '/' + varName
		if rosTopicName not in rosTopicsDictionary.keys():
			#bridge the SV if it is not in the rostopic list
			bb2rosPublishersDictionary[varName]=BB2RosPublisher(varName, bbVarsToBridge[varName])
		else:
			rospy.logwarn('The ROS Topic "' + rosTopicName + '" corresponding to the BB SV "' + varName + ' already exists. It will not be bridged.')

def bridge_ROS2BB_Topics(rosTopicsToBridge, bbVarsList, ros2bbPublishersDictionary):
	#for each ROS Topic listed create a BB Shared var with the same name
	rosTopicExceptions = ['/rosout', '/rosout_agg']
	#rosTopicExceptions = []
	for topicName in rosTopicsToBridge.keys():	
		bbVarName = topicName.replace("/","")
		#only bridge the topics wich are not in the topic list exceptions, not bridged yet and no belong to the openni camera module
		if topicName not in rosTopicExceptions and bbVarName not in bbVarsList.keys() and '/camera/' not in topicName:
			ros2bbPublishersDictionary[topicName] = ROS2BBPublisher(topicName, rosTopicsToBridge[topicName])
		else:
			rospy.logwarn('The BB SV "' + bbVarName + '" corresponding to the ROS Topic "' + topicName + ' already exists. It will not be bridged.')

def dictionaryDifference(minuend, subtrahend):
	"""
	Compute the difference between  two dictionaries, using the keys of each dictionary.
	Receives:
		minuend: The minuend dictionary.
		subtrahend: The subtrahend dictionary.
	Returns:
		The difference dictionary (minuend-subtrahend)
	"""
	difference = {}
	for varName in list(set(minuend.keys()) - set(subtrahend.keys())):
		difference[varName] = minuend[varName]

	return difference

def dictionaryAddition(dicOperator_1, dicOperator_2):
	"""
	Compute the addition of  two dictionaries, using the keys of each dictionary.
	Store the result on dicOperator_1: dicOperator_1 = dicOperator_1 + dicOperator_2
	Receives:
		dicOperator_1: The first dictionary operator of the adition.
		dicOperator_2: The second dictionary operator of the adition.
	"""
	for key in dicOperator_2.keys():
		if key not in dicOperator_1.keys():
			dicOperator_1[key] = dicOperator_2[key]

def main():
	#define the list of topics to be bridged and its msg types
	callersMap = {
		#'add_two_ints' : BB2ROS_ServiceCallers.add_two_ints_caller,
		'default_server' : BB2ROS_DefaultServicesCallers.default_caller
	}

	#Manage BB connection
	print 'Initializing BB connection'
	BB.Initialize(2080, callersMap)
	BB.Start()
	BB.SetReady(True)

	#Manage ROS connection
	rospy.init_node('bbros_bridge', log_level = rospy.DEBUG)

	#atend calls from ROS nodes to BB commands
	ROS2BB_CommandsCalls('rs_moverobot', Default_ROS_BB_Bridge)
	ROS2BB_CommandsCalls('spg_asay', Default_ROS_BB_Bridge)
	ROS2BB_CommandsCalls('spg_say', Default_ROS_BB_Bridge)
	#commands for PRS-FND Module
	ROS2BB_CommandsCalls('pf_find', Default_ROS_BB_Bridge)
	ROS2BB_CommandsCalls('pf_remember', Default_ROS_BB_Bridge)
	ROS2BB_CommandsCalls('pf_auto', Default_ROS_BB_Bridge)
	ROS2BB_CommandsCalls('pf_sleep', Default_ROS_BB_Bridge)
	ROS2BB_CommandsCalls('pf_shutdown', Default_ROS_BB_Bridge)
	ROS2BB_CommandsCalls('pf_amnesia', Default_ROS_BB_Bridge)
	ROS2BB_CommandsCalls('pf_forget', Default_ROS_BB_Bridge)

	#BRIDGE SHARED VARS AND TOPICS
	#create a dictionary of bridges, for the BB shared vars and for the ROS topics
	bb2rosPublishersDictionary = {}
	ros2bbPublishersDictionary = {}
	#initialize the dictionaries of shared vars and topics already bridged
	bbVarsList = {}
	rosTopicsDictionary = {}

	#verify if new shared vars or ros topic where created
	rate = rospy.Rate(10) # 1hz rate
	while not rospy.is_shutdown():
		#get the BB shared vars list, this list is a dictionary {varName:varType}
		new_bbVarsList = getBBSharedVars()
		#get the ROS published topics list
		new_rosTopicsDictionary = getROSTopicList()

		#verify if there are news bb sv not bridged yet
		bbVarsToBridge = dictionaryDifference(new_bbVarsList, bbVarsList)
		#verify if there are news ros topics not bridged yet
		rosTopicsToBridge = dictionaryDifference(new_rosTopicsDictionary, rosTopicsDictionary)

		if len(bbVarsToBridge) > 0:
			rospy.logdebug('Current BB var-type dictionary: ' + str(bbVarsList))
			rospy.logdebug('New BB var-type dictionary: ' + str(new_bbVarsList))
		if len(rosTopicsToBridge) > 0:
			rospy.logdebug('Current ROS topic-type dictionary: ' + str(rosTopicsDictionary))
			rospy.logdebug('New ROS topic-type dictionary: ' + str(new_rosTopicsDictionary))

		#bridge the bb shared vars to ros topics
		bridge_BB2ROS_SharedVars(bbVarsToBridge, rosTopicsDictionary, bb2rosPublishersDictionary)
		#bridge the ros topics to bb shared vars
		bridge_ROS2BB_Topics(rosTopicsToBridge, bbVarsList, ros2bbPublishersDictionary)

		#update the originals bb sv dictionary and ros topics dictionary
		dictionaryAddition(bbVarsList, bbVarsToBridge)
		dictionaryAddition(rosTopicsDictionary, rosTopicsToBridge)

		rate.sleep()

if __name__ == '__main__':
	main()
