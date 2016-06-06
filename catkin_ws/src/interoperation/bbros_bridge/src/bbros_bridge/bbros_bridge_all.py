#!/usr/bin/env python
import sys, json
from optparse import OptionParser
from topics_bridge import *
from  sv_bridge import *
from commands_bridge import *
from services_bridge import *
import rospy
import rosgraph.masterapi
from pyrobotics import BB
from pyrobotics.messages import Command, Response

#Read the list of shared vars from blackboard and stores it into a dictionary with (key, value) = (svName, svType)
#Receives:
#	inclussionList: indicates ehich svs are going to be stored, if the list is empty all the svs will be stored
#Returns
#	bbVarsDictionary: the dictionary in which the svs were stored
def getBBSharedVars(inclussionList):
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
		#exclude, from the sv list, the non included svs in the inclussion ist
		if len(inclussionList) > 0:
			if inclussionList[0] != '*':
				newSharedVarList = [] 
				#sharedVarList = filter(lambda x: x not in inclussionList, sharedVarList)
				index = 0
				while index < len(sharedVarList) - 1:
					if sharedVarList[index+1] in inclussionList:
						newSharedVarList.append(sharedVarList[index])
						newSharedVarList.append(sharedVarList[index+1])
					index += 2
				sharedVarList = newSharedVarList
			index = 0
			while index < len(sharedVarList) and index+1 < len(sharedVarList):
				#add the varname-vartype relation to the dictionary 
				bbVarsDictionary[sharedVarList[index+1]] = sharedVarList[index]
				index += 2

	#Return the shared vars dictionary
	return bbVarsDictionary

#Read the ros topic list from ros and stores it into a dictionary with (key, value) = (topicName, topicMsgType)
#Receives:
#	inclussionList: indicates which topics are going to be stored, if the list is empty all the topics are stored
#Returns
#	rosTopicsDictionary: the dictionary in which the ros topics where stored 
def getROSTopicList(inclussionList):
	rosTopicsDictionary = {}

	#get the topic list from ROS: the format is [[topicName, topicType], [topicName, topicType], .....]
	master = rosgraph.masterapi.Master('/rostopic')
	topicList = master.getPublishedTopics('/')

	#exclude, from the topics list,  the non included topics in the inclussion list
	if len(inclussionList) > 0:
		#* means that all topics will be bridged
		if inclussionList[0] != '*':
			topicList = filter(lambda x: x[0] in inclussionList, topicList)
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
	#rosTopicExceptions = ['/rosout', '/rosout_agg']
	#rosTopicExceptions = []
	for topicName in rosTopicsToBridge.keys():	
		bbVarName = topicName.replace("/","")
		#only bridge the topics wich are not in the topic list exceptions, not bridged yet and no belong to the openni camera module
		#if topicName not in rosTopicExceptions and bbVarName not in bbVarsList.keys() and '/camera/' not in topicName:
		#	ros2bbPublishersDictionary[topicName] = ROS2BBPublisher(topicName, rosTopicsToBridge[topicName])
		#else:
		#	rospy.logwarn('The BB SV "' + bbVarName + '" corresponding to the ROS Topic "' + topicName + ' already exists. It will not be bridged.')
		if bbVarName not in bbVarsList.keys():
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
	bbConnectionPort = 2080;
	topicsInclusionList = ["*"];
	svInclusionList = ["*"];
	srvInclusionList = [];
	cmdInclusionList = [];

	#load the config files
	parser = OptionParser()
	parser.add_option('-f', '--configfile', action='store', type='string', dest='configFile', help='Bridge rules')
	(options, args) = parser.parse_args()

	if options.configFile:
		#parser.error('Configuration File not given')
		#load the configuration options
		with open(options.configFile) as config_file:
			configOptions = json.load(config_file)
		#print configOptions
		try:
			topicsInclusionList = configOptions['ros_topics']
		except:
			topicsInclusionList = []
		try:
			svInclusionList = configOptions['blackboard_sv']
		except:
			svInclusionList = []
		try:
			srvInclusionList = configOptions['ros_services']
		except:
			srvInclusionList = []
		try:
			cmdInclusionList = configOptions['blackboard_cmd']
		except:
			cmdInclusionList = []
		try:
			bbConnectionPort = configOptions["bb_port"]
		except:
			bbConnectionPort = 2080
	
	#define the list of topics to be bridged and its msg types
	callersMap = {
		#'add_two_ints' : BB2ROS_ServiceCallers.add_two_ints_caller,
		'default_server' : BB2ROS_DefaultServicesCallers.default_caller
	}

	#Manage BB connection
	print 'Initializing BB connection'
	BB.Initialize(bbConnectionPort, callersMap)
	BB.Start()
	BB.SetReady(True)

	#Manage ROS connection
	#rospy.init_node('bbros_bridge', log_level = rospy.DEBUG)
	rospy.init_node('bbros_bridge')
	#atend calls from ROS nodes to BB commands
	for bbCommand in cmdInclusionList:
		ROS2BB_CommandsCalls(bbCommand, Default_ROS_BB_Bridge)

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
		new_bbVarsList = getBBSharedVars(svInclusionList)
		#get the ROS published topics list
		new_rosTopicsDictionary = getROSTopicList(topicsInclusionList)

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
