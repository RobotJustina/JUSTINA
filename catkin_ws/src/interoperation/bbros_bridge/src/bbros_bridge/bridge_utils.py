import std_msgs.msg
import bbros_bridge.msg
import rosgraph_msgs.msg
#import beginnner_tutorials.msg
from pyrobotics import BB
from pyrobotics.shared_variables import SharedVarTypes

hri_exists = True
try:
	import hri_msgs.msg
except:
	print "hri not located"
	hri_exists = False

#functions definition to convert shared variables to topic values
def BB_STRING_TO_ROS_STRING(value):
	"""
		Converts a BB String type to a ROS string type.
		Receives: 
			value: the BB string value to convert
		Returns:
			the converted  ROS String value
	"""
	return str(value)
def BB_RECO_TO_ROS_RECO(value):
	#value of type BB RecognizedSpeech contains a list which contains tuples (2 elements each) with string and confidence.
	hypotesisList = []
	confidenceList = []

	for recoTuple in value:
		hypotesisList.append(recoTuple[0])
		confidenceList.append(float(recoTuple[1]))

	return hri_msgs.msg.RecognizedSpeech(hypotesisList, confidenceList)
def BB_STRING_DEFAULT():
	"""
		Returns a BB String default value.
		Returns:
			the BB String default value
	"""
	return str('')

#functions definition to parse topic values to shared variables
def ROS_STRING_TO_BB_STRING(value):
	"""
		Converts a ROS String type to a BB string type.
		Receives: 
			value: the ROS string value to convert
		Returns:
			the converted BB String value
	"""
	return str(value.data)
def ROS_STRING_DEFAULT():
	return str('')

def	ROS_ANY_TO_BB_STRING(anyTypeValue):
	"""
		Converts a ROS value to a BB string type.
		Receives: 
			anyTypeValue: the ROS value to convert
		Returns:
			the converted BB String value
	"""
	return str(anyTypeValue)

#constants for the BB var types
BB_STRING = SharedVarTypes.STRING
BB_RECOSPEECH = SharedVarTypes.RECOGNIZED_SPEECH
#constants for ROS var types
ROS_STRING = std_msgs.msg.String
if hri_exists:
	ROS_RECOSPEECH = hri_msgs.msg.RecognizedSpeech
else:
	ROS_RECOSPEECH = bbros_bridge.msg.RecognizedSpeech
ROS_LOG = rosgraph_msgs.msg.Log
#ROS_NUM = beginnner_tutorials.msg.Num

#dictionary to store the mapping bbVarType:rosVarType
BB2ROS_TYPE_MAP = {
	'string'					:	[BB_STRING, ROS_STRING, BB_STRING_TO_ROS_STRING, BB_STRING_DEFAULT], 
	'RecognizedSpeech':	[BB_RECOSPEECH, ROS_RECOSPEECH, BB_RECO_TO_ROS_RECO, BB_STRING_DEFAULT], 
	'default'					:	[BB_STRING, ROS_STRING, BB_STRING_TO_ROS_STRING, BB_STRING_DEFAULT]
}

#dictionary to store the mapping rosVarType:bbVarType
ROS2BB_TYPE_MAP = {
	ROS_STRING	:	[BB_STRING, ROS_STRING_TO_BB_STRING, ROS_STRING_DEFAULT], 
	ROS_LOG	:	[BB_STRING, ROS_ANY_TO_BB_STRING, ROS_STRING_DEFAULT], 
	#ROS_NUM	:	[BB_STRING, ROS_ANY_TO_BB_STRING, ROS_STRING_DEFAULT], 
	'default'	:	[BB_STRING, ROS_ANY_TO_BB_STRING, ROS_STRING_DEFAULT]
}	
#dictionary to ma the ROS msg type (string representation) to ROS msg valid types
MSGTYPEALIAS_TO_MSGTYPESYSTEM = {
	'std_msgs/String'		:	ROS_STRING, 
	'rosgraph_msgs/Log'	:	ROS_LOG, 
	#'beginnner_tutorials/Num'	:	ROS_NUM, 
	'default'	:	ROS_STRING
}
