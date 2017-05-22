#!/usr/bin/env python
import rospy
import json
import std_msgs.msg
from planning_msgs.srv import *
from planning_msgs.msg import *
import time
from pyrobotics import BB
from pyrobotics.parallel_senders import ParallelSender
from pyrobotics.messages import Command, Response
import egprs_interpreter


def process_string(request):
	#get the recognized string and send it to the reco function
	interpreted_command = egprs_interpreter.interpret_command(request.sentence)
	#convert the interpretation json-format to lists 
	jsonCFR = json.loads(interpreted_command)
	#send the response back
	response = parse_sentence_cfrResponse()
	response.cfr.command = jsonCFR["action"]
	for key in jsonCFR["params"].keys():
		response.cfr.params.append(CFRParams(key, jsonCFR["params"][key]))
		#response.cfr.params.frame_value.append(jsonCFR["params"][key])
	return response

def main():
	rospy.init_node('language_understanding')
 	#f = open('interpretationResult', 'w')
	#count = 0
	#for sentence in sentences:
	#	f.write("\n----------------------------------------------------\n")
	#	f.write(str(("Sentence to interpret: ", sentence)))
	#	interpResult = egprs_interpreter.interpret_command(sentence)
	#	if interpResult != "NO_INTERPRETATION":
	#		count=count+1
	#	f.write(str("\n" + interpResult))
	#f.closed

	#print "Total interpreted: ", count

	#advertise a service to parsing
	rospy.Service('language_understanding/parse_sentence_cfr', parse_sentence_cfr, process_string)

	rospy.spin()

if __name__ == "__main__":
	main()
