#!/usr/bin/env python
import time, os
from pyrobotics import BB
from pyrobotics.parallel_senders import ParallelSender
from pyrobotics.messages import Command, Response
import egprs_interpreter
import sys
import classCola
from random import randrange
import rospy
from knowledge_msgs.srv import *
from knowledge_msgs.msg import *
import xml.etree.ElementTree as ET
import re
#from ros_pyclips.srv import *

#def cmd_one(c):
#    time.sleep(1)
#    return Response.FromCommandObject(c, True, 'cmd_one response')

#def cmd_two(c):
#    time.sleep(5)
#    return Response.FromCommandObject(c, True, 'cmd_two response')
def set_mapping(mapping):
    egprs_interpreter.set_mapping(mapping)    

def separaTask(cadena):
	spc = cadena.split("(task")
	spc.remove('')
	s = []
	step = 1
	question = 1
	location = '' 
	task_object = ''
	get_object = False
	handover_object = False
	update_location = False
	find_person = False
	deliver_object = False
	fpush = True
	tu = 2
	tempStep = 1
	no_man_guide = True
	for i in spc:
		temp  = i.split("(")
		temp.remove(' ')
		for j in temp:
			temp2 = j.rstrip(") ")
			firstparam = temp2.split(' ')
			paramTam = len(firstparam)
			
			if temp2 == 'step':
				temp2 = temp2 + " " +  str(step)
				step = step + 1
				if fpush == False:
					temp2 = 'step ' + str(100000000 + tu)
			if paramTam > 1:
				if firstparam[1] == 'get_object':
					get_object = True
				elif firstparam[1] == 'update_object_location':
					update_location = True
				elif firstparam[1] == 'handover_object':
					 handover_object = True
				elif firstparam[1] == 'find_person_in_room':
					find_person = True
				elif firstparam[1] == 'deliver_in_position':
					deliver_object = True
				elif firstparam[1] == 'question':
					temp2 = firstparam[0] + " " + "question_" + str(question) + " " + firstparam[2]
					if paramTam > 3:
						temp2 = temp2 + " " + firstparam[3]
					question = question + 1
			if paramTam > 2:
				if firstparam[2] == 'place_destiny':
					if step == tempStep + tu + 1:
						step = step - 1
						fpush = False
						temp2 = 'params'
						#print "TEST DESTINY " + str(step)
					else:
						no_man_guide = False
						tempStep = step
						tu = 2
						#print "TEST TEMPSTEP " + str(tempStep)
				elif firstparam[2] == 'follow_place_origin':
					if step == tempStep + tu:
						step = step -1
						fpush = False
						tu = tu - 1
						temp2  = 'params'
						#print "TEST ORIGIN " + str(step)
			
			if firstparam[0] == 'params' and get_object:
				task_object = firstparam[1]
				get_object = False
				if len(firstparam) < 3:
					if no_man_guide:
						temp2 = temp2 + " " + location
					else:
						no_man_guide = True
			elif firstparam[0] == 'params' and update_location:
				location = firstparam[2]
				update_location = False
			elif firstparam[0] == 'params' and handover_object:
				temp2 = temp2 + " " + task_object
				task_object = ''
				handover_object = False
			elif firstparam[0] == 'params' and find_person:
				if len(firstparam) < 3:
					temp2 = temp2 + " " + location
					find_person = False
				else:
					find_person = False
			elif firstparam[0] == 'params' and deliver_object:
				temp2 = firstparam[0] + " " + task_object + " " + firstparam[1]
				deliver_object = False
				
			s.append(temp2)
			print "PUSH: " + temp2
		if fpush:
			q.pushC(s)
		fpush = True
		s = []

def cmd_task(c):
	args = ''
	if q.es_vacia() == False:
		res = q.popC()
		for i in res:
			temp = i.split(" ")
			if len(temp) == 1:
				args = args + " " + "person"
			if len(temp)>1:
				for j in range(1,len(temp)):
					args = args + " " + temp[j]
		print args 
		#return Response.FromCommandObject(c, True, args )
		return (1, args)
	else:
		#return Response.FromCommandObject(c, False, "No_Task" )
		return (0, "No_Task")

def cmd_int(c):
	try:
		cadena  = cmdQ.popC()
	except:
		return (0, "Cola_Vacia")
			
	cadena2 = str(cadena)
	cadena2 = re.sub('"',"'",cadena2)
	try:
		content = cadena2.split("',")
	except:
		print 'Error the Command format is incorrect'
		args = 'No_Interpretation'
		#return Response.FromCommandObject(c, False, args)
		return (0, args)
	
	temp = content[0]
	temp1 = temp.lstrip("[('")
	
	##### you(may | can | will) find
	f = temp1.find("you may find")
	f1 = temp1.find("you can find")
	f2 = temp1.find("you will find")
		
	####split command in task divided for ','
	if f == -1 and f1 == -1 and f2 == -1 : 	
		firstRecoSplit = temp1.split(',')
	else:
		firstRecoSplit = [temp1];
	if len(firstRecoSplit) < 2:
		lastRecoSplit = temp1.split(' and ')
	else:
		lastRecoSplit = firstRecoSplit
	result = ''	
	
	for j in lastRecoSplit:
		interpreted_command = ''
		interpreted_command = egprs_interpreter.interpret_command(j)
		ic = str(interpreted_command)
		result = result + ic
	
	if temp1 == 'robot yes' or temp1 == 'robot no':
		result = 'False'

	print " "
	print "Comando Interpretado:"
	print result
	
	try:
		cabecera = result.split(' ')
	except:
		print 'Error the Interpreted Command format is incorrect'
		args = 'No_Interpretation'
		#return Response.FromCommandObject(c, False, args)
		return (0, args)
	
	if result == 'False' or cabecera[0] == '(task_to' or result == 'FalseFalse' or result == 'FalseFalseFalse' or result == 'FalseFalseFalseFalse':
		args = 'No_Interpretation'
		#return Response.FromCommandObject(c, False, args)
		return (0, args)
	else:
		q.empty()
		separaTask(result)
		args = temp1.replace(' ','_')
		#return Response.FromCommandObject(c, True, args)
		return (1, args)

def cmd_str_int(c):
        interpreted_command = ''
	interpreted_command = egprs_interpreter.interpret_command(c)
	print " "
	print "Comando Interpretado:"
	print interpreted_command
	
	try:
		cabecera = interpreted_command.split(' ')
	except:
		print 'Error the Interpreted Command format is incorrect'
		args = 'No_Interpretation'
		return (0, args)
	
	if cabecera[0] == '(task_to' :
		args = 'No_Interpretation'
		return (0, args)
	else:
		return (1, interpreted_command)
        return (1, interpreted_command)

def cmd_speech(c):
	print 'ReadyForReciveCommands'
	cmdQR.empty()
	cmdHQ.empty()
	print 'Give a COMMAND'
	while cmdQR.es_vacia():
		j = 0
		time.sleep(3)
	if cmdHQ.es_vacia() == False:
		temp  = cmdHQ.popPile()
		cmdQ.pushC(temp)	
		#return Response.FromCommandObject(c, True, "ReadyToDevelopCommands")
		return (1, 'ReadyToDevelopCommands')
	else:
		if cmdQR.es_vacia()== False:
			temp  = cmdQR.popC()
			cmdQ.pushC(temp)
			#return Response.FromCommandObject(c, True, "ReadyToDevelopCommands")
			return (1, 'ReadyToDevelopCommands')
		else:
			#return Response.FromCommandObject(c, False, "ReadyForReciveCommands")
			return (0, 'ReadyToDevelopCommands')


def cmd_conf(c):
	print ' '
	print 'Request CONFIRMATION '# + c.params
	spg = 'Do you want I make the command ' + c.params
	spg_say = spg.replace('_', ' ')
	print 'Sending command say...' + spg_say

	##This is for service of Ramon
	#rospy.wait_for_service('/simple_task_planner/wait_for_confirm')
	#srv_client = rospy.ServiceProxy('/simple_task_planner/wait_for_confirm', wait_for_confirm)
	#rep_sent = RepeatedSentence(spg_say, 30000)
	#response = srv_client(rep_sent, 50000)

	#if(response.confirmation_received):
	#	steps = q.lenC()
	#	plan_name_id = randrange(10000)
	#	args = "plan-" + str(plan_name_id) + " " + str(steps)
	#	print '<-------------->'
	#	print args
	#	print '<-------------->'
	#	#return Response.FromCommandObject(c, True, args)
	#	return (1, args)
	#else:
	#	q.empty()
	#	args = 'confirmation_no'
	#	#return Response.FromCommandObject(c, False, args)
	#	return (0, args)

	#print BB.SendAndWait(Command('say', spg_say), 5000, 1)
	
	# START OF CODE FOR THE CONFIRMATION WITH THE REAL ROBOT
	print 'cmdQR'
	cmdQR.empty()
	print 'cmdHQ'
	cmdHQ.empty()
	print 'Ingresar un comando Test'
	while cmdQR.es_vacia():
		j = 0
	if cmdHQ.es_vacia() == False:
		temp  = cmdHQ.popPile()
		print 'temp'
		print temp
	else:
		if cmdQR.es_vacia()== False:
			temp  = cmdQR.popC()
			print 'temp'
			print temp
		else:
			print 'False confirmation'
			return (0, args)
		
	cadena2 = str(temp)
	content = cadena2.split("',")
	temp2 = content[0]
	temp1 = temp2.lstrip("[('")
	print 'Resp ' + temp1
	
	if temp1 == 'robot yes':
		steps = q.lenC()
		plan_name_id = randrange(10000)
		args = "plan-" + str(plan_name_id) + " " + str(steps)
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif temp1 == 'robot no':
		q.empty()
		args = 'confirmation_no'
		return (0, args)
	else:
		q.empty()
		args = 'confirmation_no'
		return (0, args)

	# END OF CODE FOR THE CONFIRMATION WITH THE REAL ROBOT
	
	# START OF CODE FOR CONFIRMATION MANUAL
	#n = raw_input('[Y/N]: ')
	#if n == 'y' or n == 'Y':
	#	steps = q.lenC()
	#	plan_name_id = randrange(10000)
	#	args = "plan-" + str(plan_name_id) + " " + str(steps)
	#	print '<-------------->'
	#	print args
	#	print '<-------------->'
	#	#return Response.FromCommandObject(c, True, args)
	#	return (1, args)
	#else:	
	#	q.empty()
	#	args = 'confirmation_no'
	#	#return Response.FromCommandObject(c, False, args)
	#	return (0, args)

def cmd_ask_name(c):
	print 'REQUEST ASK NAME TO PERSON: '
	print 'cmdQR'
	cmdQR.empty()
	print 'cmdHQ'
	cmdHQ.empty()
	while cmdQR.es_vacia():
		j = 0
	if cmdHQ.es_vacia() == False:
		temp  = cmdHQ.popPile()
		print 'temp'
		print temp
	else:
		if cmdQR.es_vacia()== False:
			temp  = cmdQR.popC()
			print 'temp'
			print temp
		else:
			print 'False confirmation'
			args = 'false_confirmation'
			return (0, args)
		
	cadena2 = str(temp)
	content = cadena2.split("',")
	temp2 = content[0]
	temp1 = temp2.lstrip("[('")
	print 'Resp ' + temp1
	question = temp1.lower()
	
	if question == 'my name is emma':
		args = 'emma'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'my name is olivia':
		args = 'olivia'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'my name is sophia':
		args = 'sophia'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'my name is ava':
		args = 'ava'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)

	elif question == 'my name is isabella':
		args = 'isabella'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'my name is mia':
		args = 'mia'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)

	elif question == 'my name is abigail':
		args = 'abigail'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)

	elif question == 'my name is emily':
		args = 'emily'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)

	elif question  == 'my name is charlotte':
		args = 'charlotte'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)

	elif question  == 'my name is harper':
		args = 'harper'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)

	elif question  == 'my name is noah':
		args = 'noah'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)

	elif question  == 'my name is liam':
		args = 'liam'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)

	elif question  == 'my name is mason':
		args = 'mason'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	
	elif question  == 'my name is jacob':
		args = 'jacob'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
		
	elif question == 'my name is william':
		#q.empty()
		args = 'william'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'my name is ethan':
		#q.empty()
		args = 'ethan'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'my name is james':
		#q.empty()
		args = 'james'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'my name is alexander':
		#q.empty()
		args = 'alexander'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'my name is michael':
		#q.empty()
		args = 'michael'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'my name is benjamin':
		#q.empty()
		args = 'benjamin'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	
	else:
		#q.empty()
		args = 'ask_name_no'
		return (0, args)

def cmd_ask_incomplete(c):
	print 'REQUEST ASK FOR PLACE: '
	print 'cmdQR'
	cmdQR.empty()
	print 'cmdHQ'
	cmdHQ.empty()
	while cmdQR.es_vacia():
		j = 0
	if cmdHQ.es_vacia() == False:
		temp  = cmdHQ.popPile()
		print 'temp'
		print temp
	else:
		if cmdQR.es_vacia()== False:
			temp  = cmdQR.popC()
			print 'temp'
			print temp
		else:
			print 'False confirmation'
			args = 'false_confirmation'
			return (0, args)
		
	cadena2 = str(temp)
	content = cadena2.split("',")
	temp2 = content[0]
	temp1 = temp2.lstrip("[('")
	print 'Resp ' + temp1
	question = temp1.lower()
	
	if question == 'at the desk':
		args = 'desk'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'at the kitchen table':
		args = 'kitchen_table'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'at the teepee':
		args = 'teepee'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'at the bed':
		args = 'bed'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'at the bookcase':
		args = 'bookcase'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'at the sofa':
		args = 'sofa'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'at the TV':
		args = 'tv'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'at the left rack':
		args = 'left_rack'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'at the right rack':
		args = 'right_rack'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'at the coffee table':
		args = 'coffee_table'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'at the sideboard':
		args = 'sideboard'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'at the kitchen shelf':
		args = 'kitchen shelf'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'at the bistro table':
		args = 'bistro_table'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'at the left planks':
		args = 'left_planks'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'at the right planks':
		args = 'right_planks'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'at the balcony shelf':
		args = 'balcony_shelf'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'at the kitchen counter':
		args = 'kitchen_counter'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'at the fridge':
		args = 'fridge'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'at the kitchen rack':
		args = 'kitchen_rack'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'at the entrance shelf':
		args = 'entrance_shelf'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'at the little desk':
		args = 'little_desk'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'at the bedroom':
		args = 'bedroom'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'at the living room':
		args = 'living_room'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'at the entrance':
		args = 'entrance'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'at the kitchen':
		args = 'kitchen'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'at the corridor':
		args = 'corridor'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'at the balcony':
		args = 'balcony'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'i want chopsticks':
		args = 'chopsticks'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'i want apple':
		args = 'apple'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'i want orange':
		args = 'orange'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'i want corn':
		args = 'corn'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'i want onion':
		args = 'onion'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'i want radish':
		args = 'radish'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'i want bowl':
		args = 'bowl'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'i want soup container':
		args = 'soup_container'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'i want asience':
		args = 'asience'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'i want hair spray':
		args = 'hair_spray'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'i want moisturizer':
		args = 'moisturizer'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'i want shampoo':
		args = 'shampoo'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'i want fork':
		args = 'fork'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'i want spoon':
		args = 'spoon'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'i want aquarius':
		args = 'aquarius'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'i want coke':
		args = 'coke'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'i want cold brew':
		args = 'cold_brew'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'i want green tea':
		args = 'green_tea'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'i want bread':
		args = 'bread'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'i want candy':
		args = 'candy'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'i want chewing gum':
		args = 'chewing_gum'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'i want cup star':
		args = 'cup star'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'cup star':
		args = 'cup star'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'curry':
		args = 'curry'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'i want fries':
		args = 'fries'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'i want jelly':
		args = 'jelly'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	
	else:
		#q.empty()
		args = 'ask_place_no'
		return (0, args)


def mySubscriptionHandler(sv):
	#if sv.writer == 'GPSR-CMD-GEN':
	print '----------------------------'
	print 'Shared Variable updated:'
	print sv.varName
	print sv.svType
	print sv.writer
	if sv.size >= 0:
			print sv.size
	cmdQR.pushC(sv.data)
	print sv.data
	

def myHandlerHypothesis(sv):
	if sv.writer == 'SP-REC':
		print '----------------------------'
		print 'Shared Variable updated:'
		print sv.varName
		print sv.svType
		print sv.writer
		if sv.size >= 0:
				print sv.size
		print sv.data
		cmdHQ.pushC(sv.data)


def subsRecSpeech(sv):
    print '----------------------------'
    print 'Shared Variable updated:'
    print 'RecognizedSpeech'
    cmdQR.pushC(sv)

def subsHypSpeech(sv):
    print '----------------------------'
    print 'Shared Variable updated:'
    print 'HypothesizedSpeech'
    cmdQR.pushC(sv)

def answer(sv):
	print '----- in answer a question ----- '
	cmdQR.empty()
	cmdHQ.empty()
	print 'Waiting for instruction from user'
	while cmdQR.es_vacia():
		j = 0
		time.sleep(3)
	if cmdHQ.es_vacia() == False:
		temp  = cmdHQ.popPile()
		cmdQ.pushC(temp)	
		#return Response.FromCommandObject(c, True, "ReadyToDevelopCommands")
		return (1, 'ReadyToDevelopCommands')
	else:
		if cmdQR.es_vacia()== False:
			temp  = cmdQR.popC()
			cmdQ.pushC(temp)
			#return Response.FromCommandObject(c, True, "ReadyToDevelopCommands")
			try:
				cadena  = cmdQ.popC()
			except:
				return (0, "Cola_Vacia")
					
			cadena2 = str(cadena)
			try:
				content = cadena2.split("',")
			except:
				print 'Error the Command format is incorrect'
				args = 'No_Interpretation'
				#return Response.FromCommandObject(c, False, args)
				return (0, args)
			
			temp = content[0]
			temp1 = temp.lstrip("[('")
			filePath = os.path.dirname(os.path.abspath(__file__))
    
    			file_qst = filePath + '/Questions.xml'

			tree = ET.parse(file_qst)
			root = tree.getroot()
			i=0
			for child in root:
				if root[i][0].text == temp1:
					print root[i][1].text
					return (1, root[i][1].text)
				i += 1

			return (0, 'ActionNoComplete')
		else:
			#return Response.FromCommandObject(c, False, "ReadyForReciveCommands")
			return (0, 'ActionNoComplete')	


q = classCola.classCola() #cola de tareas
cmdQR = classCola.classCola() # cola de comandos
cmdQ = classCola.classCola()
cmdHQ =  classCola.classCola()

