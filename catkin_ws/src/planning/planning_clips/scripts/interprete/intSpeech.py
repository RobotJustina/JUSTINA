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
from planning_msgs.srv import *
from planning_msgs.msg import *
import xml.etree.ElementTree as ET
import re
#from ros_pyclips.srv import *

#def cmd_one(c):
#    time.sleep(1)
#    return Response.FromCommandObject(c, True, 'cmd_one response')

#def cmd_two(c):
#    time.sleep(5)
#    return Response.FromCommandObject(c, True, 'cmd_two response')

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
					question = question + 1
			
			if firstparam[0] == 'params' and get_object:
				task_object = firstparam[1]
				get_object = False
				if len(firstparam) < 3:
					temp2 = temp2 + " " + location
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
		q.pushC(s)
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
		
	####split command in task divided for ','
		
	firstRecoSplit = temp1.split(',')
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
	


#fmap = {
#        'cmd_one' : cmd_one,
#        'cmd_two' : (cmd_two, True),
#	'cmd_int': cmd_int,
#	'cmd_task': cmd_task,
#	'cmd_speech': cmd_speech,
#	'cmd_conf':cmd_conf
#        }

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

