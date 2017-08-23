#!/usr/bin/env python
import time
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
	for i in spc:
		temp  = i.split("(")
		temp.remove(' ')
		for j in temp:
			temp2 = j.rstrip(") ")
			s.append(temp2)
		#print "push"
		q.pushC(s)
		planQ.pushC(s)
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

def cmd_explain(c):
	args = ''
	if planQ.es_vacia() == False:
		res = planQ.popC()
		for i in res:
			temp = i.split(" ")
			if len(temp) == 1:
				args = args + " " + "person"
			if len(temp)>1:
				for j in range(1,len(temp)):
					args = args + " " + temp[j]
		print args 
		return (1, args)
	else:
		return (0, "No_Tasksssss")

def cmd_int(c):
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
	if temp1 == 'explain the plan' or temp1 == 'where is the sugar' or temp1 == 'where is the milk' or temp1 == 'where is the juice' or temp1 == 'where is the soup' or temp1 == 'what happen with the soup' or temp1 == 'what happen with the sugar' or temp1 == 'what happen with the milk' or temp1 == 'what happen with the juice' or temp1 == 'who has the milk' or temp1 == 'who has the juice' or temp1 == 'who has the soup' or temp1 == 'who has the sugar':
		print 'No Interpretation of explain the plan'
		args = 'No_Interpretation'
		return (0, args)
	else:
		interpreted_command = egprs_interpreter.interpret_command(temp1)

	print " "
	print "Comando Interpretado:"
	print interpreted_command
	
	try:
		cabecera = interpreted_command.split(' ')
	except:
		print 'Error the Interpreted Command format is incorrect'
		args = 'No_Interpretation'
		#return Response.FromCommandObject(c, False, args)
		return (0, args)
	
	if interpreted_command == 'False' or cabecera[0] == '(task_to' :
		args = 'No_Interpretation'
		#return Response.FromCommandObject(c, False, args)
		return (0, args)
	else:
		q.empty()
		separaTask(interpreted_command)
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
			args = 'false_confirmation'
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
		planQ.empty()
		args = 'confirmation_no'
		return (0, args)
	else:
		q.empty()
		planQ.empty()
		args = 'confirmation_no'
		return (0, args)

def cmd_disp(c):
	print 'NO DISPONIBLE'
	q.empty()
	planQ.empty()
	return (0, "empty")

def cmd_world(c):
	print ' '
	print 'Request WHAT YOU SEE '# + c.params
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
	
	if question == 'how many people you can see':
		args = 'what_see_person'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'find objects on the table':
		args = 'what_see_obj'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'describe the world':
		args = 'describe_world'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'could you take my order':
		args = 'take_order'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)

	elif question == 'where is the sugar' or question == 'who has the sugar':
		args = 'sugar'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	elif question == 'where is the milk' or question == 'who has the milk':
		args = 'milk'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)

	elif question == 'where is the juice' or question == 'who has the juice':
		args = 'juice'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)

	elif question == 'where is the soup' or question == 'who has the soup':
		args = 'soup'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)

	elif question  == 'explain the plan':
		args = 'explain'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)

	elif question  == 'verify please':
		args = 'verify'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)

	elif question  == 'what happen with the soup':
		args = 'happen soup'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)

	elif question  == 'what happen with the sugar':
		args = 'happen sugar'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)

	elif question  == 'what happen with the milk':
		args = 'happen milk'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
	
	elif question  == 'what happen with the juice':
		args = 'happen juice'
		print '<-------------->'
		print args
		print '<-------------->'
		return (1, args)
		
	elif question == 'robot no':
		#q.empty()
		args = 'what_see_no'
		return (0, args)
	
	elif question == 'robot yes':
		args = 'execute'
		return (1, args)
	else:
		#q.empty()
		args = 'what_see_no'
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

			if temp1 == 'would you like to drive your own car':
				return (1, 'i dont have a drivers licence')
			if temp1 == 'where do you come from':
				return (1, 'i come from Puebla')
			if temp1 == 'what is your favorite science fiction novel':
				return (1, 'my favorite sience fiction novel is do androids dreaming of electricship')
			if temp1 == 'how tall are you':
				return (1, 'my heigth is one point forty five meters')
			if temp1 == 'what is your favarite movie':
				return (1, 'my favorite movie is blade runner')
			if temp1 == 'are you feeling warm in this room':
				return (1, 'yes very much')

			if temp1 == 'how do you like your coffee':
				return (1, 'i do not drink coffee')
			if temp1 == 'who is the best soccer player ever':
				return (1, 'the best soccer player is Pele')
			if temp1 == 'did you fancy the other robot':
				return (1, 'i am not ready for a commitment')

			if temp1 == 'how many rooms does this arena have':
				return (1, 'this arena has four rooms')
			if temp1 == 'when did you have your last maintanance':
				return (1, 'my last mantanaince was this morning')
			if temp1 == 'how are you doing today':
				return (1, 'Fine thank you')
			if temp1 == 'who discovered America':
				return (1, 'America was discovered by Christopher Colombus')
			if temp1 == 'what is the color of the sky':
				return (1, 'the color of the sky is blue')
			if temp1 == 'what is the capital of this state':
				return (1, 'The capital of this state is Victoria city')
			else:
				return (0, 'ActionNoComplete')
		else:
			#return Response.FromCommandObject(c, False, "ReadyForReciveCommands")
			return (0, 'ActionNoComplete')	


q = classCola.classCola() #cola de tareas
planQ = classCola.classCola() # cola que almacena las tareas y sirve para explicar el plan
cmdQR = classCola.classCola() # cola de comandos
cmdQ = classCola.classCola()
cmdHQ =  classCola.classCola()

