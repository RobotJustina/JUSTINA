import time
from pyrobotics import BB
from pyrobotics.parallel_senders import ParallelSender
from pyrobotics.messages import Command, Response
import egprs_interpreter
import sys
import classCola
from random import randrange

def cmd_one(c):
    time.sleep(1)
    return Response.FromCommandObject(c, True, 'cmd_one response')

def cmd_two(c):
    time.sleep(5)
    return Response.FromCommandObject(c, True, 'cmd_two response')

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
		return Response.FromCommandObject(c, True, args )
		#print "verdadero"
	else:
		#print "falso"
		return Response.FromCommandObject(c, False, "No_Task" )

def cmd_int(c):
	
	#cadena  = cmdQ.popC()	
	#cadena2 = str(cadena)
	#try:
	#	content = cadena2.split("',")
	#except:
	#	print 'Error the Command format is incorrect'
	#	args = 'No_Interpretation'
	#	return Response.FromCommandObject(c, False, args)
	#temp = content[0]
	#temp1 = temp.lstrip("[('")

	#agregamos el comando mediante un archivo, despues usaremos el gpsr
	filename = sys.argv[1]
	with open(filename) as f:
		content = f.read()

	interpreted_command = egprs_interpreter.interpret_command(content)

	print " "
	print "Comando Interpretado:"
	print interpreted_command
	
	try:
		cabecera = interpreted_command.split(' ')
	except:
		print 'Error the Interpreted Command format is incorrect'
		args = 'No_Interpretation'
		return Response.FromCommandObject(c, False, args)
	
	if interpreted_command == 'False' or cabecera[0] == '(task_to' :
		args = 'No_Interpretation'
		return Response.FromCommandObject(c, False, args)
	else:
		q.empty()
		separaTask(interpreted_command)
		args = temp1.replace(' ','_')
		#steps = q.lenC()
		#plan_name_id = randrange(10000)
		#args = "plan-" + str(plan_name_id) + " " + str(steps)
		#print '<-------------->'
		#print args
		#print '<-------------->'
		return Response.FromCommandObject(c, True, args)


def cmd_speech(c):
	print 'ReadyForReciveCommands'
	cmdQR.empty()
	cmdHQ.empty()
	print 'Give a COMMAND'
	return Response.FromCommandObject(c, True, "ReadyToDevelopCommands") 
	#while cmdQR.es_vacia():
	#	j = 0
	#	time.sleep(3)
	#if cmdHQ.es_vacia() == False:
	#	temp  = cmdHQ.popPile()
	#	cmdQ.pushC(temp)	
	#	return Response.FromCommandObject(c, True, "ReadyToDevelopCommands") 
	#else:
	#	if cmdQR.es_vacia()== False:
	#		temp  = cmdQR.popC()
	#		cmdQ.pushC(temp)
	#		return Response.FromCommandObject(c, True, "ReadyToDevelopCommands")
	#	else:
	#		return Response.FromCommandObject(c, False, "ReadyForReciveCommands")


def cmd_conf(c):
	print ' '
	print 'Request CONFIRMATION '# + c.params
	spg = 'Do you want I make the command ' + c.params
	spg_say = spg.replace('_', ' ')
	print 'Sending command say...' + spg_say
	#print BB.SendAndWait(Command('say', spg_say), 5000, 1)
	
	#print 'cmdQR'
	#cmdQR.empty()
	#print 'cmdHQ'
	#cmdHQ.empty()
	#print 'Ingresar un comando Test'
	#while cmdQR.es_vacia():
	#	j = 0
	#if cmdHQ.es_vacia() == False:
	#	temp  = cmdHQ.popPile()
		#print 'temp'
		#print temp
	#else:
	#	if cmdQR.es_vacia()== False:
	#		temp  = cmdQR.popC()
			#print 'temp'
			#print temp
	#	else:
	#		print 'False confirmation'
	#		return Response.FromCommandObject(c, False, args)

		
	#cadena2 = str(temp)
	#content = cadena2.split("',")
	#temp2 = content[0]
	#temp1 = temp2.lstrip("[('")
	#print 'Resp ' + temp1
	
	#if temp1 == 'robot yes':
	#	steps = q.lenC()
	#	plan_name_id = randrange(10000)
	#	args = "plan-" + str(plan_name_id) + " " + str(steps)
	#	print '<-------------->'
	#	print args
	#	print '<-------------->'
	#	return Response.FromCommandObject(c, True, args)
	#elif temp1 == 'robot no':
	#	q.empty()
	#	args = 'confirmation_no'
	#	return Response.FromCommandObject(c, False, args)
	#else:
	#	q.empty()
	#	args = 'confirmation_no'
	#	return Response.FromCommandObject(c, False, args)
	
	n = raw_input('[Y/N]: ')
	if n == 'y' or n == 'Y':
		steps = q.lenC()
		plan_name_id = randrange(10000)
		args = "plan-" + str(plan_name_id) + " " + str(steps)
		print '<-------------->'
		print args
		print '<-------------->'
		return Response.FromCommandObject(c, True, args)
	else:	
		q.empty()
		args = 'confirmation_no'
		return Response.FromCommandObject(c, False, args)
	


fmap = {
        'cmd_one' : cmd_one,
        'cmd_two' : (cmd_two, True),
	'cmd_int': cmd_int,
	'cmd_task': cmd_task,
	'cmd_speech': cmd_speech,
	'cmd_conf':cmd_conf
        }

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


def initialize():
	
	BB.Initialize(2100, fmap)
  	BB.Start()
	BB.SetReady(True)
	
	BB.SubscribeToSharedVar('recognizedSpeech',mySubscriptionHandler)
	BB.SubscribeToSharedVar('hypothesizedSpeech',myHandlerHypothesis)

def main():
  
	initialize()
	s = ''
    	print 'Waiting for commands ...'
    	while s != 'exit':
		i = 0
		time.sleep(3)
  


q = classCola.classCola() #cola de tareas
cmdQR = classCola.classCola() # cola de comandos
cmdQ = classCola.classCola()
cmdHQ =  classCola.classCola()
flagSpeech = False #bandera para saber que se ha recivido un comando

#queda pendiente revisar el uso de esta bandera
flagPlannerBusy =  True #bandera el planeador esta desglosando el plan,
main()

