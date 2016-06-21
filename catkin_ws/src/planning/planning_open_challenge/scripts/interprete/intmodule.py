import time
from pyrobotics import BB
from pyrobotics.parallel_senders import ParallelSender
from pyrobotics.messages import Command, Response
import egprs_interpreter
import sys
import classCola

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
		print "push"
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
				args = args + " " + temp[1]
		print args 
		return Response.FromCommandObject(c, True, args )
		#print "verdadero"
	else:
		#print "falso"
		return Response.FromCommandObject(c, False, "No_Task" )
def cmd_int(c):
	#agregamos el comando mediante un archivo, despues usaremos el gpsr
	filename = sys.argv[1]
	with open(filename) as f:
		content = f.read()
		
	interpreted_command = egprs_interpreter.interpret_command(content)

	print " "
	print "Comando Interpretado:"
	print interpreted_command
	separaTask(interpreted_command)
	return Response.FromCommandObject(c, True, "Completed")



fmap = {
        'cmd_one' : cmd_one,
        'cmd_two' : (cmd_two, True),
	'cmd_int': cmd_int,
	'cmd_task': cmd_task
        }


def initialize():
	BB.Initialize(2100, fmap)
  	BB.Start()
	
	BB.CreateSharedVar(BB.SharedVarTypes.STRING, 'actuator')
        BB.WriteSharedVar(BB.SharedVarTypes.STRING, 'actuator', 'initial_value')

	BB.SetReady(True)


def main():
  
	initialize()
	s = ''
    	print 'Waiting for commands ...'
    	while s != 'exit':
		i = 0
  


q = classCola.classCola()
main()

