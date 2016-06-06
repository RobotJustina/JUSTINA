import time
from pyrobotics import BB
from pyrobotics.parallel_senders import ParallelSender
from pyrobotics.messages import Command, Response
import egprs_interpreter


def process_string(c):
  interpreted_command = egprs_interpreter.interpret_command(c.params)
  return Response.FromCommandObject(c, True, interpreted_command)


fmap = {
     'cfr_parse' : process_string
}

def ActuatorHandler(sv):
  print '[ACTUATOR] ',
  #print sv.varName
  #print sv.svType
  #if sv.size >= 0:
    #print sv.size
  print sv.data

def initialize():
	BB.Initialize(2100, fmap)
  	BB.Start()
	
	BB.CreateSharedVar(BB.SharedVarTypes.STRING, 'actuator')
        BB.WriteSharedVar(BB.SharedVarTypes.STRING, 'actuator', 'initial_value')
        BB.SubscribeToSharedVar('actuator',ActuatorHandler, subscriptionType='writeothers', reportType='content')

	BB.SetReady(True)


def main():
  
	initialize()
    
	s = ''
	while s != 'exit':
		s = 'A1 motor on left 16:51:21'
		BB.WriteSharedVar(BB.SharedVarTypes.STRING, 'actuator', s)

  

if __name__ == "__main__":
  main()

