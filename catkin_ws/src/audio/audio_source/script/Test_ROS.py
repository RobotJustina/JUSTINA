#!/usr/bin/env python

#------------------------------------------------------------#
#- Este programa envia un angulo en radianes a otro nodo de -#
#- ROS.                                                     -#
#-                                                          -#
#- El programa espera una bandera de "start" para despues   -#
#- publicar un "mensaje" tipo "srv_Angle".                  -#  
#-                                                          -#
#------------------------------------------------------------#



import rospy
import math

from std_msgs.msg import Bool   #para la bandera de "start"
from std_msgs.msg import Float32MultiArray
from audio_msgs.srv import *
from rospy.numpy_msg import numpy_msg

import threading
from threading import Thread


#------------------------------------------------------------#
#- Esta funcion se emplea para recibir una bandera que      -#
#- inicia el proceso de captura del angulo                  -#
#------------------------------------------------------------#
def callbackStart(data):
	print "Recibi el banderazo de salida"
	#no hace nada, ver que onda


#------------------------------------------------------------#
#- Esta funcion se emplea para envia un angulo de arribo    -#
#- y lo publica.                                            -#
#------------------------------------------------------------#
    
def handle(request):

	theta = 45  #angulo generico

	ang = math.pi*theta/180
	print "Angulo enviado (dec): ", theta
	print "Angulo enviado (rad): ", ang
	return srvAngleResponse(float(ang)) #regresa el angulo




    
#------------------------------------------------------------#
#- Funcion principal, inicia el nodo de ROS "audio_source"  -#
#-                                                          -#
#-                                                          -#
#-                                                          -#
#------------------------------------------------------------#

def main():

	#Inicia el nodo "audio_source"    
	rospy.init_node('audio_source')	
	print "Init Node"


	#Suscribe al nodo al topic: "/audio_source/start" para
	#recibir la bandera de "start"
	rospy.Subscriber("/audio_source/start", Bool, callbackStart)
	print "Init Service"

		
	#Crea un servicio 
	s = rospy.Service('/audio_source/srv_Angle', srvAngle, handle)

	rospy.spin()

    

if __name__ == '__main__':
	main()                #manda a llamar la funcion main()
