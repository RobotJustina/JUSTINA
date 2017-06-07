#!/usr/bin/env python


#------------------------------------------------------------#
#- Este programa se comunica con el DSP TMS320F28377s       -#
#- para recibir el angulo de arribo de una senal de voz     -#
#- para esto abre un puerto serial y convierte el angulo    -#
#- recibido de grados a radianes para enviarlo a otro nodo  -#
#- de ROS.                                                  -#
#-                                                          -#
#- El programa espera una bandera de "start" para despues   -#
#- solicitar al DSP el angulo y publica un "mensaje"        -#
#- tipo "srv_Angle".                                        -#  
#-                                                          -#
#- Puerto Serial:                                           -#
#-   nombre: /dev/ttyUSB0 (puede variar)                    -#
#-   BaudRate: 9600                                         -#
#-   No. Bits: 8                                            -#
#-   Paridad: No                                            -#
#                                                           -#
#------------------------------------------------------------#

import rospy

from std_msgs.msg import Bool   #para la bandera de "start"
from std_msgs.msg import Float32MultiArray
from audio_msgs.srv import *
from rospy.numpy_msg import numpy_msg

import threading
from threading import Thread

import serial
import math


#------------------------------------------------------------#
#- Esta funcion se emplea para recibir una bandera que      -#
#- inicia el proceso de captura del angulo                  -#
#------------------------------------------------------------#
def callbackStart(data):
	ser.write("a") #mensaje a enviar para iniciar a procesar
        print "Envie bandera"
	#no hace nada, ver que onda


#------------------------------------------------------------#
#- Esta funcion se emplea para recibir el angulo de arribo  -#
#- a traves del puerto serial y lo convierte a flotante     -#
#- para su publicacion.                                     -#
#------------------------------------------------------------#
    
def handle(request):

	x = {}       #arreglo vacio sin tamano
	i = 0;       #contador de caracteres validos

        print "Recibiendo angulo..."
	ser.write("b") #mensaje a enviar para recibir el angulo
	rx = ser.read();  #recibimos un carater

	while (rx != '*'):        #mientras el caracter sea diferente de "*"	
		x[i] = rx        #agrega el caracter al arreglo
		rx = ser.read()   #recibe el siguiente caracter
		i+=1              #incrementa el contador

	#concatena los caracteres recibidos en un string
	theta = x[0]
	for i in range(1,len(x)):
		theta  = theta+x[i]

	print "theta_dec="
	print theta
	ang = float(theta)*math.pi/180  #convierte la cadena en un flotante y calcula el angulo en radianes
	print "theta_rad="

	print "Angulo obtenido ", ang
	return srvAngleResponse(float(ang)) #regresa el angulo




    
#------------------------------------------------------------#
#- Funcion principal, inicia el nodo de ROS "audio_source"  -#
#-                                                          -#
#-                                                          -#
#-                                                          -#
#------------------------------------------------------------#

def main():
	
	global ser


        ser = serial.Serial(
            port='/dev/ttyUSB0',  #Puerto a emplear
            baudrate=9600         
        )

	ser.isOpen()

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
