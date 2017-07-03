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
import os
import time
from random import randint

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
	#Realmente no hago nada muajaja


#------------------------------------------------------------#
#- Esta funcion se emplea para recibir el angulo de arribo  -#
#- a traves del puerto serial y lo convierte a flotante     -#
#- para su publicacion.                                     -#
#------------------------------------------------------------#
    
def handle(request):
	#t = os.system("ls /dev/justinaAudio") #revisa si esta conectada la tarjeta
	ser.write("z") #envia bandera de prueba

	cadena = ser.readline()  #recibimos una linea hasta que el ultimo caracter sea "*"
	cadena2 = 'Estoy Funcionando\n'

	if (cadena == cadena2): #debemos recibir la cadena "Estoy Funcionando"
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
		print "recv= ", theta
	else:
		print "No recibi nada, evio angulo random\n"
		theta = randint(0, 359)  #numero entero aleatorio entre 0 y 359

	theta = float(theta) #convertimos a flotante para que todo lo que hagamos sea legal y bonito
        if theta>180: #para girar a ambos lados
		theta = theta-360.0 


	print "theta_dec= ", theta
#	print theta
	ang = theta*math.pi/180  #convierte la cadena en un flotante y calcula el angulo en radianes
	print "theta_rad= ", ang

#	print "Angulo obtenido ", ang
	return srvAngleResponse(float(ang)) #regresa el angulo




    
#------------------------------------------------------------#
#- Funcion principal, inicia el nodo de ROS "audio_source"  -#
#-                                                          -#
#-                                                          -#
#-                                                          -#
#------------------------------------------------------------#

def main():
	
	global ser


	t = os.system("ls /dev/justinaAudio") #revisa si esta conectada la tarjeta
	while(t != 0):
		print "La tarjeta NO esta conectada"
		time.sleep(2)
		t = os.system("ls /dev/justinaAudio")	



        ser = serial.Serial(
            port='/dev/justinaAudio',  #Puerto a emplear
            baudrate=9600,
            timeout = 1           #lee por 1 segundo
   
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

	rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            rate.sleep()

    

if __name__ == '__main__':
    main() #manda a llamar la funcion main()
