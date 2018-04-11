#!/bin/bash

pidRec="$(pidof arecord)"  #obtiene el PID del programa
#echo $pidRec
kill $pidRec  #destruye el programa (termina de grabar)
echo "Termine de grabar" 

time=`date "+%Y-%m-%d-%H-%M-%S"`
dest="/home/biorobotica/Documents/RoboCup_Home_Pumas_SAP_$1_$time.wav" 

echo "Cambiando nombre a Question_$1"
mv /home/biorobotica/Documentos/grabacion.wav $dest #cabia el nombre del archivo de salida
