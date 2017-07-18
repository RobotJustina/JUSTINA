#!/bin/bash

pidRec="$(pidof arecord)"  #obtiene el PID del programa
#echo $pidRec
kill $pidRec  #destruye el programa (termina de grabar)
echo "Termine de grabar" 

dest="/home/biorobotica/Documents/Pregunta_$1.wav" 

echo "Cambiando nombre a Pregunta_$1"
mv /home/biorobotica/Documents/grabacion.wav $dest #cabia el nombre del archivo de salida
