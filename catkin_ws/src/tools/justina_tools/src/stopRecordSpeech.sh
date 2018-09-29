#!/bin/bash

pidRec="$(pidof arecord)"  #obtiene el PID del programa
#echo $pidRec
kill $pidRec  #destruye el programa (termina de grabar)
