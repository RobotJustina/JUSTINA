#!/bin/bash

arecord -t wav -f cd -r 48000 -D hw:1,0 /home/biorobotica/Documentos/grabacion.wav & #inicia a grabar en segundo plano
echo "Grabando"

