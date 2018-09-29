#!/bin/bash
time=`date "+%Y-%m-%d-%H-%M-%S"`
arecord -t wav -f cd -r 48000 -D hw:1,0 $1_$time & #inicia a grabar en segundo plano
echo "Grabando"

