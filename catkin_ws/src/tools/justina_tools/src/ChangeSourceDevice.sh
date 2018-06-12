#!/bin/bash
#FORMAT
FRM='\033['
BLACK='0;30'
RED='1;31'
GREEN='1;32'
YELLOW='1;33'
BLUE='1;34'
PURPLE='1;35'
CYAN='1;36'
WHITE='1;37'
BGBLACK=';40m'
BGRED=';41m'
BGGREEN=';42m'
BGYELLOW=';43m'
BGBLUE=';44m'
BGWHITE=';47m'
NC='\033[0m'
if [ $# -eq 0 ] || [ $# -lt 2 ] || ( [ $# -eq 2 ] && [ $2 != "-e" ]); then
	echo -e "${FRM}${GREEN}${BGRED} No option supplied, use one of the following...${NC}"
	echo -e "The first parammeter is the command to execute"
	echo -e "\t-d, --default"
	echo -e "\t-k, --kinect"
	echo -e "\t-u, --usb"
	echo -e "\t-r, --Rode usb"
	echo -e "\t-od, --OutputDefault"
	echo -e "\t-ou, --OutputUSB"
	echo -e "The second parammeter is the device to manager"
	echo -e "\t-e, --Enable"
	echo -e "\t-v, --Volumen"
	echo -e "The third parammeter is optional, only is used when the second parammeter is -v"
	exit;
fi

op1=$1
op2=$2
op3=$3

if [ $op1 != "-od" ] && [ $op1 != "-ou" ]; then 
	sources=$( pacmd list-sources | grep alsa_input )
	for source in $sources
	do
		if [ $source != "name:" ]; then
			lenght=${#source}-2
			if echo "$source" | grep -q "alsa_input.pci"; then
				if [ $op1 = "-d" ]; then
					device=${source:1:$lenght}
					if [ $op2 = "-e" ]; then
						echo "Trying enable device " $device
						pacmd set-default-source $device
					elif [ $op2 = "-v" ];then
						echo "Trying change volumen " $device
						pacmd set-source-volume $device $op3
					fi
				fi
			elif echo "$source" | grep -q "alsa_input.usb-Microsoft_Kinect_USB_Audio"; then
				if [ $op1 = "-k" ]; then
					device=${source:1:$lenght}
					if [ $op2 = "-e" ]; then
						echo "Trying enable device " $device
						pacmd set-default-source $device
					elif [ $op2 = "-v" ];then
						echo "Trying change volumen " $device
						pacmd set-source-volume $device $op3
					fi
				fi
			elif echo "$source" | grep -q "alsa_input.usb-0d8c"; then
				if [ $op1 = "-u" ]; then
					device=${source:1:$lenght}
					if [ $op2 = "-e" ]; then
						echo "Trying enable device " $device
						pacmd set-default-source $device
					elif [ $op2 = "-v" ];then
					echo "Trying change volumen " $device
						pacmd set-source-volume $device $op3
					fi
				fi
			elif echo "$source" | grep -q "alsa_input.usb-C-Media_Electronics_Inc"; then
				if [ $op1 = "-r" ]; then
					device=${source:1:$lenght}
					if [ $op2 = "-e" ]; then
						echo "Trying enable device " $device
						pacmd set-default-source $device
					elif [ $op2 = "-v" ];then
					echo "Trying change volumen " $device
						pacmd set-source-volume $device $op3
					fi
				fi
			fi
		fi
	done
else
	sources=$( pacmd list-sinks | grep alsa_output )
	for source in $sources
	do
		if [ $source != "name:" ]; then
			lenght=${#source}-2
			if echo "$source" | grep -q "alsa_output.pci"; then
				if [ $op1 = "-od" ]; then
					device=${source:1:$lenght}
					if [ $op2 = "-e" ]; then
						echo "Trying enable device " $device
						pacmd set-default-sink $device
					elif [ $op2 = "-v" ];then
						echo "Trying change volumen " $device
						pacmd set-sink-volume $device $op3
					fi
				fi
			elif echo "$source" | grep -q "alsa_input.usb-C-Media_Electronics_Inc"; then
				if [ $op1 = "-ou" ]; then
					device=${source:1:$lenght}
					if [ $op2 = "-e" ]; then
						echo "Trying enable device " $device
						pacmd set-default-sink $device
					elif [ $op2 = "-v" ];then
					echo "Trying change volumen " $device
						pacmd set-sink-volume $device $op3
					fi
				fi
			fi
		fi
	done
fi
