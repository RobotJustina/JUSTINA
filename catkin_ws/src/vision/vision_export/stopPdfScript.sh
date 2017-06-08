#!/bin/bash
#first argument is the test and file name
TEST=$1
OPATH=/home/$USER/testPDFs
#The file will be added to /home/$USER/testPDFs/ folder
if (($# < 1 )); then
    echo "No input parameters where supplied:";
    echo "1st parameter is test and file name";
    exit 0
fi
#and his name will be $TEST_timestamp.pdf
FILE=/home/$USER/JUSTINA/catkin_ws/src/vision/vision_export/temp/$TEST.tex;
mkdir $OPATH;
echo "\\end{document}" >> $FILE;
pdflatex -output-directory=$OPATH $FILE;
REMOVABLE_DEVICE=$(find /sys/block/ -type l -printf "%f %l\n" | awk '/usb/{print $1}')

if grep -Eq "(sdb|sdc)" <<< "$REMOVABLE_DEVICE" ;then
	sudo mount /dev/sdb /media/usbPDF
	sudo mount /dev/sdc /media/usbPDF
	cp $OPATH/$FILE /media/usbPDF
else
	echo "USB NOT PRESENT!, ONLY LOCAL PDF CREATED"
fi
rm /home/$USER/JUSTINA/catkin_ws/src/vision/vision_export/temp/$1.tex;
rm /home/$USER/JUSTINA/catkin_ws/src/vision/vision_export/temp/*.log;
rm /home/$USER/JUSTINA/catkin_ws/src/vision/vision_export/temp/*.aux;
rm $1.tex
rm $OPATH/*.aux;
rm $OPATH/*.log;
exit 0
