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
#and his name will be ...
FILE=/home/$USER/JUSTINA/catkin_ws/src/vision/vision_export/temp/PUMAS_$TEST_$(date +"date_%Y-%m-%d").tex;
mkdir $OPATH;
echo "\\end{document}" >> $FILE;
pdflatex -output-directory=$OPATH $FILE;
#if [ "`lsblk -o name|grep sdb1`" != "" ]; then 
	echo "usbPDF connected, creating a copy of the local latex file";
	cp $OPATH/*.pdf /media/$USER/USBPDF/
#fi
rm /home/$USER/testPDFs/*.log;
rm /home/$USER/testPDFs/*.aux;
rm /home/$USER/JUSTINA/catkin_ws/src/vision/vision_export/temp/*.log;
rm /home/$USER/JUSTINA/catkin_ws/src/vision/vision_export/temp/*.aux;
exit 0
