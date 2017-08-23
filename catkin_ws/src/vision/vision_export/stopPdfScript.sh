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
FILE=/home/$USER/JUSTINA/catkin_ws/src/vision/vision_export/temp/PUMAS_$TEST.tex;
mkdir $OPATH;
####

if grep "\end{document}" /home/$USER/JUSTINA/catkin_ws/src/vision/vision_export/temp/PUMAS_$TEST.tex > /dev/null
then
   sed -i -- 's/\end{document}/ /g' /home/$USER/JUSTINA/catkin_ws/src/vision/vision_export/temp/*
   echo "\\end{document}" >> $FILE;
else
   echo "\\end{document}" >> $FILE;
fi
####

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
