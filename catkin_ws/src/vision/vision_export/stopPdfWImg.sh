#!/bin/bash
#first argument is the test and file name
TEST=$1
#second argument is the path of the image folder
IPATH=$2
#Output Path
OPATH=/home/$USER/testPDFs
#The file will be added to /home/$USER/testPDFs/ folder
if (($# < 2 )); then
    echo "No input parameters where supplied:";
    echo "1st parameter is test and file name";
    exit 0
fi
#and his name will be $TEST_timestamp.pdf
FILE=/home/$USER/JUSTINA/catkin_ws/src/vision/vision_export/temp/PUMAS_$TEST_$(date +"date_%Y-%m-%d").tex;
mkdir $OPATH;
####Append images at end of file
for f in $2*$EXT; do
	g=${f%.*};
	echo "\\begin{figure}[!ht]" >> $FILE;
	echo "\\caption{\detokenize{${g##*/}}}" >> $FILE;
	echo "\\centering" >> $FILE;
	echo "\\includegraphics[width=0.9\\linewidth]{$f}" >> $FILE;
	echo "\\end{figure}" >> $FILE;
done;
#####
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
