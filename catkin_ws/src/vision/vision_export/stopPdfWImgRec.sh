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
FILE=/home/$USER/JUSTINA/catkin_ws/src/vision/vision_export/temp/PUMAS_$TEST.tex;
mkdir $OPATH;

if grep "\end{document}" /home/$USER/JUSTINA/catkin_ws/src/vision/vision_export/temp/PUMAS_$TEST.tex > /dev/null
then
   sed -i -- 's/\end{document}/ /g' /home/$USER/JUSTINA/catkin_ws/src/vision/vision_export/temp/*
   sed -i '/%imgStart/,/%imgStop/d' /home/$USER/JUSTINA/catkin_ws/src/vision/vision_export/temp/*
####Append images at end of file
echo "%imgStart" >> $FILE;
for f in $2*$EXT; do
	imagesL2=($(ls $f))
	#for f2 in $f*$EXT; do
	for f2 in "${imagesL2[@]}"; do
		fPath=$f"/"$f2
		g=${fPath%.*};
		echo "\\begin{figure}[!ht]" >> $FILE;
		echo "\\caption{\detokenize{${g##*/}}}" >> $FILE;
		echo "\\centering" >> $FILE;
		echo "\\includegraphics[width=0.9\\linewidth]{$fPath}" >> $FILE;
		echo "\\end{figure}" >> $FILE;
	done;
done;
echo "%imgStop" >> $FILE;
#####
   echo "\\end{document}" >> $FILE;
else
####Append images at end of file
echo "%imgStart" >> $FILE;
for f in $2*$EXT; do
	imagesL2=($(ls $f))
	#for f2 in $f*$EXT; do
	for f2 in "${imagesL2[@]}"; do
		fPath=$f"/"$f2
		echo $image
		g=${fPath%.*};
		echo "\\begin{figure}[!ht]" >> $FILE;
		echo "\\caption{\detokenize{${g##*/}}}" >> $FILE;
		echo "\\centering" >> $FILE;
		echo "\\includegraphics[width=0.9\\linewidth]{$fPath}" >> $FILE;
		echo "\\end{figure}" >> $FILE;
	done;
done;
echo "%imgStop" >> $FILE;
#####
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
