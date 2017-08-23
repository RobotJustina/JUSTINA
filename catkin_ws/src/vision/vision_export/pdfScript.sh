#!/bin/bash
#first argument is the test name
TEST=$1
#second argument is the path of the image folder
IPATH=$2
#OPATH=/media/rag/7D1D-3EB7/testPDFs
OPATH=/home/$USER/testPDFs
#image backup path
BPATH=$IPATH$(date +"%Y-%m-%d.time_%H-%M-%S")
#extension of the files inside the folder
EXT=.png;
#The file will be added to /home/$USER/testPDFs/ folder
#and his name will be PUMAS_timestamp.pdf
FILE=PUMAS_$(date +"%Y-%m-%d.time_%H-%M-%S").tex;
if (($# < 2 )); then
    echo "No all the input parameters where supplied:";
    echo "1st parameter is test name";
    echo "2nd parameter is pictures folder path";
    echo "All the parameters without spaces.";
    exit 0
fi
mkdir $OPATH;
rm $FILE;
TFILE=$(readlink -f $FILE)
echo "Temporal Tex File Created in "$TFILE
echo "\\documentclass{article}" >> $FILE;
echo "\\usepackage{graphicx}" >> $FILE;
echo "\\usepackage[T1]{fontenc}" >> $FILE;
echo "\\begin{document}" >> $FILE;
echo "\\begin{center}" >> $FILE;
echo "Team: PUMAS\\\\" >> $FILE;
echo "Test: $TEST" >> $FILE
echo "\\end{center}" >> $FILE;
for f in $2*$EXT; do
	g=${f%.*};
	echo "\\begin{figure}[!ht]" >> $FILE;
	echo "\\caption{\detokenize{${g##*/}}}" >> $FILE;
	echo "\\centering" >> $FILE;
	echo "\\includegraphics[width=0.9\\linewidth]{$f}" >> $FILE;
	echo "\\end{figure}" >> $FILE;
done;
echo "\\end{document}" >> $FILE;
mkdir $BPATH
pdflatex -output-directory=$OPATH $FILE;
#if [ "`lsblk -o name|grep sdb1`" != "" ]; then 
	echo "usbPDF connected, creating a copy of the local latex file";
	cp $OPATH/*.pdf /media/$USER/USBPDF/
#fi
mv $IPATH*.png $BPATH
rm *.tex
rm $OPATH/*.aux;
rm $OPATH/*.log;
exit 0
