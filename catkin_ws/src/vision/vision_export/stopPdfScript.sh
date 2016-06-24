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
rm /home/$USER/JUSTINA/catkin_ws/src/vision/vision_export/temp/$1.tex;
rm /home/$USER/JUSTINA/catkin_ws/src/vision/vision_export/temp/*.log;
rm /home/$USER/JUSTINA/catkin_ws/src/vision/vision_export/temp/*.aux;
rm $1.tex
rm $OPATH/*.aux;
rm $OPATH/*.log;
exit 0
