#!/bin/bash
#first argument is the test and file name
TEST=$1
#The file will be added to /home/$USER/testPDFs/ folder
if (($# < 1 )); then
    echo "No input parameters where supplied:";
    echo "1st parameter is test and file name";
    exit 0
fi
#erase at the script start
rm /home/$USER/JUSTINA/catkin_ws/src/vision/vision_export/temp/PUMAS_$TEST_$(date +"date_%Y-%m-%d").tex;
rm /home/$USER/JUSTINA/catkin_ws/src/vision/vision_export/temp/*.log;
rm /home/$USER/JUSTINA/catkin_ws/src/vision/vision_export/temp/*.aux;
rm PUMAS_$TEST_$(date +"date_%Y-%m-%d").tex
rm $OPATH/*.aux;
rm $OPATH/*.log;
#and his name will be...
FILE=/home/$USER/JUSTINA/catkin_ws/src/vision/vision_export/temp/PUMAS_$TEST_$(date +"date_%Y-%m-%d").tex;
echo "\\documentclass{article}" >> $FILE;
echo "\\usepackage" >> $FILE;
echo "[" >> $FILE;
echo "       a4paper," >> $FILE;
echo "       left=1cm," >> $FILE;
echo "       right=2cm," >> $FILE;
echo "       top=3cm," >> $FILE;
echo "       bottom=4cm," >> $FILE;
echo "]" >> $FILE;
echo "{geometry}" >> $FILE;
echo "\\usepackage[T1]{fontenc}" >> $FILE;
echo "\\usepackage{graphicx}" >> $FILE;
echo "\\begin{document}" >> $FILE;
echo "\\begin{center}" >> $FILE;
echo "Team: PUMAS\\\\" >> $FILE;
echo "Test: $TEST\\\\" >> $FILE
echo "\\end{center}" >> $FILE;
exit 0
