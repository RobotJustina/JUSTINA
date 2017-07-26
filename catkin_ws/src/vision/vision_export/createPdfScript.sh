#!/bin/bash
#first argument is the test and file name
TEST=$1
#The file will be added to /home/$USER/testPDFs/ folder
if (($# < 1 )); then
    echo "No input parameters where supplied:";
    echo "1st parameter is test and file name";
    echo "2nd argument (optional) is the path to clean before";
    exit 0
fi
#erase at the script start
rm /home/$USER/JUSTINA/catkin_ws/src/vision/vision_export/temp/*.tex;
rm /home/$USER/JUSTINA/catkin_ws/src/vision/vision_export/temp/*.log;
rm /home/$USER/JUSTINA/catkin_ws/src/vision/vision_export/temp/*.aux;
rm $OPATH/*.aux;
rm $OPATH/*.log;
#optional stuff
if [ -z "$2" ]; then
	echo "no dir to clean, PDF created normally"
else
	mkdir /home/$USER/JUSTINA/catkin_ws/src/vision/vision_export/temp/files_from_$USER-$(date +"%H-%M-%S")
	for i in ${@:2}
	do
	    echo "directory $i will be cleaned"
	    mv $i /home/$USER/JUSTINA/catkin_ws/src/vision/vision_export/temp/files_from_$USER-$(date +"%H-%M-%S")/
	    mkdir $i
	done
fi
#and his name will be...
FILE=/home/$USER/JUSTINA/catkin_ws/src/vision/vision_export/temp/PUMAS_$TEST-$(date +"%Y-%m-%d").tex;
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
echo "\usepackage{fancyvrb}" >> $FILE;
echo "\\begin{document}" >> $FILE;
echo "\\begin{center}" >> $FILE;
echo "\\begingroup" >> $FILE;
echo "\\fontsize{16pt}{17pt}\selectfont" >> $FILE;
echo "\\begin{BVerbatim}" >> $FILE;
echo "Team: PUMAS" >> $FILE;
echo "Test: $TEST" >> $FILE
echo "\\end{BVerbatim}" >> $FILE;
echo "\\endgroup " >> $FILE;
echo "\\end{center}" >> $FILE;
exit 0
