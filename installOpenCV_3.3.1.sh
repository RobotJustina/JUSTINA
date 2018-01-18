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

cd $INSTALL_DIR
opencvFile="$(pwd)/opencv-3.3.1.zip"
opencv_contrib_file="$(pwd)/opencv_contrib-3.3.1.zip"

#### INSTALL QT5 NEEDED BY OPENCV DEPENDENCES #####
sudo apt install qtbase5-dev


### DOWNLOAD OPENCV 3.3.1
if [ ! -f "$opencvFile" ]; then
	echo -e "${FRM}${WHITE}${BGBLUE} Downloading OpenCV 3.3 ${NC}"
	wget https://sourceforge.net/projects/opencvlibrary/files/opencv-unix/3.3.1/opencv-3.3.1.zip
	echo -e "${FRM}${GREEN}${BGBLUE} OpenCV 3.3 have been downloaded ${NC}"
	unzip opencv-3.3.1.zip
fi

### DOWNLOAD OPENCV CONTRIB
if [ ! -f "$opencv_contrib_file" ]; then
	echo -e "${FRM}${WHITE}${BGBLUE} Downloading OpenCV 3.3 contrib ${NC}"
	wget https://github.com/opencv/opencv_contrib/archive/3.3.1.zip
	echo -e "${FRM}${GREEN}${BGBLUE} OpenCV 3.3 contrib  have been downloaded ${NC}"
	mv 3.3.1.zip opencv_contrib-3.3.1.zip
	unzip opencv_contrib-3.3.1.zip
fi


echo -e "${FRM}${WHITE}${BGBLUE} Installing OpenCV 3.3 ${NC}"
cd opencv-3.3.1
mkdir build
cd build
### COMPILING OPENCV 
sudo cmake -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON -D WITH_VTK=ON -D WITH_OPENNI=ON -D WITH_OPENCL=OFF -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-3.3.1/modules ..
make -j4
sudo make install
sudo touch /etc/ld.so.conf.d/opencv.conf
sudo /bin/su -c "echo '/usr/local/lib' >> /etc/ld.so.conf.d/opencv.conf"
sudo ldconfig
echo -e "${FRM}${GREEN}${BGBLUE} OpenCV 3.3.1 have been installed ${NC}"
echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:/opt/codigo/JUSTINA/catkin_ws/src:/opt/ros/kinetic/share" >> /home/$USER/.bashrc
source /home/$USER/.bashrc
