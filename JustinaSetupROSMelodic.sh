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

if [ "$EUID" -ne 0 ]; then
	echo -e "This script ${FRM}${RED}${BGBLACK}must be executed as sudo${NC}"
	exit;
fi
#ROS STUFF
sudo /bin/su -c "echo 'deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main' >> /etc/apt/sources.list.d/ros-latest.list"

sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt-get update
sudo apt-get install ros-melodic-desktop-full
sudo apt install python-rosdep
sudo rosdep init
rosdep update
if [ "$EUID" -ne 0 ]; then #HASNT BEEN RUNED AS ROOT
	echo "source /opt/ros/melodic/setup.bash" >> /home/$USER/.bashrc
	source /home/$USER/.bashrc
else #U R ROOT DUMB
	echo "source /opt/ros/melodic/setup.bash" >> /home/$SUDO_USER/.bashrc
	source /home/$SUDO_USER/.bashrc
fi
sudo rosdep fix-permissions
sudo apt install python-rosinstall
