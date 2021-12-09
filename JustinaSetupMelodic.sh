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

if [ $# -eq 0 ] ; then
	echo -e "${FRM}${GREEN}${BGRED} No option supplied, use one of the following...${NC}"
	echo -e "\t-i, --install"
	echo -e "\t\t To install Justina software for first time"
	echo -e "\t\t (${FRM}${RED}${BGBLACK}must be executed as sudo${NC})"
	echo -e "\t-u, --update"
	echo -e "\t\t To update an already existent Justina installation,"
	echo -e "\t\t this includes udev rules, folder creations and user groups"
else
	SOURCE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
	echo -e "${FRM}${WHITE}${BGBLUE}The source directory is $SOURCE_DIR ${NC}"
	if [ "$1" == "-i" ] || [ "$1" == "--install" ]; then
		if [ ! "$EUID" -ne 0 ]; then
			echo -e "This script ${FRM}${RED}${BGBLACK}must be executed as normal user${NC}"
			exit;
		fi
		INSTALL_DIR=""
		if [ $# -eq 2 ] ; then
			INSTALL_DIR=$2
			if [ ! -d "$INSTALL_DIR" ]; then
				echo -e "${FRM}${RED}${BGBLACK}Not exist installation directory${NC}"
				exit -1
			else
				echo -e "${FRM}${WHITE}${BGBLUE}The installation directory is $INSTALL_DIR ${NC}"
			fi
		else
			INSTALL_DIR=$HOME
			echo -e "${FRM}${WHITE}${BGBLUE}The installation directory is $INSTALL_DIR ${NC}"
		fi
		#SCRIPT START
		#THE REAL STUFF
		echo -e "${FRM}${WHITE}${BGBLUE} Installing Justina's dependencies ${NC}"
		sudo apt-get -y update
		sudo apt-get install -y freeglut3-dev pkg-config build-essential libxmu-dev libxi-dev libusb-1.0-0-dev doxygen graphviz mono-complete
		sudo apt-get install -y build-essential libgtk2.0-dev libjpeg-dev libtiff5-dev libjasper-dev libopenexr-dev cmake python-dev python-numpy python-tk libtbb-dev libeigen3-dev yasm libfaac-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev sphinx-common texlive-latex-extra libv4l-dev libdc1394-22-dev libavcodec-dev libavformat-dev libswscale-dev ant default-jdk libvtk6.3 libprotoc-dev protobuf-compiler
		echo -e "${FRM}${GREEN}${BGBLUE} Jusina's dependencies have been installed ${NC}"

		#TODO Validate that as necesary, because this is for test kinect one installation
		sudo apt-get install -y libturbojpeg libjpeg-turbo8-dev
		sudo apt-get install -y libglfw3-dev
		#TODO END
		
		cd $INSTALL_DIR
		opencvFile="$(pwd)/opencv-3.3.1.zip"
		opencv_contrib_file="$(pwd)/opencv_contrib-3.3.1.zip"
		if [ ! -f "$opencvFile" ]; then
			echo -e "${FRM}${WHITE}${BGBLUE} Downloading OpenCV 3.3 ${NC}"
			wget https://sourceforge.net/projects/opencvlibrary/files/opencv-unix/3.3.1/opencv-3.3.1.zip
			echo -e "${FRM}${GREEN}${BGBLUE} OpenCV 3.3 have been downloaded ${NC}"
			unzip opencv-3.3.1.zip
		fi
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
		cmake -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON -D WITH_VTK=ON -D WITH_OPENNI=ON -D WITH_OPENCL=OFF -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-3.3.1/modules ..
		make -j4
		sudo make install
		sudo touch /etc/ld.so.conf.d/opencv.conf
		sudo /bin/su -c "echo '/usr/local/lib' >> /etc/ld.so.conf.d/opencv.conf"
		sudo ldconfig
		echo -e "${FRM}${GREEN}${BGBLUE} OpenCV 3.3 have been installed ${NC}"
		echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:/opt/codigo/JUSTINA/catkin_ws/src:/opt/ros/melodic/share" >> /home/$USER/.bashrc
		source /home/$USER/.bashrc

		cd $INSTALL_DIR
		dlib_file="v19.6.zip"
		dlib_file_desc="dlib-19.6"
		dlib_file_path="$INSTALL_DIR/$dlib_file"
		if [ ! -f "$dlib_file_path" ]; then
			echo -e "${FRM}${WHITE}${BGBLUE} Downloading dlib library ${NC}"
			wget https://github.com/davisking/dlib/archive/v19.6.zip
			unzip $dlib_file
			echo -e "${FRM}${GREEN}${BGBLUE} dlib library have been downloading ${NC}"
		fi
		echo -e "${FRM}${WHITE}${BGBLUE} Installing dlib library ${NC}"
		cd $dlib_file_desc
		mkdir build
		cd build
		cmake ..
		make -j4
		sudo make install
		echo -e "${FRM}${GREEN}${BGBLUE} dlib library have been installing ${NC}"

		echo -e "${FRM}${WHITE}${BGBLUE} Preparing to build Prime sense drivers ${NC}"
		cd $INSTALL_DIR
		mkdir -p prime_sense
		cd prime_sense
		sensorKinect_file="$(pwd)/SensorKinect"
		if [ ! -d "$sensorKinect_file" ]; then
			git clone https://github.com/ph4m/SensorKinect.git
		fi
		cd SensorKinect
		git checkout unstable
		echo -e "${FRM}${GREEN}${BGBLUE} Prime sense drivers have been prepared ${NC}"
		cd ../SensorKinect/Platform/Linux/CreateRedist
		echo -e "${FRM}${WHITE}${BGBLUE} Installing Prime sense drivers ${NC}"
		./RedistMaker
		cd ../Redist/Sensor-Bin-Linux-x64-v5.1.2.1/
		sudo ./install.sh
		echo -e "${FRM}${GREEN}${BGBLUE}Prime sense drivers have been installed${NC}"
		#Comment this of the script, the url not work and the skeleton finder is deprecated node
		#echo -e "${FRM}${WHITE}${BGBLUE}Installing NITE for skeleton traking${NC}"
		#cd $INSTALL_DIR
		#nite_file="$(pwd)/NITE-Bin-Linux-x64-v1.5.2.23.tar.zip"
		#if [ ! -f "$nite_file" ]; then
		#	wget http://www.openni.ru/wp-content/uploads/2013/10/NITE-Bin-Linux-x64-v1.5.2.23.tar.zip
		#	unzip NITE-Bin-Linux-x64-v1.5.2.23.tar.zip
		#	tar -xvf NITE-Bin-Linux-x64-v1.5.2.23.tar.bz2
		#fi
		#cd NITE-Bin-Dev-Linux-x64-v1.5.2.23
		#sudo ./install.sh
		#echo -e "${FRM}${GREEN}${BGBLUE}NITE correctly installed ${NC}"
		echo -e "${FRM}${WHITE}${BGBLUE}Installing OpenNI to update default libraries${NC}"
		cd $INSTALL_DIR
		openni_file_dir="$(pwd)/OpenNI"
		if [ ! -f "$openni_file_dir" ]; then
			git clone https://github.com/OpenNI/OpenNI
		fi
		cd OpenNI/
		git checkout unstable
		cd Platform/Linux/CreateRedist
		./RedistMaker	
		cd ../Redist/OpenNI-Bin-Dev-Linux-x64-v1.5.8.5/
		sudo ./install.sh
		echo -e "${FRM}${GREEN}${BGBLUE}OpenNI have been installed to update default libraries${NC}"
		echo -e "${FRM}${WHITE}${BGBLUE}Installing ros package's dependencies${NC}"
		sudo apt-get -y install ros-melodic-urg-node
		sudo apt-get -y install ros-melodic-joy
		sudo apt-get -y install ros-melodic-openni-camera
		sudo apt-get -y install ros-melodic-openni-launch
		sudo apt-get -y install ros-melodic-openni2-camera
		sudo apt-get -y install ros-melodic-openni2-launch
		sudo apt-get -y install ros-melodic-amcl
		sudo apt-get -y install ros-melodic-tf2-bullet
		sudo apt-get -y install ros-melodic-fake-localization
		sudo apt-get -y install ros-melodic-map-server
		sudo apt-get -y install ros-melodic-sound-play
		sudo apt-get -y install ros-melodic-gmapping
		sudo apt-get -y install ros-melodic-pointcloud-to-laserscan
		#sudo apt-get -y install ros-kinetic-dynamixel-sdk
		echo -e "${FRM}${GREEN}${BGBLUE}Ros package's dependencies have been installed${NC}"

		echo -e "${FRM}${WHITE}${BGBLUE}Installing pyRobotics and clips dependencies${NC}"
		cd $SOURCE_DIR/ToInstall/pyRobotics-1.8.0
		sudo python setup.py config
		sudo python setup.py install
		cd $SOURCE_DIR/ToInstall/CLIPS/instPy
		sudo python setup.py config
		sudo python setup.py install
		cd $SOURCE_DIR/ToInstall/CLIPS/
		sudo cp -r clips/ /usr/local/lib/python2.7/dist-packages/
		echo -e "${FRM}${GREEN}${BGBLUE}pyRobotics and clips dependencies has been installed${NC}"

		#TODO Validate that as necesary, because this is for test the kinect one installation
		sudo apt-get -y install build-essential cmake pkg-config
		sudo apt-get -y install libva-dev libjpeg-dev
		sudo apt-get -y install libturbojpeg0-dev
		cd $INSTALL_DIR
		git clone https://github.com/OpenKinect/libfreenect2.git
		cd libfreenect2
		mkdir build
		sudo rm build/*
		cd build
		cmake ..
		make -j4
		sudo make install
		sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
		#TODO END

		sudo apt-get install -y libgflags-dev libgtest-dev libc++-dev clang
		sudo apt-get install -y ros-melodic-sound-play
		sudo apt-get install -y ros-melodic-map-server
		sudo apt-get install -y ros-melodic-tf2-bullet

		echo -e "${FRM}${WHITE}${BGBLUE} Preparing to build the Dynamixel SDK${NC}"
		cd $INSTALL_DIR
		git clone https://github.com/ROBOTIS-GIT/DynamixelSDK
		cd DynamixelSDK
		git checkout 3.5.4
		echo -e "${FRM}${GREEN}${BGBLUE} Dynamixel SDK have been prepared ${NC}"
		echo -e "${FRM}${WHITE}${BGBLUE} Installing the Dynamixel SDK${NC}"
		cd c++/build
		cmake ..
		make -j4
		sudo make install
		echo -e "${FRM}${GREEN}${BGBLUE} Dynamixel SDK have been installed ${NC}"

		echo -e "${FRM}${WHITE}${BGBLUE} Preparing the serial library, that use jrk controller${NC}"
		cd $INSTALL_DIR
		git clone https://github.com/wjwwood/serial
		cd serial
		mkdir build
		cd build
		echo -e "${FRM}${GREEN}${BGBLUE} The serial lib have been prepared ${NC}"
		echo -e "${FRM}${WHITE}${BGBLUE} Installing the serial library, that use jrk controller${NC}"
		cmake ..
		make -j4
		sudo make install
		echo -e "${FRM}${GREEN}${BGBLUE} The serial lib have been installed ${NC}"

		echo -e "${FRM}${WHITE}${BGBLUE}Installing basic audio libraries${NC}"
		sudo apt-get -y install libzbar-dev
		echo -e "${FRM}${WHITE}${BGBLUE}Audio support will be installed, choose <yes> when asked for real time permissions${NC}"
		read -p "(Waiting for key press in order to continue)"
		sudo apt-get -y install jackd2 libjack-jackd2-dev pulseaudio-module-jack qjackctl
		echo -e "${FRM}${WHITE}${BGBLUE}Installing kinect audio driver${NC}"
		sudo apt-get -y install kinect-audio-setup
		echo -e "${FRM}${WHITE}${BGBLUE}Installing pyaudio lib for directional audio node${NC}"
		sudo apt-get -y install libasound-dev portaudio19-dev libportaudio2 libportaudiocpp0
		sudo apt-get -y install ffmpeg
		sudo pip install pyaudio==0.2.9 --upgrade
		echo -e "${FRM}${GREEN}${BGBLUE}Audio libraries have been installed${NC}"
		echo -e "${FRM}${WHITE}${BGBLUE}Installing the dependecies to sphinxbase${NC}"
		sudo apt-get install libbison-dev
		sudo apt-get install swig
		echo -e "${FRM}${GREEN}${BGBLUE}The sphinxbase dependencies has been installed${NC}"
		echo -e "${FRM}${WHITE}${BGBLUE}Downloading the sphinxbase${NC}"
		cd $INSTALL_DIR
		sphinxbase_file="$(pwd)/sphinxbase.tar.gz"
		if [ ! -f "$sphinxbase_file" ]; then
			wget https://sourceforge.net/projects/cmusphinx/files/sphinxbase/5prealpha/sphinxbase-5prealpha.tar.gz/download -O sphinxbase.tar.gz
		fi
		echo -e "${FRM}${GREEN}${BGBLUE}The sphinxbase has been downloaded${NC}"
		echo -e "${FRM}${WHITE}${BGBLUE}Installing the sphinxbase${NC}"
		tar -xvf sphinxbase.tar.gz
		mv sphinxbase-5prealpha sphinxbase
		cd sphinxbase
		./configure
		make
		sudo make install
		echo -e "${FRM}${GREEN}${BGBLUE}The sphinxbase has been installed${NC}"
		echo -e "${FRM}${WHITE}${BGBLUE}Downloading the pocketsphinx${NC}"
		cd $INSTALL_DIR
		pocketsphinx_file="$(pwd)/pocketsphinx.tar.gz"
		if [ ! -f "$pocketsphinx_file" ]; then
			wget https://sourceforge.net/projects/cmusphinx/files/pocketsphinx/5prealpha/pocketsphinx-5prealpha.tar.gz/download -O pocketsphinx.tar.gz
		fi
		echo -e "${FRM}${GREEN}${BGBLUE}The pocketsphinx has been downloaded${NC}"
		echo -e "${FRM}${WHITE}${BGBLUE}Installing the pocketsphinx${NC}"
		tar -xvf pocketsphinx.tar.gz
		mv pocketsphinx-5prealpha pocketsphinx
		cd pocketsphinx
		./configure
		make clean all
		make check
		sudo make install
		echo -e "${FRM}${GREEN}${BGBLUE}The pocketsphinx has been installed${NC}"
		echo -e "${FRM}${WHITE}${BGBLUE}Coping OpenCV libraries to ros directory${NC}"
		FILES="/usr/local/lib/libopencv*"
		pathCopy="/opt/ros/melodic/lib/"
		pattherDelete=$pathCopy"libopencv*"
		for f in $pattherDelete
		do
			sudo rm $f
		done

		for f in $FILES
		do
			filename=$(basename "$f")
			extension="${filename##*.}"
			if [ $extension = "so" ]; then
				sudo cp $f $pathCopy
				newfilename="${filename%.*}"3.$extension
				sudo mv $pathCopy$filename $pathCopy$newfilename
				sudo echo $newfilename
				cd $pathCopy
				sudo ln -s $newfilename.3.3 $newfilename.3.3.1
				sudo ln -s $newfilename $newfilename.3.3
			fi
			#echo $f
			#mv $f $f
		done
		echo -e "${FRM}${GREEN}${BGBLUE}Have been copying the OpenCV libraries to ROS directory${NC}"
		if [ ! -d "/media/$USER/usbPDF/" ]; then
			sudo mkdir /media/$USER/USBPDF/
			mkdir /home/$USER/objs/
			#Add user to dialout, in order to use Arduino and Texas instrument board----
			sudo adduser $USER dialout
		fi
		echo "source /opt/ros/melodic/setup.bash" >> /home/$USER/.bashrc
		echo "source $SOURCE_DIR/catkin_ws/devel/setup.bash" >> /home/$USER/.bashrc
		source /home/$USER/.bashrc
		source $SOURCE_DIR/catkin_ws/devel/setup.bash
		echo -e "${FRM}${WHITE}${BGBLUE}Copying the rules of Justina to system${NC}"
		cd $SOURCE_DIR
		sudo cp ToInstall/USB/80-justinaRobot.rules /etc/udev/rules.d/
		sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
		serialsPort=($(ls /dev/ttyACM*))
		for f in "${serialsPort[@]}"
		do
			sudo echo $f
			cmd="sudo udevadm test \$(udevadm info \-q path \-n $f)"
			sudo echo $cmd
			eval "$cmd"
			 
		done
		echo -e "${FRM}${WHITE}${BGBLUE}Sourcing to get git branche and alias launchers${NC}"
		echo "green=\"\[\033[01;32m\]\"" >> /home/$USER/.bashrc
		echo "blue=\"\[\033[01;34m\]\"" >> /home/$USER/.bashrc
		echo "purple=\"\[\033[01;35m\]\"" >> /home/$USER/.bashrc
		echo "red=\"\[\033[01;31m\]\"" >> /home/$USER/.bashrc
		echo "yellow=\"\[\033[01;33m\]\"" >> /home/$USER/.bashrc
		echo "reset=\"\[\033[0m\]\"" >> /home/$USER/.bashrc
		echo "export GIT_PS1_SHOWDIRTYSTATE=1" >> /home/$USER/.bashrc
		echo "export PS1=\"\$red\u@\$green\h\$yellow:\$red\\\$(__git_ps1)\$blue\\\\W\$green->\$reset \"" >> /home/$USER/.bashrc
		echo "alias em='emacs24 -nw'" >> /home/$USER/.bashrc
		echo "alias jsea='roslaunch surge_et_ambula justina.launch'" >> /home/$USER/.bashrc
		echo "alias jseas='roslaunch surge_et_ambula justina_simul.launch'" >> /home/$USER/.bashrc
		echo -e "${FRM}${RED}${BGWHITE}You can now ${NC}${FRM}${BLACK}${BGWHITE}behold${NC}${FRM}${RED}${BGWHITE} the power of Justina software${NC}"
	elif [ "$1" == "-u" ] || [ "$1" == "--update" ]; then
		if [ ! -d "/media/$USER/usbPDF/" ]; then
			sudo mkdir /media/$USER/USBPDF/
			mkdir /home/$USER/objs/
			#Add user to dialout, in order to use Arduino and Texas instrument board----
			sudo adduser $USER dialout
		fi
		echo "source /opt/ros/melodic/setup.bash" >> /home/$USER/.bashrc
		echo "source $SOURCE_DIR/catkin_ws/devel/setup.bash" >> /home/$USER/.bashrc
		source /home/$USER/.bashrc
		source $SOURCE_DIR/catkin_ws/devel/setup.bash
		echo -e "${FRM}${WHITE}${BGBLUE}Sourcing to get git branche and alias launchers${NC}"
		echo "green=\"\[\033[01;32m\]\"" >> /home/$USER/.bashrc
		echo "blue=\"\[\033[01;34m\]\"" >> /home/$USER/.bashrc
		echo "purple=\"\[\033[01;35m\]\"" >> /home/$USER/.bashrc
		echo "red=\"\[\033[01;31m\]\"" >> /home/$USER/.bashrc
		echo "yellow=\"\[\033[01;33m\]\"" >> /home/$USER/.bashrc
		echo "reset=\"\[\033[0m\]\"" >> /home/$USER/.bashrc
		echo "export GIT_PS1_SHOWDIRTYSTATE=1" >> /home/$USER/.bashrc
		echo "export PS1=\"\$red\u@\$green\h\$yellow:\$red\\\$(__git_ps1)\$blue\\\\W\$green->\$reset \"" >> /home/$USER/.bashrc
		echo "alias em='emacs24 -nw'" >> /home/$USER/.bashrc
		echo "alias jsea='roslaunch surge_et_ambula justina.launch'" >> /home/$USER/.bashrc
		echo "alias jseas='roslaunch surge_et_ambula justina_simul.launch'" >> /home/$USER/.bashrc
		echo -e "${FRM}${WHITE}${BGBLUE}Copying the rules of Justina to system${NC}"
		cd $SOURCE_DIR
		sudo cp ToInstall/USB/80-justinaRobot.rules /etc/udev/rules.d/
		sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
		serialsPort=($(ls /dev/ttyACM*))
		for f in "${serialsPort[@]}"
		do
			sudo echo $f
			cmd="sudo udevadm test \$(udevadm info \-q path \-n $f)"
			sudo echo $cmd
			eval "$cmd"
			 
		done
		echo -e "${FRM}${RED}${BGWHITE}You can now ${NC}${FRM}${BLACK}${BGWHITE}behold${NC}${FRM}${RED}${BGWHITE} the power of Justina software${NC}"
	else
		echo -e "${FRM}${CYAN}${BGRED} Invalid option supplied, use one of the following...${NC}"
		echo -e "\t-i, --install"
		echo -e "\t\t To install Justina software for first time"
		echo -e "\t\t (${FRM}${RED}${BGBLACK}must be executed as sudo${NC})"
		echo -e "\t-u, --update"
		echo -e "\t\t To update an already existent Justina installation,"
		echo -e "\t\t this includes udev rules, folder creations and user groups"
	fi
fi
