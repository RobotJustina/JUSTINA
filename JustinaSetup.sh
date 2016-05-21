#!/bin/bash

#############################################################################
#
# Setup the following
#                               VARIABLES
#
#############################################################################

# Prerrequisites
PRERREQ=(
	freeglut3-dev pkg-config build-essential libxmu-dev libxi-dev 
	libusb-1.0-0-dev doxygen graphviz mono-complete build-essential 
	libgtk2.0-dev libjpeg-dev libtiff4-dev libjasper-dev
	libopenexr-dev cmake python-dev python-numpy python-tk libtbb-dev 
	libeigen3-dev yasm libfaac-dev libopencore-amrnb-dev 
	libopencore-amrwb-dev libtheora-dev libvorbis-dev libxvidcore-dev 
	libx264-dev libqt4-dev libqt4-opengl-dev sphinx-common 
	texlive-latex-extra libv4l-dev libdc1394-22-dev libavcodec-dev 
	libavformat-dev libswscale-dev default-jdk ant libvtk5-qt4-dev
)

# ROS Version
ROS_VER="indigo"
# ROS Key
ROS_KEY="0xB01FA116"
# ROS minimum package
ROS_MIN=(
	"ros-${ROS_VER}-desktop-full"
)

# ROS Packages
ROS_PKG=(
	ros-indigo-hokuyo-node ros-indigo-joy ros-indigo-openni-camera
	ros-indigo-openni-launch ros-indigo-openni2-camera
	ros-indigo-openni2-launch ros-indigo-amcl ros-indigo-tf2-bullet
	ros-indigo-fake-localization ros-indigo-map-server ros-indigo-sound-play
	ros-indigo-pocketsphinx
)

# OpenCV version
OpenCV_ver="2.4.9"
# OpenCV zip file
OpenCV_zip="opencv-${OpenCV_ver}.zip"
# OpenCV directory
OpenCV_dir="opencv-${OpenCV_ver}"
# OpenCV library url
OpenCV_url="http://sourceforge.net/projects/opencvlibrary/files/opencv-unix/${OpenCV_ver}/${OpenCV_zip}"
#OpenCV CMake flags
OpenCV_CMake_flags=(
	-D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D
	INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON 
	-D WITH_QT=ON -D WITH_OPENGL=ON -D WITH_VTK=ON -D WITH_OPENNI=ON
	-D WITH_OPENCL=OFF
)

# PrimeSense driver dir
PSD_dir="prime_sense"

#############################################################################
#
# Starting from here DO NOT TOUCH
# Functions
#
#############################################################################
function silent {
	$@ > /dev/null
	return $?
}

function message {
	echo "$(tput setaf 2)$@$(tput sgr 0)"
}

function warn {
	echo "$(tput setaf 3)$@$(tput sgr 0)"
}

function err {
	echo "$(tput setaf 1)$@$(tput sgr 0)" 1>&2
}

function q_continue {
	read -p "$(tput setaf 9)Continue [y/N]? $(tput sgr 0)" -n 1 -r
	if [[ ! $REPLY =~ ^[Yy]$ ]]; then
		exit 1
	fi
}

function q_sure {
	read -p "$(tput setaf 9)Are you sure [y/N]? $(tput sgr 0)" -n 1 -r
	if [[ ! $REPLY =~ ^[Yy]$ ]]; then
		exit 1
	fi
}

function add_repo {
	silent sudo add-apt-repository -y $@ || { err "Can't add repository $REPO"; exit 1; }
}

function update {
	silent sudo apt-get -qq update || { err "Can't update package list"; exit 1; }
}

function install_packages {
	silent sudo apt-get -y -qq install $@ || { err "Can't install $@"; exit 1; }
}

function check_OpenCV {
	local openCV_flag=""
	message "Checking OpenCV..."
	mapfile -t OpenCV_flag < <(locate libopencv_core)
	if [[ -z "${OpenCV_flag[@]// }" ]]; then
		message "Installing OpenCV..."
		install_openCV
		message "	done."
	else
		message "	Already installed!"
	fi
}

function install_openCV {
	cd ~
	if [[! -d "~/${OpenCV_dir}" ]]; then
		if [[ ! -f "~/${OpenCV_zip}" ]]; then
			silent wget $OpenCV_url || { err "Can't get file $OpenCV_url"; exit 1; }
		fi
		silent unzip $OpenCV_zip || { err "Can't unzip file $OpenCV_zip"; exit 1; }
	fi
	cd $OpenCV_dir || { err "Can't access $OpenCV_dir"; exit 1; }
	echo "	Building OpenCV ${OpenCV_ver}"
	silent mkdir build || { err "Can't create $OpenCV_dir/build"; exit 1; }
	cd build || { err "Can't access $OpenCV_dir/build"; exit 1; }
	silent cmake $OpenCV_CMake_flags[@] .. || { err "Error executing CMake"; exit 1; }
	silent make || { err "Error building OpenCV"; exit 1; }
	silent sudo make install || { err "Error installing OpenCV"; exit 1; }
	silent sudo echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf || { err "Error registering OpenCV library"; exit 1; }
	silent sudo ldconfig || { err "Error registering OpenCV library"; exit 1; }
}

function check_PSD {
	local PSD_flag=""
	message "Checking PrimeSense drivers..."
	mapfile -t PSD_flag < <(locate libXnCore)
	if [[ -z "${PSD_flag[@]// }" ]]; then
		message "Installing PrimeSense drivers..."
		install_PSD
		message "	done."
	else
		message "	Already installed!"
	fi
}

function install_PSD {
	cd ~
	silent mkdir -p $PSD_dir || { err "Can't create $PSD_dir"; exit 1; }
	cd prime_sense
	silent git clone https://github.com/ph4m/SensorKinect.git || { err "Error cloning repository"; exit 1; }
	cd SensorKinect
	silent git checkout unstable
	cd Platform/Linux/CreateRedist
	silent ./RedistMaker
	cd ../Redist/Sensor-Bin-Linux-x64-v5.1.2.1/
	silent sudo ./install.sh
}

function check_PCL {
	local PCL_flag=""
	message "Checking point-clouds library..."
	mapfile -t PCL_flag < <(locate libpcl_common)
	if [[ -z "${PCL_flag[@]// }" ]]; then
		message "Installing point-clouds library..."
		install_PCL
		message "	done."
	else
		message "	Already installed!"
	fi
}

function check_ROS {
	message "Checking ROS installation..."
	local ROS_flag=$(rosversion -d &> /dev/null)
	if [[ "${ROS_flag// }" -eq "${ROS_VER}" ]]; then
		message "	ROS ${ROS_VER} already installed!"
		return 0
	fi
	warn 	"	ROS is not installed"
	warn 	"	Before proceeding, please configure your Ubuntu repositories to allow restricted, universe, and multiverse."
	echo	"	Installing ROS might take up to one hour"
	q_continue
	install_ROS
}

function install_ROS {
	message "	Installing ROS ..."

	message "	Setting up sources..."
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'  || { err "Error while setting up sources."; exit 1; }
	message "	Setting up keys..."
	silent sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key $ROS_KEY  || { err "Error while adding key."; exit 1; }
	message "	Updating package list..."
	silent update
	message "	Installing ROS ..."
	warn	"		Patience (this may take up to one hour)"
	silent install_packages ${ROS_MIN[@]} || { err "Error while installing ROS."; exit 1; }
	message "	Initializing rosdep ..."
	silent sudo rosdep init || { warn "Error initializing rosdep."; }
	silent rosdep update || { warn "Error initializing rosdep."; }
	message "	Setting up environment ..."
	$(source /opt/ros/${ROS_VER}/setup.bash)
	echo "source /opt/ros/${ROS_VER}/setup.bash">> ~/.bashrc
	$(source ~/.bashrc)
	message "	ROS installation complete."
}

function install_PCL {
	silent add_repo ppa:v-launchpad-jochen-sprickerhof-de/pcl
	silent update
	silent install_packages libpcl-all
}

function check_root {
	echo "$(id -u)"
	if [ "$(id -u)" -eq "0" ]; then
		return 0
	fi
	warn "This script requires root privileges."
	silent sudo echo "$(id -u)" || { err "This script must be run as root."; exit 1; }
}

#############################################################################
#
# Main Script
#
#############################################################################
check_root

clear

message "Updating submodules..."
silent git submodule update --init --recursive
message "	done."
echo ""

message "Updating package list..."
update
message "	done."
echo ""

message "Installing dependencies..."
install_packages ${PRERREQ[@]}
message "	done."
echo ""

check_ROS
echo ""

message "Installing other ROS packages..."
install_packages ${ROS_PKG[@]}
message "	done."
echo ""

silent sudo updatedb

check_OpenCV
echo ""

check_PSD
echo ""

check_PCL
echo ""

message "Install complete"