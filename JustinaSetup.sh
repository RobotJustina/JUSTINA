echo "INSTALLING  DEPENDENCIES..."
sudo apt get update
sudo apt-get install freeglut3-dev pkg-config build-essential libxmu-dev libxi-dev libusb-1.0-0-dev doxygen graphviz mono-complete
sudo apt-get install build-essential libgtk2.0-dev libjpeg-dev libtiff4-dev libjasper-dev libopenexr-dev cmake python-dev python-numpy python-tk libtbb-dev libeigen3-dev yasm libfaac-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev libqt4-dev libqt4-opengl-dev sphinx-common texlive-latex-extra libv4l-dev libdc1394-22-dev libavcodec-dev libavformat-dev libswscale-dev default-jdk ant libvtk5-qt4-dev
cd ~
echo "INSTALLING OPENCV 2.4.9"
wget http://sourceforge.net/projects/opencvlibrary/files/opencv-unix/2.4.9/opencv-2.4.9.zip
unzip opencv-2.4.9.zip
cd opencv-2.4.9
mkdir build
cd build
cmake -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON -D WITH_VTK=ON -D WITH_OPENNI=ON -D WITH_OPENCL=OFF ..
make -j4
sudo make install
sudo echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf
sudo ldconfig
echo "OpenCV is ready to use"
echo "Installing PrimeSense drivers..."
mkdir -p prime_sense
cd prime_sense
git clone https://github.com/ph4m/SensorKinect.git
git checkout unstable
cd SensorKinect/Platform/Linux/CreateRedist
./RedistMaker
cd ../Redist/Sensor-Bin-Linux-x64-v5.1.2.1/
sudo ./install.sh
echo "Prime sense drivers correctly installed"
echo "INSTALLING POINT CLOUDS LIBRARY..."
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-all
echo "Installing OTHER NEEDED ROS PACKAGES..."
sudo apt-get install ros-indigo-hokuyo-node
sudo apt-get install ros-indigo-joy
sudo apt-get install ros-indigo-openni-camera
sudo apt-get install ros-indigo-openni-launch
sudo apt-get install ros-indigo-openni2-camera
sudo apt-get install ros-indigo-openni2-launch
sudo apt-get install ros-indigo-amcl
sudo apt-get install ros-indigo-tf2-bullet
sudo apt-get install ros-indigo-fake-localization
sudo apt-get install ros-indigo-map-server
sudo apt-get install ros-indigo-sound-play
sudo apt-get install ros-indigo-pocketsphinx
echo "INSTALLING OTHER NEEDED LIBRARIES..."
sudo apt-get install libzbar-dev
sudo apt-get purge jackd2 jackd jackd2-firewire libjack-jackd2-dev pulseaudio-module-jack qjackctl
echo "PLEASE SAY YES WHEN ASKED FOR REAL TIME"
sudo apt-get install jackd2 libjack-jackd2-dev pulseaudio-module-jack qjackctl
echo "YOUR NEW JUSTINA-SOFTWARE IS READY TO USE :D"
