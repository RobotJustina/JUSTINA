FROM ros:indigo-ros-base-trusty

# install packages
RUN apt-get update && apt-get install -y -q \
	build-essential \
	gnupg2 \
	lsb-release \
	sudo \
	vim \
	net-tools \
	git \
	bash-completion \
	wget \
	iputils-ping \
	iproute2 \
	netcat \
	terminator \
	xauth \
	openssh-server \
	pcmanfm \
	cmake \
	libboost-all-dev \
	libprotoc-dev \
	protobuf-compiler \
	libssl-dev \
	ros-$ROS_DISTRO-map-server \
	ros-indigo-rqt-gui \
	ros-indigo-rqt-gui-cpp

RUN easy_install python-Levenshtein


#setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# Create the home directory for the new app user.
RUN mkdir -p /home/biorobotica

# Create an app user so our program doesn't run as root.
RUN  useradd biorobotica && echo "biorobotica:biorobotica" | chpasswd && adduser biorobotica sudo
# Set the home directory to our app user's home.
ENV HOME=/home/biorobotica

# Chown all the files to the app user.
RUN chown -R biorobotica:biorobotica $HOME

# Change to the app user.
USER biorobotica


