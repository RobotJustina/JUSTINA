# Need nvidia-docker to run https://github.com/NVIDIA/nvidia-docker
# Image from https://gitlab.com/nvidia/cuda/
FROM stereolabs/zed:ubuntu1604-cuda9.0-zed2.6-gl

# The runtime image might be enough, depending on the need

# Setup ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt-get update -y
RUN apt-get install ros-kinetic-desktop-full -y


#RUN adduser --gecos "ROS User" --home /home/ros --disabled-password ros && \
#    usermod -a -G dialout ros && \
#    echo "ros ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/99_aptget

RUN apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential nano cmake usbutils git -y


# switch to user ros, but the HOME is still /, not /home/ros
#RUN chown ros /usr/local/zed/ -R
#USER ros

# setup ros env
RUN mkdir /home/ros/catkin_ws/src -p
WORKDIR /home/ros/catkin_ws/src
RUN git clone https://github.com/stereolabs/zed-ros-wrapper


#RUN rosdep init && rosdep update && echo "source \"/opt/ros/kinetic/setup.bash\"" >> /etc/bash.bashrc
#RUN . /opt/ros/kinetic/setup.sh
WORKDIR /home/ros/catkin_ws
RUN . /opt/ros/kinetic/setup.sh && catkin_make 
#RUN catkin_make
RUN echo "source \"/home/ros/catkin_ws/devel/setup.bash\"" >> /etc/bash.bashrc



# cd /home/ros default
#WORKDIR /home/ros

# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
