from justina-base:latest

USER root

RUN apt-get update \
	&& apt-get install -y pkg-config unzip ffmpeg qtbase5-dev python-dev python3-dev python-numpy python3-numpy
RUN apt-get install -y libopencv-dev libgtk-3-dev libdc1394-22 libdc1394-22-dev libjpeg-dev libpng12-dev libtiff5-dev libjasper-dev
RUN apt-get install -y libavcodec-dev libavformat-dev libswscale-dev libxine2-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev
RUN apt-get install -y libv4l-dev libtbb-dev libfaac-dev libmp3lame-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev
RUN apt-get install -y libvorbis-dev libxvidcore-dev v4l-utils
RUN apt-get install -y liblapacke-dev libopenblas-dev checkinstall
RUN apt-get install -y libgdal-dev
RUN apt-get install -y libatlas-base-dev
RUN easy_install pip

RUN wget https://sourceforge.net/projects/opencvlibrary/files/opencv-unix/3.3.1/opencv-3.3.1.zip
RUN unzip opencv-3.3.1.zip

RUN wget https://github.com/opencv/opencv_contrib/archive/3.3.1.zip
RUN mv 3.3.1.zip opencv_contrib-3.3.1.zip
RUN unzip opencv_contrib-3.3.1.zip

RUN cd opencv-3.3.1
RUN mkdir build
RUN cd build
RUN cmake -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPEPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON -D WITH_VTK=ON -D WITH_OPENNI=ON -D WITH_OPENCL=OFF -D CMAKE_BUILD_TYPE=RELEASE FORCE_VTK=ON -D WITH_CUBLAS=ON -D CUDA_NVCC_FLAGS="-D_FORCE_INLINES" -D WITH_GDAL=ON -D WITH_XINE=ON -D BUILD_EXAMPLES=ON -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-3.3.1/modules ..
RUN make -j4
RUN make install
RUN touch /etc/ld.so.conf.d/opencv.conf
RUN /bin/su -c "echo '/usr/local/lib' >> /etc/ld.so.conf.d/opencv.conf"
RUN ldconfig
