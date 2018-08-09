from rmartella/justina-base:latest

USER root

RUN apt-get update \
	&& apt-get install -y pkg-config unzip ffmpeg qtbase5-dev python-dev python3-dev python-numpy python3-numpy
RUN apt-get install -y libavcodec-dev libavformat-dev libswscale-dev libxine2-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev
RUN apt-get install -y libv4l-dev libtbb-dev libfaac-dev libmp3lame-dev libopencore-amrnb-dev libopencore-amrwb-dev libtheora-dev
RUN apt-get install -y libvorbis-dev libxvidcore-dev v4l-utils
RUN apt-get install -y liblapacke-dev libopenblas-dev checkinstall
RUN apt-get install -y libgdal-dev
RUN apt-get install -y libatlas-base-dev
RUN easy_install pip

RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates \
        cmake \
        git \
        libatlas-base-dev \
        libatlas-dev \
        libboost-all-dev \
        libgflags-dev \
        libgoogle-glog-dev \
        libhdf5-dev \
        libprotobuf-dev \
        pkg-config \
        protobuf-compiler \
        python-yaml \
        python-six \
        wget && \
    rm -rf /var/lib/apt/lists/*

ENV CUDA_ARCH_BIN "35 52 60 61 70"
ENV CUDA_ARCH_PTX "70"

ENV NCCL_VERSION 2.2.13
RUN apt-get update && apt-get install -y --no-install-recommends \
        cuda-libraries-$CUDA_PKG_VERSION \
        cuda-cublas-9-0=9.0.176.3-1 \
        libnccl2=$NCCL_VERSION-1+cuda9.0 && \
    apt-mark hold libnccl2 && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
        cuda-libraries-dev-$CUDA_PKG_VERSION \
        cuda-nvml-dev-$CUDA_PKG_VERSION \
        cuda-minimal-build-$CUDA_PKG_VERSION \
        cuda-command-line-tools-$CUDA_PKG_VERSION \
        cuda-core-9-0=9.0.176.3-1 \
        cuda-cublas-dev-9-0=9.0.176.3-1 \
        libnccl-dev=$NCCL_VERSION-1+cuda9.0 && \
    rm -rf /var/lib/apt/lists/*

USER biorobotica
WORKDIR /home/biorobotica

RUN wget https://sourceforge.net/projects/opencvlibrary/files/opencv-unix/3.3.1/opencv-3.3.1.zip
RUN unzip opencv-3.3.1.zip

RUN wget https://github.com/opencv/opencv_contrib/archive/3.3.1.zip
RUN mv 3.3.1.zip opencv_contrib-3.3.1.zip
RUN unzip opencv_contrib-3.3.1.zip

RUN mkdir /home/biorobotica/opencv-3.3.1/build
WORKDIR /home/biorobotica/opencv-3.3.1/build
#RUN cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON \
#          -DWITH_CUDA=ON -DWITH_CUFFT=OFF -DCUDA_ARCH_BIN="${CUDA_ARCH_BIN}" -DCUDA_ARCH_PTX="${CUDA_ARCH_PTX}" \
#          -DWITH_JPEG=ON -DBUILD_JPEG=ON -DWITH_PNG=ON -DBUILD_PNG=ON \
#          -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF -DWITH_FFMPEG=OFF -DWITH_GTK=OFF \
#          -DWITH_OPENCL=OFF -DWITH_QT=OFF -DWITH_V4L=OFF -DWITH_JASPER=OFF \
#          -DWITH_1394=OFF -DWITH_TIFF=OFF -DWITH_OPENEXR=OFF -DWITH_IPP=OFF -DWITH_WEBP=OFF \
#          -DBUILD_opencv_superres=OFF -DBUILD_opencv_java=OFF -DBUILD_opencv_python2=OFF \
#          -DBUILD_opencv_videostab=OFF -DBUILD_opencv_apps=OFF -DBUILD_opencv_flann=OFF \
#          -DBUILD_opencv_ml=OFF -DBUILD_opencv_photo=OFF -DBUILD_opencv_shape=OFF \
#          -DBUILD_opencv_cudabgsegm=OFF -DBUILD_opencv_cudaoptflow=OFF -DBUILD_opencv_cudalegacy=OFF \
#          -DCUDA_NVCC_FLAGS="-O3" -DCUDA_FAST_MATH=ON -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib-3.3.1/modules ~/opencv-3.3.1


#RUN cmake -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPEPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON -D WITH_VTK=ON -D WITH_OPENNI=ON -D WITH_OPENCL=OFF -D CMAKE_BUILD_TYPE=RELEASE FORCE_VTK=ON -D WITH_CUBLAS=ON -D CUDA_NVCC_FLAGS="-D_FORCE_INLINES" -D WITH_GDAL=ON -D WITH_XINE=ON -D BUILD_EXAMPLES=ON -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib-3.3.1/modules ~/opencv-3.3.1
RUN cmake -D WITH_TBB=ON -D BUILD_NEW_PYTHON_SUPPORT=ON -D WITH_V4L=ON -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=OFF -D BUILD_EXAMPLES=OFF -D WITH_QT=ON -D WITH_OPEPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_QT=ON -D WITH_OPENGL=ON -D WITH_VTK=ON -D WITH_OPENNI=ON -D WITH_OPENCL=OFF -D CMAKE_BUILD_TYPE=RELEASE FORCE_VTK=ON -D WITH_CUBLAS=ON -D -DBUILD_opencv_cudalegacy=OFF CUDA_NVCC_FLAGS="-D_FORCE_INLINES" -D WITH_GDAL=ON -D WITH_XINE=ON -D BUILD_EXAMPLES=OFF -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib-3.3.1/modules ..
RUN make -j4

USER root

RUN make install
RUN touch /etc/ld.so.conf.d/opencv.conf
RUN /bin/su -c "echo '/usr/local/lib' >> /etc/ld.so.conf.d/opencv.conf"
RUN ldconfig

user biorobotica

WORKDIR /home/biorobotica/
RUN wget https://github.com/davisking/dlib/archive/v19.6.zip
RUN unzip /home/biorobotica/v19.6.zip -d /home/biorobotica/
WORKDIR /home/biorobotica/dlib-19.6
RUN mkdir /home/biorobotica/dlib-19.6/build
WORKDIR /home/biorobotica/dlib-19.6/build
RUN cmake ..
RUN make -j4
USER root

RUN make install

user biorobotica

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
WORKDIR /home/biorobotica/
