from justina-opencv:latest

WORKDIR $HOME
USER biorobotica

RUN git clone https://github.com/pjreddie/darknet.git
WORKDIR darknet
RUN sed -i 's/GPU=0*/GPU=1/' Makefile \
    && sed -i 's/OPENCV=0/OPENCV=1/' Makefile \
#    && sed -i 's/CUDNN=0/CUDNN=1/' Makefile \
    && sed -i 's/\/usr\/local\/cuda\/lib64/\/usr\/local\/cuda\/lib64 -L\/usr\/local\/cuda-9.0\/targets\/x86_64-linux\/lib\/stubs/' Makefile
	 	
RUN make

RUN wget https://pjreddie.com/media/files/yolov3.weights
RUN wget https://pjreddie.com/media/files/yolov3-tiny.weights
RUN wget https://pjreddie.com/media/files/yolov2.weights
RUN wget https://pjreddie.com/media/files/yolov2-tiny.weights

#Object detectuib tensor flow
USER root
RUN apt-get update && \
    apt-get -y install --no-install-recommends protobuf-compiler python-pil python-lxml python-tk && \
    rm -rf /var/lib/apt/lists/*

USER biorobotica
RUN pip install --user Cython
RUN pip install --user contextlib2
RUN pip install --user jupyter
RUN pip install --user matplotlib
RUN pip install --user pillow
RUN pip install --user lxml

RUN pip install --user imutils

WORKDIR $HOME
RUN git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose
RUN cd openpose && git checkout v1.4.0
RUN mkdir openpose/build && cd openpose/build \
	&& cmake \
	-DCMAKE_BUILD_TYPE="Release" \
	-DCUDA_ARCH=Manual \
	-DCUDA_ARCH_BIN="30 35 50 52 60 61 70" \
#	-DBUILD_CAFFE=OFF \
	-DBUILD_EXAMPLES=ON \
	-DBUILD_DOCS=OFF \
	-DBUILD_SHARED_LIBS=ON \
	-DDOWNLOAD_BODY_25_MODEL=ON \
      	-DDOWNLOAD_BODY_COCO_MODEL=OFF \
      	-DDOWNLOAD_BODY_MPI_MODEL=OFF \
      	-DDOWNLOAD_HAND_MODEL=OFF \
      	-DDOWNLOAD_FACE_MODEL=OFF \
      	-DWITH_3D_RENDERER:BOOL=OFF \
#      	-DCaffe_INCLUDE_DIRS="/opt/caffe/include" \
#      	-DCaffe_LIBS="/opt/caffe/build/lib/libcaffe.so" \
	-DBUILD_PYTHON=ON ../
RUN cd openpose/build && make -j"${nproc}"
USER root
RUN cd openpose/build && make install
USER biorobotica
ENV OPENPOSE_HOME $HOME/openpose

USER root
RUN sudo easy_install face_recognition_models Click face_recognition

USER biorobotica
WORKDIR $HOME
