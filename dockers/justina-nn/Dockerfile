from rmartella/justina-opencv:latest

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


USER root
#USER biorobotica
RUN sudo easy_install face_recognition_models Click face_recognition

USER biorobotica
WORKDIR $HOME
