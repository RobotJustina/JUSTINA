from rmartella/justina-opencv:latest

WORKDIR $HOME
USER biorobotica

RUN git clone https://github.com/pjreddie/darknet.git
WORKDIR darknet
RUN sed -i 's/GPU=0*/GPU=1/' Makefile \
    && sed -i 's/OPENCV=0/OPENCV=1/' Makefile \
    && sed -i 's/CUDNN=0/CUDNN=1/' Makefile \
    && sed -i 's/\/usr\/local\/cuda\/lib64/\/usr\/local\/cuda\/lib64 -L\/usr\/local\/cuda-9.0\/targets\/x86_64-linux\/lib\/stubs/' Makefile
	 	
RUN make

RUN wget https://pjreddie.com/media/files/yolov3.weights
RUN wget https://pjreddie.com/media/files/yolov3-tiny.weights
RUN wget https://pjreddie.com/media/files/yolov2.weights
RUN wget https://pjreddie.com/media/files/yolov2-tiny.weights

WORKDIR $HOME
