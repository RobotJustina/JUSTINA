FROM tensorflow/tensorflow:latest-gpu

# Create the home directory for the new app user.
RUN mkdir -p /home/biorobotica
ENV APP_HOME=/home/biorobotica

# Create an app user so our program doesn't run as root.
RUN  useradd biorobotica && echo "biorobotica:biorobotica" | chpasswd && adduser biorobotica sudo

# install packages
RUN apt-get update && apt-get install -y -q \
	dirmngr \
	gnupg2 \
	lsb-release \
	sudo \
	vim \
	net-tools \
	git \
	pkg-config \
	build-essential \
	&& rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# install ros packages
ENV ROS_DISTRO kinetic
RUN apt-get update && apt-get install -y \
    ros-kinetic-desktop-full \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get -y install python-rosinstall 

# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

#RUN apt-get update && apt-get install -y --no-install-recommends \
#        build-essential \
#        cuda-command-line-tools-9-0 \
#        cuda-cublas-dev-9-0 \
#        cuda-cudart-dev-9-0 \
#        cuda-cufft-dev-9-0 \
#        cuda-curand-dev-9-0 \
#        cuda-cusolver-dev-9-0 \
#        cuda-cusparse-dev-9-0 \
#        curl \
#        git \
#        libcudnn7=7.1.4.18-1+cuda9.0 \
#        libcudnn7-dev=7.1.4.18-1+cuda9.0 \
#        libcurl3-dev \
#        libfreetype6-dev \
#        libhdf5-serial-dev \
#        libpng12-dev \
#        libzmq3-dev \
#        pkg-config \
#        python-dev \
#        rsync \
#        software-properties-common \
#        unzip \
#        zip \
#        zlib1g-dev \
#        wget \
#        && \
#    rm -rf /var/lib/apt/lists/* && \
#    find /usr/local/cuda-9.0/lib64/ -type f -name 'lib*_static.a' -not -name 'libcudart_static.a' -delete && \
#    rm /usr/lib/x86_64-linux-gnu/libcudnn_static_v7.a

#RUN curl -fSsL -O https://bootstrap.pypa.io/get-pip.py && \
#    python get-pip.py && \
#    rm get-pip.py

#RUN pip --no-cache-dir install \
#        Pillow \
#        h5py \
#        ipykernel \
#        jupyter \
#        matplotlib \
#        mock \
#        numpy \
#        scipy \
#        sklearn \
#        pandas \
#        && \
#    python -m ipykernel.kernelspe

# Set up Bazel.

# Running bazel inside a `docker build` command causes trouble, cf:
#   https://github.com/bazelbuild/bazel/issues/134
# The easiest solution is to set up a bazelrc file forcing --batch.
#RUN echo "startup --batch" >>/etc/bazel.bazelrc
# Similarly, we need to workaround sandboxing issues:
#   https://github.com/bazelbuild/bazel/issues/418
#RUN echo "build --spawn_strategy=standalone --genrule_strategy=standalone" \
#    >>/etc/bazel.bazelrc
# Install the most recent bazel release.
#ENV BAZEL_VERSION 0.11.0
#WORKDIR /
#RUN mkdir /bazel && \
#    cd /bazel && \
#    curl -H "User-Agent: Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/57.0.2987.133 Safari/537.36" -fSsL -O https://github.com/bazelbuild/bazel/releases/download/$BAZEL_VERSION/bazel-$BAZEL_VERSION-installer-linux-x86_64.sh && \
#    curl -H "User-Agent: Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/57.0.2987.133 Safari/537.36" -fSsL -o /bazel/LICENSE.txt https://raw.githubusercontent.com/bazelbuild/bazel/master/LICENSE && \
#    chmod +x bazel-*.sh && \
#    ./bazel-$BAZEL_VERSION-installer-linux-x86_64.sh && \
#    cd / && \
#    rm -f /bazel/bazel-$BAZEL_VERSION-installer-linux-x86_64.sh

# Download and build TensorFlow.
#WORKDIR /tensorflow
#RUN git clone --branch=r1.9 --depth=1 https://github.com/tensorflow/tensorflow.git .


# Configure the build for our CUDA configuration.
#ENV CI_BUILD_PYTHON python
#ENV LD_LIBRARY_PATH /usr/local/cuda/extras/CUPTI/lib64:$LD_LIBRARY_PATH
#ENV TF_NEED_CUDA 1
#ENV TF_CUDA_COMPUTE_CAPABILITIES=3.0,3.5,5.2,6.0,6.1
#ENV TF_CUDA_VERSION=9.0
#ENV TF_CUDNN_VERSION=7

#RUN ln -s /usr/local/cuda/lib64/stubs/libcuda.so /usr/local/cuda/lib64/stubs/libcuda.so.1 && \
#    LD_LIBRARY_PATH=/usr/local/cuda/lib64/stubs:${LD_LIBRARY_PATH} \
#    tensorflow/tools/ci_build/builds/configured GPU \
#    bazel build -c opt --copt=-mavx --config=cuda \
#	--cxxopt="-D_GLIBCXX_USE_CXX11_ABI=0" \
#        tensorflow/tools/pip_package:build_pip_package && \
#    rm /usr/local/cuda/lib64/stubs/libcuda.so.1 && \
#    bazel-bin/tensorflow/tools/pip_package/build_pip_package /tmp/pip && \
#    pip --no-cache-dir install --upgrade /tmp/pip/tensorflow-*.whl && \
#    rm -rf /tmp/pip && \
#    rm -rf /root/.cache
# Clean up pip wheel and Bazel cache when done.

#WORKDIR /root

# Set the home directory to our app user's home.
ENV HOME=/home/biorobotica

# SETTING UP THE APP ##
WORKDIR $APP_HOME

# Copy in the application code.
ADD . $APP_HOME

# Chown all the files to the app user.
RUN chown -R biorobotica:biorobotica $APP_HOME

# Change to the app user.
USER biorobotica

