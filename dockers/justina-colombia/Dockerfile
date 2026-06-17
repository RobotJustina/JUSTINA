ARG ROS_BASE_IMAGE=ros:kinetic-ros-base-xenial
FROM ${ROS_BASE_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=justina
ARG USER_UID=1000
ARG USER_GID=1000

ENV TZ=America/Bogota \
    LANG=es_CO.UTF-8 \
    LANGUAGE=es_CO:es \
    LC_ALL=es_CO.UTF-8 \
    LC_TIME=es_CO.UTF-8 \
    ROS_DISTRO=kinetic \
    JUSTINA_WS=/workspace/JUSTINA/catkin_ws

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        bash-completion \
        build-essential \
        ca-certificates \
        git \
        libjack-jackd2-dev \
        libopencv-dev \
        libpcl-dev \
        locales \
        python-catkin-tools \
        python-rosdep \
        qtbase5-dev \
        ros-kinetic-amcl \
        ros-kinetic-desktop-full \
        ros-kinetic-dynamixel-sdk \
        ros-kinetic-fake-localization \
        ros-kinetic-gmapping \
        ros-kinetic-joy \
        ros-kinetic-laser-geometry \
        ros-kinetic-map-server \
        ros-kinetic-pcl-ros \
        ros-kinetic-pointcloud-to-laserscan \
        ros-kinetic-serial \
        ros-kinetic-sound-play \
        ros-kinetic-tf \
        ros-kinetic-tf2-bullet \
        ros-kinetic-tf-conversions \
        ros-kinetic-urg-node \
        sudo \
        tzdata \
    && locale-gen es_CO.UTF-8 \
    && update-locale LANG=es_CO.UTF-8 LANGUAGE=es_CO:es LC_TIME=es_CO.UTF-8 \
    && ln -snf /usr/share/zoneinfo/America/Bogota /etc/localtime \
    && echo America/Bogota > /etc/timezone \
    && groupadd --gid "${USER_GID}" "${USERNAME}" \
    && useradd --uid "${USER_UID}" --gid "${USER_GID}" --create-home --shell /bin/bash "${USERNAME}" \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}" \
    && rm -rf /var/lib/apt/lists/*

COPY container-entrypoint.sh /usr/local/bin/container-entrypoint
COPY build-workspace.sh /usr/local/bin/build-justina-workspace
RUN chmod 0755 /usr/local/bin/container-entrypoint /usr/local/bin/build-justina-workspace

USER justina
WORKDIR /workspace/JUSTINA

ENTRYPOINT ["/usr/local/bin/container-entrypoint"]
CMD ["bash"]
