FROM osrf/ros:kinetic-desktop-xenial
MAINTAINER Kirill Artemov "kaartemov@itmo.ru"

ENV HTTP_PROXY="http://proxy.ifmo.ru:3128"
ENV HTTPS_PROXY="http://proxy.ifmo.ru:3128"
ENV FTP_PROXY="http://proxy.ifmo.ru:3128"
ENV ALL_PROXY="http://proxy.ifmo.ru:3128"
ENV NO_PROXY="localhost"

ENV TZ=Europe/Moscow
ENV DEBIAN_FRONTEND noninteractive
ENV UBUNTU_CODENAME=xenial
ENV ROS_DISTRO=kinetic
ENV CATKIN_DIR="/home/catkin_ws"

# Install Ubuntu additional packages
RUN apt-get update \
    && apt-get dist-upgrade -y \
    && apt-get install -y --no-install-recommends \
        apt-utils locales libxv1 va-driver-all \
        sudo gnupg2 wget mesa-utils module-init-tools \
        net-tools curl git mc tmux nano vim

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install C++ Libraries for ya-profi
RUN apt-get install -y --no-install-recommends \
        libvisp-dev libvisp-doc \
        libeigen3-dev libeigen3-doc
        
# Install Python2 Libraries for ya-profi
RUN apt-get install -y python-pip python-matplotlib python-numpy python-scipy
RUN apt-get install -y python-rosinstall python-rosinstall-generator \
                        python-wstool build-essential python-catkin-tools

# Install ROS additional packages
RUN /bin/bash -c "echo 'deb http://packages.ros.org/ros/ubuntu ${UBUNTU_CODENAME} main' > /etc/apt/sources.list.d/ros-latest.list && \
                  wget -qO - http://packages.ros.org/ros.key | sudo apt-key add - && \
                  apt-get update"
RUN apt-get install -y ros-${ROS_DISTRO}-controller-manager \
                        ros-${ROS_DISTRO}-ros-control \
                        ros-${ROS_DISTRO}-joint-state-controller

# Install CoppeliaSim (prev. V-REP) simpulator
RUN wget http://coppeliarobotics.com/files/CoppeliaSim_Edu_V4_0_0_Ubuntu16_04.tar.xz
RUN tar -xf CoppeliaSim_Edu_V4_0_0_Ubuntu16_04.tar.xz && \
    mv CoppeliaSim_Edu_V4_0_0_Ubuntu16_04 /opt/csim && \
    cp /opt/csim/compiledRosPlugins/libsimExtROSInterface.so /opt/csim/ && \
    echo 'export QT_DEBUG_PLUGINS=1' >> ~/.bashrc && \
    echo 'export PATH=/opt/csim/:${PATH}' >> ~/.bashrc



# Some cleaning steps
RUN rm -rf /var/lib/apt/lists/*
RUN rm -rf CoppeliaSim_Edu_V4_0_0_Ubuntu16_04.tar.xz          
RUN rm -rf NVIDIA-Linux-x86_64-${NVIDIA_DRIVER_VERSION}.run

RUN echo 'source "/opt/ros/kinetic/setup.bash"' >> ~/.bashrc
RUN echo 'source "/home/root/catkin_ws/devel/setup.bash"' >> ~/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]

