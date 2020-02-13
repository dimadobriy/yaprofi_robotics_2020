FROM yaprofi-task-base
MAINTAINER Kirill Artemov "kaartemov@itmo.ru"

ENV TZ=Europe/Moscow
ENV DEBIAN_FRONTEND noninteractive
ENV UBUNTU_CODENAME=xenial
ENV ROS_DISTRO=kinetic

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get update | apt-get install -y gnupg2 ca-certificates xauth

# Install ROS additional packages
RUN /bin/bash -c "echo 'deb http://packages.ros.org/ros/ubuntu ${UBUNTU_CODENAME} main' > /etc/apt/sources.list.d/ros-latest.list && \
                  wget -vO - http://packages.ros.org/ros.key | sudo apt-key add - && \
                  apt-get update"

RUN apt-get install -y ros-${ROS_DISTRO}-visp*


# Some cleaning steps
RUN rm -rf /var/lib/apt/lists/*


# COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
# CMD ["bash"]
