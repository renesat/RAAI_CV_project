FROM nvidia/cuda:11.4.0-devel-ubuntu20.04

ARG DOCKER_USER=tracker
ARG USER_PASS=tracker

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update -y && \
    apt-get upgrade -y && \
    apt-get install -y wget sudo lsb-release

# ROS install:
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-get install -y curl && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - && \
    apt-get update -y && \
    apt-get install -y ros-noetic-ros-base && \
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

RUN . /opt/ros/noetic/setup.sh && \
    apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential && \
    rosdep init && \
    rosdep update

RUN apt-get clean all

# Create user:
RUN useradd -ms /bin/bash $DOCKER_USER && \
    echo "$DOCKER_USER:$USER_PASS" | chpasswd && \
    adduser $DOCKER_USER sudo
USER $DOCKER_USER
WORKDIR /home/$DOCKER_USER

# ROS init:
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    . /opt/ros/noetic/setup.sh

# Conda install:
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh && \
    mkdir ~/.conda && \
    bash Miniconda3-latest-Linux-x86_64.sh  -b && \
    rm -f Miniconda3-latest-Linux-x86_64.sh

ENV DOCKER_USER $DOCKER_USER
