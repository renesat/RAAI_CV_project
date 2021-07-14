FROM nvidia/cuda:10.2-base

ENV MAIN_USER tracker

RUN apt-get update -y && \
    apt-get upgrade -y && \
    apt-get install -y wget sudo

RUN useradd -ms /bin/bash $MAIN_USER && \
    echo "$MAIN_USER:$MAIN_USER" | chpasswd && \
    adduser $MAIN_USER sudo
USER $MAIN_USER 
WORKDIR /home/$MAIN_USER 


RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh && \
    mkdir ~/.conda && \
    bash Miniconda3-latest-Linux-x86_64.sh  -b && \
    rm -f Miniconda3-latest-Linux-x86_64.sh
