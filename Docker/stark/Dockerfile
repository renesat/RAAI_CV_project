FROM renesat/raai_tracking:common

USER root
RUN apt-get install -y ninja-build libturbojpeg git build-essential \
                       ffmpeg libsm6 libxext6

RUN mkdir -p /workspace && \
    echo $DOCKER_USER && \
    chown $DOCKER_USER /workspace
WORKDIR /workspace


USER $DOCKER_USER
RUN git clone "https://github.com/researchmm/Stark"
RUN cd /workspace/Stark && \
    . ~/miniconda3/etc/profile.d/conda.sh && \
    conda create -y -n stark python=3.6 && \
    conda activate stark && \
    bash install.sh

RUN mkdir /workspace/data
