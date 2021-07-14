FROM renesat/raai_tracking:common

USER root
RUN apt-get install -y ninja-build libturbojpeg git build-essential \
                       ffmpeg libsm6 libxext6

RUN mkdir -p /workspace && \
    chown $MAIN_USER /workspace
WORKDIR /workspace


USER $MAIN_USER
RUN git clone --recurse-submodules "https://github.com/visionml/pytracking"
COPY install.sh /workspace/pytracking
RUN cd /workspace/pytracking && \
    bash install.sh ~/miniconda3 pytracking

RUN mkdir /workspace/data
