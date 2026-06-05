FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV CC=gcc-11
ENV CXX=g++-11

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        freeglut3-dev \
        g++-11 \
        gcc-11 \
        git \
        libgl1-mesa-dri \
        libgl1-mesa-dev \
        libglfw3 \
        libglu1-mesa-dev \
        libx11-dev \
        libglx-mesa0 \
        libxcursor-dev \
        libxi-dev \
        libxinerama-dev \
        libxrandr-dev \
        make \
        mesa-utils \
        pkg-config \
        python3 \
        python3-pip \
        rsync \
        x11-apps \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /work
