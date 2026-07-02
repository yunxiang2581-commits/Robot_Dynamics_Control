FROM openloong-ubuntu22-build:local

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        fluxbox \
        mesa-utils \
        novnc \
        x11-utils \
        websockify \
        x11vnc \
        xvfb \
    && rm -rf /var/lib/apt/lists/*
