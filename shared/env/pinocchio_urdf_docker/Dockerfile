FROM mambaorg/micromamba:1.5.10

USER root

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    build-essential \
    libgl1 \
    libglib2.0-0 \
    libxrender1 \
    libxext6 \
    libsm6 \
    libosmesa6 \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

USER $MAMBA_USER

WORKDIR /workspace

COPY --chown=$MAMBA_USER:$MAMBA_USER environment.yml /tmp/environment.yml

RUN micromamba install -y -n base -f /tmp/environment.yml \
    && micromamba clean --all --yes

COPY --chown=$MAMBA_USER:$MAMBA_USER . /workspace

ENV PYTHONUNBUFFERED=1
ENV MUJOCO_GL=osmesa

CMD ["bash"]
