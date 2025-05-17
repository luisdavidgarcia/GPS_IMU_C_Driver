FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
    python3 \
    python3-pip \
    python3-venv \
    build-essential \
    cmake \
    clang \
    clangd \
    clang-tidy \
    cppcheck \
    libi2c-dev \
    libeigen3-dev \
    gdb \
    valgrind \
    lcov \
    git \
    && apt clean && rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade pip && \
    pip3 install matplotlib numpy

WORKDIR /app

CMD ["/bin/bash"]
