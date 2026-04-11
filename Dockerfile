FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive

#This takes a very long time
RUN apt-get update && apt-get install -y \
    libpcl-dev

RUN apt update -y && apt install -y \
    clang \
    cmake \
    libeigen3-dev \
    libomp-dev \
    curl

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --default-toolchain stable

ENV CC=clang
ENV CXX=clang++

WORKDIR /app
CMD ["/bin/bash"]
