FROM ubuntu:latest

WORKDIR /var/sw
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y make
RUN apt-get update && apt-get install -y libopencv-dev
RUN apt-get update && apt-get -y install cmake protobuf-compiler
RUN apt-get update && apt-get install -y build-essential  
RUN apt-get update && apt-get install -y openocd
RUN apt-get update && apt-get install -y gcc-arm-none-eabi
RUN apt-get update && apt-get install -y usbutils
RUN apt-get update && apt-get install -y gdb-multiarch
RUN apt-get update && apt-get install -y clangd
RUN apt-get update && apt-get install -y git
RUN git config --global user.name "MedvedevDavid"
RUN git config --global user.email "medvedevdavid@gmail.com"


CMD ["/bin/bash"]