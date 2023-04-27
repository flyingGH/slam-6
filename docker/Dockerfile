FROM ubuntu:20.04 AS cpp-base
# FROM nvidia/cuda:11.3.1-cudnn8-devel-ubuntu20.04

WORKDIR /root

#--------------#
# Localization #
#--------------#

ENV DEBIAN_FRONTEND="noninteractive" 
RUN apt-get update && apt-get install -y --no-install-recommends \
      tzdata=2023c-0ubuntu0.20.04.0 \
      locales=2.31-0ubuntu9.9 \
      fonts-takao=00303.01-3ubuntu1 \
      && apt-get clean \
      && rm -rf /var/lib/apt/lists/*
ENV TZ=Asia/Tokyo
ENV LANG=ja_JP.UTF-8
ENV LANGUAGE=ja_JP:ja
ENV LC_ALL=ja_JP.UTF-8
RUN locale-gen ja_JP.UTF-8 \
      && localedef -f UTF-8 -i ja_JP ja_JP.utf8

#-----------#
# Dev Tools #
#-----------#

# Install CMake 3.21.6 for g2o
# CLion supports CMake 2.8.11~3.21.x
RUN apt-get update && apt-get install -y --no-install-recommends \
      git=1:2.25.1-1ubuntu3.10 \
      build-essential=12.8ubuntu1.1 \
      libssl-dev=1.1.1f-1ubuntu2.18 \
      # needed for git clone of cmake source
      ca-certificates=20211016ubuntu0.20.04.1 \
      && apt-get clean \
      && rm -rf /var/lib/apt/lists/*
RUN git clone https://gitlab.kitware.com/cmake/cmake.git -b v3.21.6 \
      && mkdir /root/cmake/build
WORKDIR /root/cmake/build
RUN ../bootstrap && make -j"$(nproc)" && make install
WORKDIR /root

# use "apt list hoge -a" to check versions
RUN apt-get update && apt-get install -y --no-install-recommends \
      gcc=4:9.3.0-1ubuntu2 \
      g++=4:9.3.0-1ubuntu2 \
      gdb=9.2-0ubuntu1~20.04.1 \
      clang=1:10.0-50~exp1 \
      make=4.2.1-1.2 \
      ninja-build=1.10.0-1build1 \
      autoconf=2.69-11.1 \
      automake=1:1.16.1-4ubuntu6 \
      locales-all=2.31-0ubuntu9.9 \
      dos2unix=7.4.0-2 \
      rsync=3.1.3-8ubuntu0.5 \
      python3=3.8.2-0ubuntu2 \
      && apt-get clean \
      && rm -rf /var/lib/apt/lists/

#---------------#
# Source Builds #
#---------------#

FROM cpp-base AS cpp-builder

RUN apt-get update && apt-get install -y --no-install-recommends \
      # libboost-all-dev is needed for PCL
      libboost-all-dev=1.71.0.0ubuntu2 \ 
      libopencv-dev=4.2.0+dfsg-5 \
      && apt-get clean \
      && rm -rf /var/lib/apt/lists/*

# glog
ARG REPO_NAME="glog"
RUN git clone https://github.com/google/glog.git -b v0.5.0 \
      && mkdir /root/"$REPO_NAME"/build
WORKDIR /root/"$REPO_NAME"/build
RUN cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF -DBUILD_TESTING=OFF .. \
      && make -j"$(nproc)" && make install
WORKDIR /root
RUN rm -rf /root/"$REPO_NAME"

# gflags    
ARG REPO_NAME="gflags"
RUN git clone https://github.com/gflags/gflags.git -b v2.2.2 \
      && mkdir /root/"$REPO_NAME"/build
WORKDIR /root/"$REPO_NAME"/build
RUN cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF -DBUILD_TESTING=OFF .. \
      && make -j"$(nproc)" && make install
WORKDIR /root
RUN rm -rf /root/"$REPO_NAME"

# gtest
ARG REPO_NAME="googletest"
RUN git clone https://github.com/google/googletest.git -b release-1.11.0 \
      && mkdir /root/"$REPO_NAME"/build
WORKDIR /root/"$REPO_NAME"/build
RUN cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF .. \
      && make -j"$(nproc)" && make install
WORKDIR /root
RUN rm -rf /root/"$REPO_NAME"

# Eigen3
ARG REPO_NAME="eigen"
RUN git clone https://gitlab.com/libeigen/eigen.git -b 3.4-rc1 \
      && mkdir /root/"$REPO_NAME"/build
WORKDIR /root/"$REPO_NAME"/build
RUN cmake -DCMAKE_BUILD_TYPE=Release .. \
      && make -j"$(nproc)" && make install
WORKDIR /root
RUN rm -rf /root/"$REPO_NAME"

# ceres-solver
RUN apt-get update && apt-get install -y --no-install-recommends \
      libatlas-base-dev=3.10.3-8ubuntu7 \
      libsuitesparse-dev=1:5.7.1+dfsg-2 \
      && apt-get clean \
      && rm -rf /var/lib/apt/lists/*
ARG REPO_NAME="ceres-solver"
RUN git clone https://github.com/ceres-solver/ceres-solver.git -b 2.1.0 \
      && mkdir /root/"$REPO_NAME"/build
WORKDIR /root/"$REPO_NAME"/build
RUN cmake -DCMAKE_BUILD_TYPE=Release .. && make -j"$(nproc)" && make install
WORKDIR /root
RUN rm -rf /root/"$REPO_NAME"

# g2o
RUN apt-get update && apt-get install -y --no-install-recommends \
      libsuitesparse-dev=1:5.7.1+dfsg-2 \
      qtdeclarative5-dev=5.12.8-0ubuntu1 \
      qt5-qmake=5.12.8+dfsg-0ubuntu2.1 \
      libqglviewer-dev-qt5=2.6.3+dfsg2-6build1 \
      && apt-get clean \
      && rm -rf /var/lib/apt/lists/*
ARG REPO_NAME="g2o"
RUN git clone https://github.com/RainerKuemmerle/g2o.git \
      && mkdir /root/"$REPO_NAME"/build
WORKDIR /root/"$REPO_NAME"/build
RUN cmake -DCMAKE_BUILD_TYPE=Release .. && make -j"$(nproc)" && make install
WORKDIR /root
RUN rm -rf /root/"$REPO_NAME"

# Sophus
ARG REPO_NAME="Sophus"
RUN git clone https://github.com/strasdat/Sophus.git \
      && mkdir /root/"$REPO_NAME"/build
WORKDIR /root/"$REPO_NAME"/build
RUN cmake -DCMAKE_BUILD_TYPE=Release .. && make -j"$(nproc)" && make install
WORKDIR /root
RUN rm -rf /root/"$REPO_NAME"

# PCL
RUN apt-get update && apt-get install -y --no-install-recommends \
      libusb-1.0-0-dev=2:1.0.23-2build1 \
      libflann-dev=1.9.1+dfsg-9build1 \
      libvtk7-dev=7.1.1+dfsg2-2ubuntu1 \
      libpcap-dev=1.9.1-3 \
      && apt-get clean \
      && rm -rf /var/lib/apt/lists/*
ARG REPO_NAME="pcl"
RUN git clone https://github.com/PointCloudLibrary/pcl.git -b pcl-1.12.1 \
      && mkdir /root/"$REPO_NAME"/build
WORKDIR /root/"$REPO_NAME"/build
RUN cmake -DCMAKE_BUILD_TYPE=Release .. && make -j"$(nproc)" && make install
WORKDIR /root
RUN rm -rf /root/"$REPO_NAME"

# Open3D
RUN apt-get update && apt-get install -y --no-install-recommends \
      # Open3D
      xorg-dev=1:7.7+19ubuntu14 \
      libxcb-shm0=1.14-2 \
      libglu1-mesa-dev=9.0.1-1build1 \
      python3-dev=3.8.2-0ubuntu2 \
      # Filament build-from-source
      clang=1:10.0-50~exp1 \
      libc++-dev=1:10.0-50~exp1 \
      libc++abi-dev=1:10.0-50~exp1 \
      libsdl2-dev=2.0.10+dfsg1-3 \
      ninja-build=1.10.0-1build1 \
      libxi-dev=2:1.7.10-0ubuntu1 \
      # ML
      libtbb-dev=2020.1-2 \
      # Headless rendering
      libosmesa6-dev=21.2.6-0ubuntu0.1~20.04.2 \
      && apt-get clean \
      && rm -rf /var/lib/apt/lists/*
ARG REPO_NAME="Sophus"
RUN git clone https://github.com/isl-org/Open3D.git -b v0.17.0 \
      && mkdir /root/"$REPO_NAME"/build
WORKDIR /root/"$REPO_NAME"/build
RUN cmake -DCMAKE_BUILD_TYPE=Release .. && make -j"$(nproc)" && make install
WORKDIR /root
RUN rm -rf /root/"$REPO_NAME"

# pybind11
RUN git clone https://github.com/pybind/pybind11.git \
      && mkdir /root/"$REPO_NAME"/build
WORKDIR /root/"$REPO_NAME"/build
RUN cmake -DCMAKE_BUILD_TYPE=Release .. && make -j"$(nproc)" && make install
WORKDIR /root

# matplotlib-cpp
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-matplotlib=3.1.2-1ubuntu4 \
      python3-numpy=1:1.17.4-5ubuntu3.1 \
      python3-dev=3.8.2-0ubuntu2 \
      wget=1.20.3-1ubuntu2 \
      && apt-get clean \
      && rm -rf /var/lib/apt/lists/*
RUN wget -nv https://github.com/lava/matplotlib-cpp/raw/master/matplotlibcpp.h -P /usr/local/include

#--------#
# Runner #
#--------#

FROM cpp-base AS runner

# for OpenCV Mat debugger
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3=3.8.2-0ubuntu2 \
      python3-pip=20.0.2-5ubuntu1.8 \
      python3-numpy=1:1.17.4-5ubuntu3.1 \
      && apt-get clean \
      && rm -rf /var/lib/apt/lists/*
# Place gdb scripts
COPY gdbinit.sh /root/.gdbinit
COPY gdb /root/gdb/

RUN apt-get update && apt-get install -y --no-install-recommends \
      # libboost-all-dev is needed for PCL
      libboost-all-dev=1.71.0.0ubuntu2 \ 
      libopencv-dev=4.2.0+dfsg-5 \
      && apt-get clean \
      && rm -rf /var/lib/apt/lists/*

COPY --from=cpp-builder /usr/local/bin /usr/local/bin
COPY --from=cpp-builder /usr/local/include /usr/local/include
COPY --from=cpp-builder /usr/local/lib /usr/local/lib
COPY --from=cpp-builder /usr/local/share /usr/loca/share

RUN mkdir /workspace
WORKDIR /workspace
