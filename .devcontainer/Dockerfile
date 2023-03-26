#-----#
# Pip #
#-----#

#cf. https://qiita.com/XPT60/items/e123fe88ec88a4ac2749

FROM amd64/ubuntu:20.04 AS pip-installer

ENV DEBIAN_FRONTEND="noninteractive" 
RUN apt-get update && \
      apt-get install -y python3.8 python3-pip \
      git gcc cmake

ENV HOME /root
WORKDIR "${HOME}"

COPY requirements.txt "${HOME}"
RUN pip install -r requirements.txt
RUN rm "${HOME}/requirements.txt"

#--------#
# Python #
#--------#

FROM amd64/ubuntu:20.04 AS python-builder

ENV DEBIAN_FRONTEND="noninteractive" 
RUN apt-get update && \
      apt-get install -y python3.8 python3-pip \
      git gcc cmake

ENV PIP_DIR /usr/local/lib/python3.8/dist-packages
WORKDIR "${PIP_DIR}"

# Pangolin
RUN apt-get update && \
      apt-get install -y libglew-dev libpython2.7-dev libeigen3-dev
# Bug fixed PR version
RUN git clone https://github.com/bravech/pangolin "${PIP_DIR}/pangolin"
RUN mkdir "${PIP_DIR}/pangolin/build"
WORKDIR "${PIP_DIR}/pangolin/build"
RUN cmake .. && make -j"$(nproc)"
RUN sed -i "s/install_dirs/install_dir/g" "${PIP_DIR}/pangolin/setup.py"
WORKDIR "${PIP_DIR}"
RUN pip install -e "${PIP_DIR}/pangolin"

# g2opy
RUN apt-get update && \
      apt-get install -y libsuitesparse-dev qtdeclarative5-dev libqglviewer-dev-qt5
# Bug fixed PR version
RUN git clone https://github.com/codegrafix/g2opy.git "${PIP_DIR}/g2opy"
RUN mkdir "${PIP_DIR}/g2opy/build"
WORKDIR "${PIP_DIR}/g2opy/build"
RUN cmake .. && make -j"$(nproc)"
WORKDIR "${PIP_DIR}"
RUN pip install -e "${PIP_DIR}/g2opy"

#-------------#
# C++ Builder #
#-------------#

FROM amd64/ubuntu:20.04 AS cpp-builder-base

ENV DEBIAN_FRONTEND="noninteractive" 
ENV HOME /root
WORKDIR "$HOME"

# Install CMake 3.21.6 for g2o
# CLion supports CMake 2.8.11~3.21.x
RUN apt-get update && \
      apt-get install -y git build-essential libssl-dev
RUN git clone https://gitlab.kitware.com/cmake/cmake.git -b v3.21.6
RUN mkdir "$HOME/cmake/build"
WORKDIR "$HOME/cmake/build"
RUN ../bootstrap && make -j"$(nproc)" && make install
WORKDIR "$HOME"

RUN apt-get update && \
      apt-get install -y \
      gcc g++ gdb clang make \
      ninja-build autoconf automake \
      locales-all dos2unix rsync tar

#----------#
# C++ Libs #
#----------#

FROM cpp-builder-base AS cpp-builder

# glog
RUN git clone https://github.com/google/glog.git -b v0.5.0
RUN mkdir "$HOME/glog/build"
WORKDIR "$HOME/glog/build"
RUN cmake .. && make -j"$(nproc)" && make install
WORKDIR "$HOME"
RUN rm -rf "$HOME/glog"

# gflags    
RUN git clone https://github.com/gflags/gflags.git -b v2.2.2
RUN mkdir "$HOME/gflags/build"
WORKDIR "$HOME/gflags/build"
RUN cmake .. && make -j"$(nproc)" && make install
WORKDIR "$HOME"
RUN rm -rf "$HOME/gflags"

# gtest
RUN git clone https://github.com/google/googletest.git -b release-1.11.0
RUN mkdir "$HOME/googletest/build"
WORKDIR "$HOME/googletest/build"
RUN cmake .. && make -j"$(nproc)" && make install
WORKDIR "$HOME"
RUN rm -rf "$HOME/googletest"

# Eigen
RUN git clone https://gitlab.com/libeigen/eigen.git -b 3.4.0
RUN mkdir "$HOME/eigen/build"
WORKDIR "$HOME/eigen/build"
RUN cmake .. && make -j"$(nproc)" && make install
WORKDIR "$HOME"
RUN rm -rf "$HOME/eigen"

# Sophus
RUN git clone https://github.com/strasdat/Sophus.git -b 1.22.10
RUN mkdir "$HOME/Sophus/build"
WORKDIR "$HOME/Sophus/build"
RUN cmake .. && make -j"$(nproc)" && make install
WORKDIR $HOME
RUN rm -rf "$HOME/Sophus"

# ceres-solver
RUN apt-get update && \
      apt-get install -y libatlas-base-dev libsuitesparse-dev
RUN git clone https://github.com/ceres-solver/ceres-solver.git -b 2.1.0
RUN mkdir "$HOME/ceres-solver/build"
WORKDIR "$HOME/ceres-solver/build"
RUN cmake .. && make -j"$(nproc)" && make install
WORKDIR "$HOME"
RUN rm -rf "$HOME/ceres-solver"

# g2o
RUN apt-get update && \
      apt-get install -y libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5
RUN git clone https://github.com/RainerKuemmerle/g2o.git -b 20230223_git
RUN mkdir "$HOME/g2o/build"
WORKDIR "$HOME/g2o/build"
RUN cmake .. && make -j"$(nproc)" && make install
WORKDIR "$HOME"
RUN rm -rf "$HOME/g2o"

# pybind11
RUN apt-get update && apt-get install -y python3.8 python3-pip
RUN git clone https://github.com/pybind/pybind11.git -b v2.10.4
RUN mkdir "$HOME/pybind11/build"
WORKDIR "$HOME/pybind11/build"
RUN cmake .. && make -j"$(nproc)" && make install
WORKDIR "$HOME"
RUN rm -rf "$HOME/pybind11"

# PCL
# RUN apt-get install -y libusb-1.0-0-dev libflann-dev libvtk7-dev libpcap-dev
# RUN git clone https://github.com/PointCloudLibrary/pcl.git -b pcl-1.12.1
# RUN mkdir "$HOME/pcl/build"
# WORKDIR "$HOME/pcl/build"
# RUN cmake .. && make -j"$(nproc)" && make install
# WORKDIR "$HOME"
# RUN rm -rf "$HOME/pcl"

#--------#
# runner #
#--------#

FROM cpp-builder-base AS runner

ENV HOME /root
WORKDIR "$HOME"

# Localization
ENV DEBIAN_FRONTEND="noninteractive"
RUN apt-get update && apt-get upgrade -y
RUN apt-get update && apt-get install -y tzdata
ENV TZ=Asia/Tokyo
RUN apt-get install -y locales fonts-takao && locale-gen ja_JP.UTF-8
ENV LANG=ja_JP.UTF-8
ENV LANGUAGE=ja_JP:ja
ENV LC_ALL=ja_JP.UTF-8
RUN localedef -f UTF-8 -i ja_JP ja_JP.utf8

# VNC
RUN apt-get update && \
      apt-get install -y lxde \
      tigervnc-standalone-server tigervnc-common \
      novnc websockify

# gdb printers
# for OpenCV Mat debugger
RUN apt-get update && \
      apt-get install -y python3-pip
RUN pip3 install numpy
# Place gdb scripts
COPY gdbinit.sh "$HOME/.gdbinit"
COPY gdb/ "$HOME/gdb/"

# Python
RUN apt-get update && \
      apt-get install -y python3.8 python3-pip python3-matplotlib python3-numpy python3-dev wget

# matplotlib-cpp
RUN wget https://github.com/lava/matplotlib-cpp/raw/master/matplotlibcpp.h -P /usr/local/include

# libboost-"all"-dev is needed for PCL
RUN apt-get update && apt-get install -y \
      libboost-all-dev  libopencv-dev libpcl-dev

COPY --from=pip-installer /usr/local/lib/python3.8/dist-packages /usr/local/lib/python3.8/dist-packages
COPY --from=pip-installer /usr/local/bin /usr/local/bin

# Pangolin
RUN apt-get update && \
      apt-get install -y libglew-dev libpython2.7-dev libeigen3-dev
# g2opy
RUN apt-get update && \
      apt-get install -y libsuitesparse-dev qtdeclarative5-dev libqglviewer-dev-qt5
COPY --from=python-builder /usr/local/lib/python3.8/dist-packages /usr/local/lib/python3.8/dist-packages
COPY --from=python-builder /usr/local/bin /usr/local/bin

# ceres-solver
RUN apt-get update && \
      apt-get install -y libatlas-base-dev libsuitesparse-dev
# g2o
RUN apt-get update && \
      apt-get install -y libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5
COPY --from=cpp-builder /usr/local/bin /usr/local/bin      
COPY --from=cpp-builder /usr/local/include /usr/local/include
COPY --from=cpp-builder /usr/local/lib /usr/local/lib
COPY --from=cpp-builder /usr/local/share /usr/local/share

RUN apt-get clean && \ 
      rm -rf /var/lib/apt/lists/*

RUN mkdir /workspace
WORKDIR /workspace