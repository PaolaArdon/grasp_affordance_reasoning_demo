FROM nvidia/cuda:8.0-cudnn6-devel-ubuntu16.04
MAINTAINER paola.ardon@ed.ac.uk

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        wget \
        libatlas-base-dev \
        libboost-all-dev \
        libgflags-dev \
        libgoogle-glog-dev \
        libhdf5-serial-dev \
        libleveldb-dev \
        liblmdb-dev \
        libopencv-dev \
        libprotobuf-dev \
        libsnappy-dev \
        protobuf-compiler \
        python-dev \
        python-numpy \
        python-pip \
        python-setuptools \
        python-scipy \
        pkg-config yasm \
	      gfortran \
	      libjpeg8-dev \
      	libjasper-dev \
      	libpng12-dev \
      	libtiff5-dev \
      	libavcodec-dev \
      	libavformat-dev \
      	libswscale-dev \
      	libdc1394-22-dev \
      	libxine2-dev \
      	libv4l-dev \
      	libgstreamer0.10-dev \
      	libgstreamer-plugins-base0.10-dev \
      	qt5-default \
      	libgtk2.0-dev \
      	libtbb-dev \
      	libfaac-dev \
      	libmp3lame-dev \
      	libtheora-dev \
      	libvorbis-dev \
      	libxvidcore-dev \
      	libopencore-amrnb-dev \
      	libopencore-amrwb-dev \
      	x264 \
      	v4l-utils \
      	libprotobuf-dev \
      	protobuf-compiler \
      	libgoogle-glog-dev \
      	libgflags-dev \
      	libgphoto2-dev \
      	libeigen3-dev \
      	libhdf5-dev \
      	doxygen \
      	python-dev \
      	python-pip \
      	python3-dev \
      	python3-pip \
      	python-matplotlib \
      	python-pil \
      	python-protobuf \
      	cython \
      	python-skimage \
      	vim \
      	python-tk \
      	python-tabulate \
      	python-pyparsing \
      	python-psutil \
      	python-Image \
      	python-dnutils \
      	python-appdirs \
      	cython && \
    rm -rf /var/lib/apt/lists/*

RUN pip2 install pracmln

#
# Install Miniconda in /opt/conda
#

RUN apt-get update --fix-missing && apt-get install -y wget bzip2 ca-certificates \
    libglib2.0-0 libxext6 libsm6 libxrender1 \
    git mercurial subversion libbz2-dev libz-dev libpng-dev

RUN echo 'export PATH=/opt/conda/bin:$PATH' > /etc/profile.d/conda.sh && \
    wget --quiet https://repo.continuum.io/miniconda/Miniconda2-4.3.21-Linux-x86_64.sh -O ~/miniconda.sh && \
    /bin/bash ~/miniconda.sh -b -p /opt/conda && \
    rm ~/miniconda.sh

ENV PATH /opt/conda/bin:$PATH
ENV LD_LIBRARY_PATH /usr/local/cuda-8.0/lib64:/usr/local/cuda-8.0/extras/CUPTI/lib64:$LD_LIBRARY_PATH

RUN conda install numba
ADD /mln_affordance.tar.xz /home/mln_affordance &&/
