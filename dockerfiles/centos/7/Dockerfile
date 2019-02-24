#
# Run the following command from top-level folder of libserial source code
# to build the libserial image for CentOS-7:
#
# docker build -t libserial:centos-7 -f dockerfiles/centos/7/Dockerfile .
#
# ------------------------------------------------------------------------------
# base
# ------------------------------------------------------------------------------
FROM centos:7 AS base

RUN yum install -y centos-release-scl \
    && yum install -y devtoolset-7-gcc-c++ \
    && yum install -y https://dl.fedoraproject.org/pub/epel/epel-release-latest-7.noarch.rpm \
    && yum install -y \
       boost-devel \
       cmake3 \
       doxygen \
       graphviz \
       make \
       python-devel 

# ------------------------------------------------------------------------------
# build
# ------------------------------------------------------------------------------
FROM base AS build

ENV CC /opt/rh/devtoolset-7/root/bin/gcc
ENV CXX /opt/rh/devtoolset-7/root/bin/g++
COPY . /usr/src/libserial
RUN cd /usr/src/libserial \
    && rm -rf build \
    && mkdir -p build \
    && cd build \
    && cmake3 .. \
    && make -j$(nproc) \
    && make install

# ------------------------------------------------------------------------------
# release
# ------------------------------------------------------------------------------
FROM centos:7 AS release
COPY --from=build /usr/local /usr/local
