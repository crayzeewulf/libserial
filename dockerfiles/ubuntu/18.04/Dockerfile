#
# Run the following command from top-level folder of libserial source code
# to build the libserial image for Ubuntu 18.04:
#
# docker build -t libserial:ubuntu-18.04 -f dockerfiles/ubuntu/18.04/Dockerfile .
#
# ------------------------------------------------------------------------------
# base
# ------------------------------------------------------------------------------
FROM ubuntu:18.04 AS base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get -yq update && apt-get install -yq --no-install-recommends \
        build-essential \
        cmake \
        coreutils \
        doxygen \
        g++ \
        graphviz \
        libboost-test-dev \
        libgtest-dev \
        libpython-dev \
    && apt-get autoremove -y \
    && apt-get clean -y


# ------------------------------------------------------------------------------
# build
# ------------------------------------------------------------------------------
FROM base AS build

COPY . /usr/src/libserial
RUN cd /usr/src/libserial \
    && rm -rf build \
    && mkdir -p build \
    && cd build \
    && cmake .. \
    && make -j$(nproc) \
    && make install \
    && chown -R root:root /usr/local

# ------------------------------------------------------------------------------
# release
# ------------------------------------------------------------------------------

FROM ubuntu:18.04 AS release
COPY --from=build /usr/local /usr/local
