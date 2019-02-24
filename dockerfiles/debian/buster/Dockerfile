# ------------------------------------------------------------------------------
# base
# ------------------------------------------------------------------------------
FROM debian:buster AS base

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
    && make install

# ------------------------------------------------------------------------------
# release
# ------------------------------------------------------------------------------

FROM debian:buster AS release
COPY --from=build /usr/local /usr/local
