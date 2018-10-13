#!/bin/bash
mkdir -p build-gtest
cd build-gtest
wget https://github.com/google/googletest/archive/release-1.8.0.tar.gz
tar xf release-1.8.0.tar.gz
cd googletest-release-1.8.0
cmake -DBUILD_SHARED_LIBS=ON .
make
cp -a googletest/include/gtest /usr/include
cp -a googlemock/gtest/libgtest_main.so googlemock/gtest/libgtest.so /usr/lib/
