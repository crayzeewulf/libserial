#!/bin/bash
mkdir -p build
cd build
cmake ..
cd ..
make -j3 -C build
