#!/bin/bash
mkdir -p build
cd build
cmake ..
#cmake -DCMAKE_INSTALL_PREFIX=/usr ..
make
cd ..
make -j3 -C build