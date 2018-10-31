#!/bin/bash
mkdir -p build
cd build
cmake ..
#cmake -DCMAKE_INSTALL_PREFIX=/usr ..
sudo make install
cd ..
make -j3 -C build
