#!/bin/bash
mkdir -p build
cd build
cmake ..
cmake -G Sublime\ Text\ 2\ -\ Unix\ Makefiles .. 
cd ..
make -j3 -C build
