#!/bin/bash
set -e 
set -x
mkdir -p build
cd build
cmake ..
#cmake -DCMAKE_INSTALL_PREFIX=/usr ..
make
cd ..
make -j3 -C build
sphinx-build -b html docs/user_manual/ docs/html/
