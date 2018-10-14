#!/bin/bash
MYPWD=$(pwd)
cd /usr/src/gtest
sudo cmake CMakeLists.txt
sudo make
sudo cp *.a /usr/lib
cd $MYPWD
