#!/bin/bash
present_working_directory=$(pwd)
cd /usr/src/gtest
sudo cmake CMakeLists.txt
sudo make
sudo cp *.a /usr/lib
cd $present_working_directory
