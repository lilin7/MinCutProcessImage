#!/bin/bash                                                                                                                               
cd build
module load opencv
cmake ../ -DCMAKE_CXX_COMPILER=/encs/bin/g++ -DCMAKE_CXX_FLAGS=-std=c++11
cd ../
