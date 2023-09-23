#!/bin/bash

# ----- Has to be run from the folder 'laser_trace'

g++ laser_trace.cpp -fPIC -O3  -shared  -I/usr/include/python3.10 -lboost_python310 -lpython3.10 -o laser_trace.so

mv laser_trace.so ../
