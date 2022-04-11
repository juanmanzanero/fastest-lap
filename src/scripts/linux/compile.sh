#!/bin/sh

mkdir build
cd build
cmake .. -DPYTHON_API_ABSOLUTE_PATH=off
make