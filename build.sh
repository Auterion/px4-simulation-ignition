#!/bin/bash

if [ ! -e build ]; then
  mkdir build
fi
cd build
cmake ..
make
cpack -G DEB

