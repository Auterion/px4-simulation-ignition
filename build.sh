#!/bin/bash

# present working directory
PWD=$(pwd)

# Clone c_library_v2 commit matching with current px4-firmware mavlink commit
# => mavlink/c_library_v2:fbdb7c29 is built from mavlink/mavlink:08112084
git clone -q https://github.com/mavlink/c_library_v2.git ${PWD}/mavlink && \
    cd ${PWD}/mavlink && git checkout -q fbdb7c29e47902d44eeaa58b4395678a9b78f3ae && \
    rm -rf ${PWD}/mavlink/.git && cd ${PWD}

export _MAVLINK_INCLUDE_DIR=${PWD}/mavlink


if [ ! -e build ]; then
  mkdir build
fi
cd build
cmake ..
make

