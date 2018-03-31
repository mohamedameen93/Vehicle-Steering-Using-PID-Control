#!/bin/bash
# Script to build all components from scratch, using the maximum available CPU power

# Go into the directory where this bash script is contained.
cd `dirname $0`

# Compile code.
mkdir -p build
cd build
cmake ..
make -j `nproc` $*
echo Build process completed!
