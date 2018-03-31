#!/bin/bash
# Script to clean the tree from all compiled files.

cd `dirname $0`
rm -rf build
echo Cleaned up the project!
