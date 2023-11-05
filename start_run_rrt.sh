#!/bin/bash
set -e
echo "***** run nav****"

cd build/
cmake .. &&make &&  ./rrt

