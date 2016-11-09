# Template matcher 2016

## Goal

- Speed up existing TM implementation
- Implement image registration for fine image alignment

## Overview

To address the slow matching we're going to use MIH approach from the paper [1](http://www.cs.toronto.edu/~norouzi/research/papers/multi_index_hashing.pdf).
For image registration the inital idea is to employ to inverse-compositional image alignment algorithm as it has
huge benefit as compared to forward-mapping approach - J matrix should be computed only once at training stage.


## Dependencies
- OpenCV 3.0 or newer (external)
- JsonCPP (bundled)
- Gtest (bundled)

## Building

Building is done via CMake. OpenCV should be installed prior running cmake. 

Linux:
    export OpenCV_DIR="~/OpenCV/build"
    mkdir build
    cd build
    cmake -D OpenCV_DIR=$OpenCV_DIR ..

MacOSX (Xcode):
    export OpenCV_DIR="~/OpenCV/build"
    mkdir build
    cd build
    cmake -G Xcode -D OpenCV_DIR=$OpenCV_DIR ..

Windows (MS Visual Studio):
    set OpenCV_DIR="C:\OpenCV\build"
    mkdir build
    cd build
    cmake -G "Visual Studio 9 2008" -D OpenCV_DIR=%OpenCV_DIR% ..
