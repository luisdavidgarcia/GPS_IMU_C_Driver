#!/bin/bash

# Update package lists
sudo apt update

# Install C++ development tools
sudo apt install g++ cmake

# Install I2C libraries
sudo apt install libi2c-dev

# Install Eigen Library
sudo apt install libeigen3-dev

# Install JPEG and TIFF libraries
sudo apt install libjpeg-dev libtiff5-dev

# Install BLAS and LAPACK libraries
sudo apt install libblas-dev liblapack-dev

# Install a 64-bit ARM Cross-Compiler:
sudo apt-get install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu

# Install FFTW library
sudo apt install libfftw3-dev

# Clone the Matplot++ repository
git clone https://github.com/alandefreitas/matplotplusplus.git

# Navigate to the Matplot++ directory
cd matplotplusplus

# Create a build directory
mkdir build

# Enter the build directory
cd build

# Configure and build Matplot++
cmake .. -DCMAKE_CXX_FLAGS="-O2" -DMATPLOTPP_BUILD_EXAMPLES=OFF -DMATPLOTPP_BUILD_TESTS=OFF
sudo cmake --build .
sudo cmake --install .

# Install Matplot++ system-wide
sudo make install

# Install the MatplotLib C++ header
wget https://raw.githubusercontent.com/lava/matplotlib-cpp/master/matplotlibcpp.h

