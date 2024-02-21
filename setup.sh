#!/bin/bash

# setup virtual environment
python3 -m venv venv
source venv/bin/activate

pip3 install matplotlib
pip3 install numpy
pip3 install pyserial

# Update package lists
apt update

# Install C++ development tools
apt install -y build-essential cmake

# Install I2C libraries
apt install -y libi2c-dev

# Install Serial libraries
apt install -y libserialport0
apt install -y libserialport-dev

# Install Eigen Library
apt install -y libeigen3-dev

###############################################
# ONLY NECESSARY FOR PLOTTING
###############################################
# # Install JPEG and TIFF libraries
# apt install libjpeg-dev libtiff5-dev

# # Install BLAS and LAPACK libraries
# apt install libblas-dev liblapack-dev

# # Install a 64-bit ARM Cross-Compiler:
# apt-get install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu

# # Install FFTW library
# apt install libfftw3-dev

# # Clone the Matplot++ repository
# git clone https://github.com/alandefreitas/matplotplusplus.git

# # Navigate to the Matplot++ directory
# cd matplotplusplus

# # Create a build directory
# mkdir build

# # Enter the build directory
# cd build

# # Configure and build Matplot++
# cmake .. -DCMAKE_CXX_FLAGS="-O2" -DMATPLOTPP_BUILD_EXAMPLES=OFF -DMATPLOTPP_BUILD_TESTS=OFF
# cmake --build .
# cmake --install .

# # Install Matplot++ system-wide
# make install

# # Install the MatplotLib C++ header
# wget https://raw.githubusercontent.com/lava/matplotlib-cpp/master/matplotlibcpp.h
