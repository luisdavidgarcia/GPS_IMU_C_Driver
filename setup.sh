#!/bin/bash

# setup virtual environment
python3 -m venv venv
source venv/bin/activate

pip3 install matplotlib
pip3 install numpy

# Update package lists
apt update

# Install C++ development tools
apt install -y build-essential cmake

# Install I2C libraries
apt install -y libi2c-dev

# Install Eigen Library
apt install -y libeigen3-dev
