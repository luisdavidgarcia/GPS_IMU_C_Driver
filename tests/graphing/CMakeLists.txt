cmake_minimum_required(VERSION 3.10)  # Set the minimum version of CMake

# Set your project name
project(MyProject)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find the Matplot++ package
find_package(Matplot++ REQUIRED)

# Add the executable
add_executable(gps_graph gps_graphs.cpp)
add_executable(imu_graph imu_graphs.cpp)

# Link against Matplot++
target_link_libraries(gps_graph Matplot++::matplot ../ubx_lib/ubx_msg.cpp ../../gps_module/gps.cpp)
target_link_libraries(imu_graph Matplot++::matplot ../../imu_module/imu.cpp)

