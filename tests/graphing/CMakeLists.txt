cmake_minimum_required(VERSION 3.10)  # Set the minimum version of CMake

# Set your project name
project(MyProject)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find the Matplot++ package
find_package(Matplot++ REQUIRED)

# Add the executables with their corresponding source files
add_executable(gps_graph gps_graphs.cpp ../../gps_module/gps.cpp ../../ubx_lib/ubx_msg.cpp)
add_executable(imu_graph imu_graphs.cpp ../../imu_module/imu.cpp)

# Link against Matplot++ and add the i2c library
target_link_libraries(gps_graph Matplot++::matplot i2c)
target_link_libraries(imu_graph Matplot++::matplot i2c)

# Add the -ggdb flag for debugging information (for both executables)
target_compile_options(gps_graph PRIVATE -ggdb)
target_compile_options(imu_graph PRIVATE -ggdb)

