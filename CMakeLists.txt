cmake_minimum_required(VERSION 3.15)
project(GPS_IMU_Drivers CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(src/GPS)
add_subdirectory(src/UBX)
add_subdirectory(src/IMU)

set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install)