cmake_minimum_required(VERSION 3.0)
project(SensorGraphing)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add debugging flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -ggdb")

# Find the required Python and NumPy libraries
find_package(PythonLibs 2.7 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

# NumPy headers
find_package(NumPy REQUIRED)
include_directories(${NUMPY_INCLUDE_DIRS})

# Include the matplotlib-cpp header file
include_directories("path/to/matplotlib-cpp") # Update with the actual path

# Include directories of your IMU and GPS modules and other dependencies
include_directories("../../imu_module")
include_directories("../../gps_module")
include_directories("../../ubx_lib")

# Executable for GPS graph
add_executable(gps_graph gps_graphs.cpp ../../gps_module/gps.cpp ../../ubx_lib/ubx_msg.cpp)

# Executable for IMU graph
add_executable(imu_graph imu_graphs.cpp ../../imu_module/imu.cpp)

# Link Python libraries and i2c library
target_link_libraries(gps_graph ${PYTHON_LIBRARIES} i2c)
target_link_libraries(imu_graph ${PYTHON_LIBRARIES} i2c)
