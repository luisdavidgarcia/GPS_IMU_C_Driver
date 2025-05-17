# GPS, IMU, and UBX C++ Drivers for Autonomous Rover Projects

A high-performance C++ driver suite for GPS, IMU, and UBX modules in rover 
projects. Optimized for resource-constrained systems like Raspberry Pi.

## Features

- **High-Performance C++ Drivers**: Python-to-C++ conversion for real-time 
  navigation applications
- **Modern C++ Standards**: C++11-20 features for clean, maintainable code
- **Resource Efficient**: Optimized for systems with limited resources (1GB RAM)
- **Quality Assured**: Git pre-commit hooks for code quality control
- **Containerized Development**: Docker environment for consistent development

## Demo

[![Kalman Filter Demo](https://img.youtube.com/vi/vySJalcy1_8/0.jpg)](https://youtu.be/vySJalcy1_8?feature=shared)

*Click the thumbnail to watch the IMU Kalman Filter visualization*

## Project Structure

```
├── src/           # Source code for GPS, IMU, and UBX drivers
├── include/       # Header files
├── tests/         # Unit tests
├── Dockerfile     # Development environment configuration
├── setup.sh       # Environment setup script
└── CMakeLists.txt # Build configuration
```

## Requirements

### Hardware
- Raspberry Pi (or compatible microcontroller)
- GPS Module (e.g., SAM-M8Q)
- IMU Module (e.g., ICM-20948)
- Qwiic cables for connections

### Software
- Ubuntu 22.04 LTS (primary focus, others may work)
- Docker
- CMake
- Git
- Python 3.10 (for testing/calibration scripts)
- pre-commit (`pip install pre-commit`)

### Dependencies
- Eigen3 (matrix operations, Kalman filter)
- I2C library (module communication)
- UBX protocol library (GPS message parsing)

## Setup and Installation

1. **Clone the Repository**
   ```bash
   git clone https://github.com/luisdavidgarcia/GPS_IMU_C_Driver
   cd GPS_IMU_C_Driver
   ```

2. **Setup Environment**
   ```bash
   ./setup.sh   # Creates Docker image and installs pre-commit hooks
   ```

3. **Build the Project within the Docker Container**
   ```bash
   mkdir build
   cd build
   cmake ..
   make
   ```

## Development Workflow

The project is designed to be built and developed within a Docker container:

1. Use VSCode with Remote-Containers extension to work inside the container
2. Run pre-commit hooks before committing to ensure code quality
3. Follow C++ Google Style Guide and C++ Core Guidelines

## Hardware Setup

![Wiring Diagram](WiringDiagram.png "Wiring Diagram for GPS and IMU with Raspberry Pi")

### Connections
1. **Power**: Raspberry Pi 3.3V → GPS module VCC
2. **Ground**: Raspberry Pi GND → GPS module GND
3. **I2C**: 
   - Raspberry Pi SCL → GPS module SCL
   - Raspberry Pi SDA → GPS module SDA
4. **Daisy Chain**: Connect GPS to IMU module using Qwiic cable
   - Ensure different I2C addresses for GPS and IMU modules

## Troubleshooting

### Common Issues
- **No Response**: Check wiring and power supply
- **Incorrect Readings**: Verify calibration and configuration
- **I2C Errors**: Check addresses and connections
- **UBX Parse Errors**: Verify message format and checksum

### Debugging Tips
- Use gdb for code debugging
- Check connections and power
- Verify I2C configuration
- Confirm library installations

## Contributing

1. Fork and branch for features/fixes
2. Make changes and run pre-commit hooks
3. Submit PR with clear description
4. Address reviewer feedback

## References

### Hardware Datasheets
- [SAM-M8Q GPS Module](https://www.u-blox.com/en/product/sam-m8q-module)
- [ICM-20948 IMU](https://www.invensense.com/products/motion-tracking/9-axis/mpu-9250/)
- [UBX Protocol](https://content.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_(GPS.G6-SW-10018)_Public.pdf)

### Libraries
- **GPS**: [SparkFun UBX](https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library), [Melopero SAM-M8Q](https://github.com/melopero/Melopero_SAM-M8Q)
- **IMU**: [SparkFun ICM-20948](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary)
- **Kalman**: [EKF Navigation](https://github.com/balamuruganky/ekf_nav_ins), [Eigen3](https://eigen.tuxfamily.org)
- **I2C**: [I2C on Raspberry Pi](https://raspberrypi.stackexchange.com/questions/33485/using-i2c-in-c-on-raspberry-pi), [Linux I2C](https://www.kernel.org/doc/html/v5.4/i2c/dev-interface.html)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) 
file for details.

## Acknowledgments

Thanks to the open-source projects that inspired this driver suite and to the 
community for contributions and feedback.
