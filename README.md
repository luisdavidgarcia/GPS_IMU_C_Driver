# GPS and IMU C/C++ Driver for Rover

This project focuses on converting existing GPS and IMU Python code into C++. It provides a comprehensive C/C++ driver for GPS and IMU modules, allowing you to interface with these sensors in your rover project.

## Features

- Conversion of Python code to C++ for GPS and IMU functionality.
- Integration of Git pre-commit hooks to maintain code quality.

## Getting Started

Follow these steps to get started with this project:

1. **Fork this Repository**: Start by [forking this template repository](https://github.com/Solar-Autonomous-ROACH/GPS_IMU_C_Driver/tree/main) to your own GitHub account. This will allow you to work on your own copy of the project.

2. **Install Git Hooks**: Use the provided `setup.py` script to install Git pre-commit hooks. These hooks ensure that your code adheres to quality standards before committing changes.

3. **Programming**: Begin programming your rover using the GPS and IMU modules provided in this repository. You can customize and extend the code to suit your project's specific needs.

## Project Structure

The project consists of the following main modules:

- **GPS Module**: Contains code for interfacing with the GPS sensor.
- **IMU Module**: Includes code for working with the IMU (Inertial Measurement Unit) sensor.
- **UBX_Lib**: A library for GPS communication, implementing the UBX protocol for GPS data exchange.

## Documentation

### Hardware Specifications

- **GPS**: ZOE M8Q Board - [Datasheet](https://www.mouser.com/datasheet/2/813/ZOE_M8_HIM__UBX_16030136_-2487913.pdf)
    - UBX Protocol - [Datasheet](https://docs.rs-online.com/8e7c/0900766b815aef70.pdf)
- **IMU**: ICM-20948 - [Datasheet](https://invensense.tdk.com/download-pdf/icm-20948-datasheet/)

### Libraries

- **GPS Module Libraries**:
  - SparkFun Arduino I2C with UBX Library: [GitHub](https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library)
  - GPS SAM-M8Q Library (Python): [Melopero SAM-M8Q Module GitHub](https://github.com/melopero/Melopero_SAM-M8Q/blob/master/README.md)
- **IMU Module Libraries**:
  - SparkFun Library for IMU: [GitHub](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/blob/main/src/ICM_20948.h#L34)
  - Old Rover Code for IMU: [Rover IMU Code Old](https://github.com/GoScoutOrg/Rover/blob/main/src/rover/imu.py#L23)
- **Plotting Library**:
  - Matplot++: [GitHub](https://github.com/alandefreitas/matplotplusplus)
  - Installation: [Build from Source](https://alandefreitas.github.io/matplotplusplus/integration/install/build-from-source/build-and-install/)
- **I2C Libraries**:
  - I2C on Raspberry Pi: [I2C on C on Pi](https://raspberrypi.stackexchange.com/questions/33485/using-i2c-in-c-on-raspberry-pi)
  - I2C Library for C/C++/Python: [I2C in C Library](https://www.kernel.org/doc/html/v5.4/i2c/dev-interface.html)

### Additional Notes

- UBX is a communication protocol with specific functions for GPS.
- Commands for interacting with GPS via Python:
  ```bash
  source myenv/bin/activate
  sudo i2cdetect -y <value>  # Checks for live i2c slave address
  cd /home/simba/Desktop/Rover/src

### Wiring Instructions

- [Wiring guide and diagrams](https://external-content.duckduckgo.com/iu/?u=https%3A%2F%2Fwww.etechnophiles.com%2Fwp-content%2Fuploads%2F2021%2F01%2FR-Pi-4-GPIO-Pinout.jpg&f=1&nofb=1&ipt=0ca1f8c57d8139e68c3cfa5b652b44c059319ac6829c54b741515728d93f7703&ipo=images)

### Compiling and Testing

- Compile with: `g++ -O example_i2c_test.cpp -o weak -li2c -ggdb`
- Replicated Test example for IMU: [Python Example](https://github.com/sparkfun/Qwiic_9DoF_IMU_ICM20948_Py/blob/main/examples/ex1_qwiic_ICM20948.py)

## License

This project is released under the [MIT License](LICENSE).

## Contributing

If you find issues, have suggestions, or would like to contribute to this project, please [open an issue](https://github.com/Solar-Autonomous-ROACH/GPS_IMU_C_Driver/issues) or submit a [pull request](https://github.com/Solar-Autonomous-ROACH/GPS_IMU_C_Driver/pulls). Your contributions are welcome and appreciated!

## Acknowledgments

This project draws inspiration from the fields of robotics, sensor integration, and C/C++ development.

Happy coding and happy rover building!
