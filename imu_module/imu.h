#ifndef IMU_H
#define IMU_H

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <time.h>

#define SLAVE_ADDRESS 0x42
#define TIME_DELAY_MS 1000
#define ACCEL_REG_START     0x00
#define MAGNETO_REG_START   0x00
#define GYRO_REG_START      0x00
#define DATA_SIZE           6
#define BYTE_SHIFT_AMOUNT   8

class Imu {
private:
    int i2c_fd;
    float accelerometer[3];
    float magnetometer[3];
    float gyroscope[3];

    void readSensorData();

public:
    Imu();
    ~Imu();
    void printAccel();
    void printMag();
    void printGyro();
    void Telementary(int delay);
};

#endif // IMU_H

