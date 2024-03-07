import pandas as pd
import numpy as np

# Load the IMU data from the CSV file
imu_data = pd.read_csv('accel_gyro_imu_data.csv')

# Accelerometer Calibration
# Assuming the accelerometer data is in units of g
# Calculate the means for each axis
accel_means = imu_data[['AccelX', 'AccelY', 'AccelZ']].mean()
accel_offsets = {
    'x_offset': accel_means['AccelX'],
    'y_offset': accel_means['AccelY'],
    'z_offset': accel_means['AccelZ'] - 1  # Assuming the z-axis faced upwards during calibration
}

# Gyroscope Calibration
# Calculate the means (biases) for each axis
gyro_means = imu_data[['GyroX', 'GyroY', 'GyroZ']].mean()
gyro_biases = {
    'x_bias': gyro_means['GyroX'],
    'y_bias': gyro_means['GyroY'],
    'z_bias': gyro_means['GyroZ']
}

print("Accelerometer Offsets:", accel_offsets)
print("Gyroscope Biases:", gyro_biases)

# Apply these offsets and biases to your sensor readings in the future
