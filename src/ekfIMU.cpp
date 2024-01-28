#include "ekfIMU.h"

EKF_IMU::EKF_IMU()
{
    // Initialize state vector (e.g., [roll, pitch, yaw, gyro_bias_x, gyro_bias_y, gyro_bias_z])
    state = Eigen::VectorXf(6);
    state << 0, 0, 0, 0, 0, 0;

    // Initialize error covariance matrix P
    P = Eigen::MatrixXf::Identity(6, 6);

    // Define process noise covariance matrix Q
    Q = Eigen::MatrixXf::Identity(6, 6) * 0.1; // Example value, needs tuning

    // Define measurement noise covariance matrix R
    R = Eigen::MatrixXf::Identity(6, 6) * 0.1; // Example value, needs tuning
}

void EKF_IMU::Predict(const Eigen::Vector3f &gyro, float dt)
{
    // Extract gyroscope biases from the state
    Eigen::Vector3f gyro_bias = state.segment<3>(3);

    // Gyroscope data corrected for biases
    Eigen::Vector3f corrected_gyro = gyro - gyro_bias;

    // Update state using gyroscope data
    // This example assumes Euler angles in the state vector
    state(0) += corrected_gyro(0) * dt; // Roll
    state(1) += corrected_gyro(1) * dt; // Pitch
    state(2) += corrected_gyro(2) * dt; // Yaw

    // State transition matrix (Jacobian of the state transition function)
    Eigen::MatrixXf F = Eigen::MatrixXf::Identity(6, 6);
    F(0, 3) = -dt;
    F(1, 4) = -dt;
    F(2, 5) = -dt;

    // Update error covariance matrix
    P = F * P * F.transpose() + Q;
}

void EKF_IMU::Update(const Eigen::Vector3f &accel, const Eigen::Vector3f &mag)
{
    // Measurement function - map the state vector into the measurement space
    // For simplicity, we'll just use the accelerometer and magnetometer readings directly
    Eigen::VectorXf z(6);
    z << accel, mag;

    // Measurement matrix (Jacobian of the measurement function)
    Eigen::MatrixXf H = Eigen::MatrixXf::Identity(6, 6);

    // Measurement residual
    Eigen::VectorXf y = z - H * state;

    // Residual covariance
    Eigen::MatrixXf S = H * P * H.transpose() + R;

    // Kalman gain
    Eigen::MatrixXf K = P * H.transpose() * S.inverse();

    // Update the state and error covariance matrix
    state = state + K * y;
    P = (Eigen::MatrixXf::Identity(6, 6) - K * H) * P;
}

std::tuple<float, float, float> EKF_IMU::GetPitchRollYaw() 
{
    // Since the state vector is assumed to be in the format: [roll, pitch, yaw, gyro_bias_x, gyro_bias_y, gyro_bias_z]
    float roll = state(0);  // Roll
    float pitch = state(1); // Pitch
    float yaw = state(2);   // Yaw

    return std::make_tuple(pitch, roll, yaw);
}
