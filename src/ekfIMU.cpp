#include "ekfIMU.h"

EKF_IMU::EKF_IMU() {
    // State vector: [qw, qx, qy, qz, wx, wy, wz]
    state = Eigen::VectorXf(7);
    state << 1, 0, 0, 0, 0, 0, 0; // Initial orientation as a unit quaternion, and zero angular velocity

    // Covariance matrix P
    P = Eigen::MatrixXf::Identity(7, 7);
    P.block<4, 4>(0, 0) *= 0.1; // Initial uncertainty in orientation
    P.block<3, 3>(4, 4) *= 0.1; // Initial uncertainty in gyro bias

    // Process noise covariance matrix Q
    Q = Eigen::MatrixXf::Identity(7, 7);
    Q.block<4, 4>(0, 0) *= 0.001; // Process noise in orientation
    Q.block<3, 3>(4, 4) *= 0.001; // Process noise in gyro bias

    // Measurement noise covariance matrix R
    R = Eigen::MatrixXf::Identity(6, 6);
    R.block<3, 3>(0, 0) *= 0.1; // Noise in accelerometer measurements
    R.block<3, 3>(3, 3) *= 0.1; // Noise in magnetometer measurements
}

void EKF_IMU::predict(float gx, float gy, float gz, float dt) {
    // Gyroscope measurements to angular velocity in body frame
    Eigen::Vector3f omega(gx - state(4), gy - state(5), gz - state(6));

    // Quaternion derivative
    Eigen::Quaternionf q(state(0), state(1), state(2), state(3));
    Eigen::Quaternionf qDot = (Eigen::Quaternionf(0, omega.x(), omega.y(), omega.z()) * q) * 0.5f;
    
    // State update
    q = q + qDot * dt;
    q.normalize(); // Normalize quaternion

    // Update state
    state(0) = q.w();
    state(1) = q.x();
    state(2) = q.y();
    state(3) = q.z();

    // State transition matrix F
    Eigen::MatrixXf F = computeF(dt);

    // Update covariance matrix
    P = F * P * F.transpose() + Q;
}

void EKF_IMU::update(float ax, float ay, float az, float mx, float my, float mz) {
    // Measurement matrix H
    Eigen::MatrixXf H = computeH();

    // Compute the expected measurement
    Eigen::VectorXf z = computeMeasurement(ax, ay, az, mx, my, mz);

    // Innovation or measurement residual
    Eigen::VectorXf y = z - H * state;

    // Innovation covariance
    Eigen::MatrixXf S = H * P * H.transpose() + R;

    // Kalman gain
    Eigen::MatrixXf K = P * H.transpose() * S.inverse();

    // Update the state
    state += K * y;

    // Update covariance matrix
    int size = state.size();
    Eigen::MatrixXf I = Eigen::MatrixXf::Identity(size, size);
    P = (I - K * H) * P;
}

/* Utility Functions */
Eigen::MatrixXf EKF_IMU::computeF(float dt) {
    Eigen::MatrixXf F = Eigen::MatrixXf::Identity(7, 7);

    // Derivatives of the quaternion elements (qw, qx, qy, qz) with respect to angular velocity
    F(0, 4) = -0.5 * state(1) * dt;
    F(0, 5) = -0.5 * state(2) * dt;
    F(0, 6) = -0.5 * state(3) * dt;
    F(1, 4) =  0.5 * state(0) * dt;
    F(1, 5) = -0.5 * state(3) * dt;
    F(1, 6) =  0.5 * state(2) * dt;
    F(2, 4) =  0.5 * state(3) * dt;
    F(2, 5) =  0.5 * state(0) * dt;
    F(2, 6) = -0.5 * state(1) * dt;
    F(3, 4) = -0.5 * state(2) * dt;
    F(3, 5) =  0.5 * state(1) * dt;
    F(3, 6) =  0.5 * state(0) * dt;

    return F;
}

Eigen::MatrixXf EKF_IMU::computeH() {
    Eigen::MatrixXf H = Eigen::MatrixXf::Zero(6, 7);

    // Partial derivatives of the accelerometer measurements with respect to the quaternion
    // Assuming the accelerometer measures the gravitational acceleration
    H(0, 1) = 2 * G; H(0, 3) = -2 * G;
    H(1, 0) = -2 * G; H(1, 2) = -2 * G;
    H(2, 1) = 2 * G; H(2, 3) = 2 * G;

    // Partial derivatives of the magnetometer measurements with respect to the quaternion
    // Assuming a simple magnetic field model aligned with the earth's surface
    H(3, 0) = 2 * MY; H(3, 2) = 2 * MX;
    H(4, 1) = 2 * MX; H(4, 3) = 2 * MY;
    H(5, 0) = -2 * MX; H(5, 2) = 2 * MY;

    return H;
}

// Constants for magnetic field (MX, MY) need to be defined based on your location or sensor's characteristics

Eigen::VectorXf EKF_IMU::computeMeasurement(float ax, float ay, float az, float mx, float my, float mz) {
    Eigen::VectorXf z(6);

    // Use the gravity vector as the expected measurement for the accelerometer
    z(0) = 0; // Expected acceleration in X
    z(1) = 0; // Expected acceleration in Y
    z(2) = G; // Expected acceleration in Z (gravity)

    // Use the local magnetic field as the expected measurement for the magnetometer
    z(3) = mx; // Expected magnetic field in X
    z(4) = my; // Expected magnetic field in Y
    z(5) = mz; // Expected magnetic field in Z

    return z;
}

// G is the gravitational acceleration constant

/* Orientation */
std::tuple<float, float, float> EKF_IMU::getOrientation() {
    // Convert the state quaternion to Euler angles (pitch, yaw, roll)
    Eigen::Quaternionf q(state(0), state(1), state(2), state(3));
    return toEulerAngles(q);
}

std::tuple<float, float, float> EKF_IMU::toEulerAngles(const Eigen::Quaternionf& q) {
    // Convert a quaternion to Euler angles
    float pitch = std::atan2(2.0f * (q.w() * q.x() + q.y() * q.z()), 1.0f - 2.0f * (q.x() * q.x() + q.y() * q.y()));
    float yaw = std::asin(2.0f * (q.w() * q.y() - q.z() * q.x()));
    float roll = std::atan2(2.0f * (q.w() * q.z() + q.x() * q.y()), 1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z()));

    return std::make_tuple(pitch, yaw, roll);
}

