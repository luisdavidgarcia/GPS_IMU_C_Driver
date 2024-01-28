#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <tuple>
#include <iostream>

class EKF_IMU
{
    private:
        Eigen::VectorXf state; // State vector
        Eigen::MatrixXf P;     // Error covariance matrix
        Eigen::MatrixXf Q;     // Process noise covariance matrix
        Eigen::MatrixXf R;     // Measurement noise covariance matrix
        float normalizeAngle(float angle); 

    public:
        EKF_IMU();
        void Predict(const Eigen::Vector3f &gyro, float dt);
        void Update(const Eigen::Vector3f &accel, const Eigen::Vector3f &mag);
        std::tuple<float, float, float> GetPitchRollYaw();
};
