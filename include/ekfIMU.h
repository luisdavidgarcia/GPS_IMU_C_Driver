#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <tuple>

// Constants
constexpr float PI = 3.14159265358979323846f;
// acceleration due to gravity
constexpr float G = 9.807f;
// Local magnetic field strength in X (microteslas)
constexpr float MX = 20.0f; 
// Local magnetic field strength in Y (microteslas)
constexpr float MY = 15.0f; 


class EKF_IMU {
public:
    EKF_IMU();
    void predict(float gx, float gy, float gz, float dt);
    void update(float ax, float ay, float az, float mx, float my, float mz);
    std::tuple<float, float, float> getOrientation(); // Returns pitch, yaw, roll

private:
    Eigen::VectorXf state; // State vector
    Eigen::MatrixXf P;     // State covariance matrix
    Eigen::MatrixXf Q;     // Process noise covariance matrix
    Eigen::MatrixXf R;     // Measurement noise covariance matrix

    // Utility functions
    Eigen::MatrixXf computeF(float dt);
    Eigen::MatrixXf computeH();
    Eigen::VectorXf computeMeasurement(float ax, float ay, float az, float mx, float my, float mz);
    std::tuple<float, float, float> toEulerAngles(const Eigen::Quaternionf& q);
};
