/*
*) Refactor the code to remove reduentent part and improve the readabilty. 
*) Compiled for Linux with C++14 standard
Copyright (c) 2021 Balamurugan Kandan.
MIT License; See LICENSE.md for complete details
Author: 2021 Balamurugan Kandan
*/

/*
Updated to be a class, use Eigen, and compile as an Arduino library.
Added methods to get gyro and accel bias. Added initialization to
estimated angles rather than assuming IMU is level.

Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

/*
Addapted from earlier version
Copyright 2011 Regents of the University of Minnesota. All rights reserved.
Original Author: Adhika Lie
*/

#pragma once

#include <stdint.h>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <tuple>

constexpr float SIG_W_A = 0.05f;
// Std dev of gyro output noise (rad/s)
constexpr float SIG_W_G = 0.00175f;
// Std dev of Accelerometer Markov Bias
constexpr float SIG_A_D = 0.01f;
// Correlation time or time constant
constexpr float TAU_A = 100.0f;
// Std dev of correlated gyro bias
constexpr float SIG_G_D = 0.00025;
// Correlati1on time or time constant
constexpr float TAU_G = 50.0f;
// Initial set of covariance
constexpr float P_P_INIT = 10.0f;
constexpr float P_V_INIT = 1.0f;
constexpr float P_A_INIT = 0.34906f;
constexpr float P_HDG_INIT = 3.14159f;
constexpr float P_AB_INIT = 0.9810f;
constexpr float P_GB_INIT = 0.01745f;
// acceleration due to gravity
constexpr float G = 9.807f;
// major eccentricity squared
constexpr double ECC2 = 0.0066943799901;
// earth semi-major axis radius (m)
constexpr double EARTH_RADIUS = 6378137.0;

class imuData {
    public:
        float gyroX;
        float gyroY;
        float gyroZ;
        float accX;
        float accY;
        float accZ;
        float hX;
        float hY;
        float hZ;
};

class ekfNavINS {
  public:
    // constructor
    ekfNavINS() {
      theta = 0.0f;
      phi = 0.0f;
      psi = 0.0f;
    }
    // // returns the pitch angle, rad
    float getPitch_rad()        { return theta; }
    // returns the roll angle, rad
    float getRoll_rad()         { return phi; }
    float getHeading_rad()      { return psi; }
    // return pitch, roll and yaw
    std::tuple<float, float, float> getPitchRollYaw(
      float ax, float ay, float az,
      float gx, float gy, float gz,
      float hx, float hy, float hz,
      float dt);

  private:
    // estimated attitude
    float phi, theta, psi;
    // magnetic heading corrected for roll and pitch angle
    float Bxc, Byc;
};