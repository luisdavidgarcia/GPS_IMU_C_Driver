// Modified by: Luis D. Garcia
// Edited to become IMU only

/*
*) Refactor the code to remove reduentent part.
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

#include "ekfNavINS.h"
#include "stdio.h"

std::tuple<float, float, float> ekfNavINS::getPitchRollYaw(
    float ax, float ay, float az,
    float gx, float gy, float gz,
    float hx, float hy, float hz,
    float dt)
{
  // Previous attitude (from the last call)
  float prevTheta = theta;
  float prevPhi = phi;
  float prevPsi = psi;

  // Update the attitude from accelerometer
  theta = asinf(std::max(-1.0f, std::min(1.0f, ax)));
  float cos_theta = cosf(theta);
  if (fabs(cos_theta) < 1e-6)
    cos_theta = cos_theta < 0 ? -1e-6 : 1e-6;
  float corrected_ay = std::max(-1.0f, std::min(1.0f, ay / cos_theta));
  phi = -asinf(corrected_ay);

  // Update heading from magnetometer
  Bxc = hx * cosf(theta) + (hy * sinf(phi) + hz * cosf(phi)) * sinf(theta);
  Byc = hy * cosf(phi) - hz * sinf(phi);
  psi = -atan2f(Byc, Bxc);

  return std::make_tuple(theta, phi, psi);
}
