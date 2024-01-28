// Note code was adapted from the below source:

// -------------------------------------------------------------------
// Mahony AHRS for the ICM_20948  S.J. Remington 6/2021
// Requires the Sparkfun ICM_20948 library
// Standard sensor orientation X North (yaw=0), Y West, Z up
// magnetometer Y and Z axes are reflected to reconcile with accelerometer.

// New Mahony filter error scheme uses Up (accel Z axis) and West (= Acc cross Mag) as the orientation reference vectors
// heavily modified from http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
// Both the accelerometer and magnetometer MUST be properly calibrated for this program to work.
// Follow the procedure described in http://sailboatinstruments.blogspot.com/2011/08/improved-magnetometer-calibration.html
// or in more detail, the tutorial https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/
// -------------------------------------------------------------------

#include "imu.h" 
#include <fstream> 
#include <stdio.h>
#include <csignal>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <string.h>
#include <chrono>
#include <thread>
#include <math.h>

#define PI 3.14159265358979323846F

Imu imu_module;

//Gyro default scale 250 dps. Convert to radians/sec subtract offsets
float G_offset[3] = {74.3, 153.8, -5.5};

//Accel scale: divide by 16604.0 to normalize
float A_B[3]
{   79.60,  -18.56,  383.31};

float A_Ainv[3][3]
{ {  1.00847,  0.00470, -0.00428},
  {  0.00470,  1.00846, -0.00328},
  { -0.00428, -0.00328,  0.99559}
};

//Mag scale divide by 369.4369.4 to normalize
float M_B[3] = { 0.48860 , 0.88597 , 1.08226 };


float M_Ainv[3][3]
{
  {0.34274, 0.001731, 0.00114}, 
  {0.001731, 0.32467, -0.000853}, 
  {0.001134, -0.000853, 0.34557}
};

// local magnetic declination in degrees
float declination = -14.84;

// These are the free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
// Kp is not yet optimized (slight overshoot apparent after rapid sensor reorientations). Ki is not used.
#define Kp 50.0
#define Ki 0.0

// Vector to hold quaternion
static float q[4] = {1.0, 0.0, 0.0, 0.0};
static float yaw, pitch, roll; //Euler angle output

// Define a flag to indicate if the program should exit gracefully.
volatile bool exit_flag = false;
// std::ofstream outfile;

// Signal handler function for Ctrl+C (SIGINT)
void signal_handler(int signum) {
    if (signum == SIGINT) {
        std::cout << "Ctrl+C received. Cleaning up..." << std::endl;

        // Set the exit flag to true to trigger graceful exit.
        exit_flag = true;
    }
}

// Scaling the IMU data
void get_scaled_IMU(float Gxyz[3], float Axyz[3], float Mxyz[3]);
// Mahony AHRS filter
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);

const float alpha = 0.5; // Alpha value for the filter, between 0 and 1.

void lowPassFilter(float input[], float output[]) {
    for (int i = 0; i < 3; i++) {
        output[i] = output[i] + alpha * (input[i] - output[i]);
    }
}

int main(void) {
  // Register the signal handler
    signal(SIGINT, signal_handler);

    static float Gxyz[3], Axyz[3], Mxyz[3]; //centered and scaled gyro/accel/mag data 

	// Open the file in write mode (this will truncate the existing file)
    // outfile.open("tests/mahony_AHRS/ypr_data.txt", std::ios::out);
    // if (!outfile.is_open()) {
    //     std::cerr << "Unable to open file for writing." << std::endl;
    //     return -1;
    // }

    float filteredGxyz[3] = {0, 0, 0}; // Initialize filtered values
    float filteredAxyz[3] = {0, 0, 0};
    float filteredMxyz[3] = {0, 0, 0};

    auto last = std::chrono::high_resolution_clock::now();
    
    while(!exit_flag) {
      // All data for IMU is normalized already for 250dps, 2g, and 4 gauss
      imu_module.ReadSensorData();
      get_scaled_IMU(Gxyz, Axyz, Mxyz);

        // Apply low-pass filter
        lowPassFilter(Gxyz, filteredGxyz);
        lowPassFilter(Axyz, filteredAxyz);
        lowPassFilter(Mxyz, filteredMxyz);

      // printf("Gyro (rad/s): (X: %f, Y: %f, Z: %f)\n", Gxyz[0], Gxyz[1], Gxyz[2]);
      // printf("Acceleration (m/s^2): (X: %f, Y: %f, Z: %f)\n", Axyz[0], Axyz[1], Axyz[2]);
      // printf("Magnetometer (uTesla): (X: %f, Y: %f, Z: %f)\n", Mxyz[0], Mxyz[1], Mxyz[2]);

      printf("Filtered Gyroscope (rad/s): (X: %f, Y: %f, Z: %f)\n", filteredGxyz[0], filteredGxyz[1], filteredGxyz[2]);
      printf("Filtered Acceleration (m/s^2): (X: %f, Y: %f, Z: %f)\n", filteredAxyz[0], filteredAxyz[1], filteredAxyz[2]);
      printf("Filtered Magnetometer (uTesla): (X: %f, Y: %f, Z: %f)\n", filteredMxyz[0], filteredMxyz[1], filteredMxyz[2]);

      // reconcile magnetometer and accelerometer axes. X axis points magnetic North for yaw = 0

      // Mxyz[1] = -Mxyz[1]; //reflect Y and Z
      // Mxyz[2] = -Mxyz[2]; //must be done after offsets & scales applied to raw data

      filteredMxyz[1] = -filteredMxyz[1]; //reflect Y and Z
      filteredMxyz[2] = -filteredMxyz[2]; //must be done after offsets & scales applied to raw data

       auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> elapsed = now - last;
        float deltat = elapsed.count();
        last = now;

      //MahonyQuaternionUpdate(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2],
      //                      Mxyz[0], Mxyz[1], Mxyz[2], deltat);
      MahonyQuaternionUpdate(filteredAxyz[0], filteredAxyz[1], filteredAxyz[2], filteredGxyz[0], filteredGxyz[1], filteredGxyz[2],
                            filteredMxyz[0], filteredMxyz[1], filteredMxyz[2], deltat);

      // Define Tait-Bryan angles. Strictly valid only for approximately level movement
      
      // Standard sensor orientation : X magnetic North, Y West, Z Up (NWU)
      // this code corrects for magnetic declination.
      // Pitch is angle between sensor x-axis and Earth ground plane, toward the
      // Earth is positive, up toward the sky is negative. Roll is angle between
      // sensor y-axis and Earth ground plane, y-axis up is positive roll.
      // Tait-Bryan angles as well as Euler angles are
      // non-commutative; that is, the get the correct orientation the rotations
      // must be applied in the correct order.
      //
      // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
      // which has additional links.
            
      // WARNING: This angular conversion is for DEMONSTRATION PURPOSES ONLY. It WILL
      // MALFUNCTION for certain combinations of angles! See https://en.wikipedia.org/wiki/Gimbal_lock
      
      roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
      pitch = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
      yaw   = atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - ( q[2] * q[2] + q[3] * q[3]));
      // to degrees
      // yaw   *= 180.0 / PI;
      // pitch *= 180.0 / PI;
      // roll *= 180.0 / PI;

      // http://www.ngdc.noaa.gov/geomag-web/#declination
      //conventional nav, yaw increases CW from North, corrected for local magnetic declination

      // yaw = -(yaw + declination);
      // if (yaw < 0) yaw += 360.0;
      // if (yaw >= 360.0) yaw -= 360.0;

      printf("yaw: %f, pitch: %f, roll: %f\n", yaw, pitch, roll);
	   // Log the data to the file
        // outfile << yaw << "," << pitch << "," << roll << std::endl;

      
      // Sleep for a bit to limit the update rate
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
          std::ofstream outfile("tests/mahony_AHRS/rpy_data.txt");
          if (outfile.is_open()) {
              outfile << roll << "," << pitch << "," << yaw << std::endl;
              outfile.flush(); // Flush the stream
              outfile.close(); // Close the file to save the changes
          } else {
              std::cerr << "Unable to open file for writing." << std::endl;
          }

      printf("--------------------\n");
    }

   // Clean up and close the file
    // outfile.close();
    return 0;
}



// vector math
float vector_dot(float a[3], float b[3])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vector_normalize(float a[3])
{
  float mag = sqrt(vector_dot(a, a));
  a[0] /= mag;
  a[1] /= mag;
  a[2] /= mag;
}

// function to subtract offsets and apply scale/correction matrices to IMU data

void get_scaled_IMU(float Gxyz[3], float Axyz[3], float Mxyz[3]) {
    const int16_t *accel_data = imu_module.GetRawAccelerometerData();
    if (accel_data[0] == ACCEL_MAX_THRESHOLD && accel_data[1] == ACCEL_MAX_THRESHOLD && accel_data[2] == ACCEL_MAX_THRESHOLD) {
        printf("Accelerometer data is invalid.\n");
    }
    // else {
    //     printf("Acceleration (m/s^2): (X: %d, Y: %d, Z: %d)\n", accel_data[0], accel_data[1], accel_data[2]);
    // }
    const int16_t *gyro_data = imu_module.GetRawGyroscopeData();
    if (gyro_data[0] == GYRO_MAX_THRESHOLD && gyro_data[1] == GYRO_MAX_THRESHOLD && gyro_data[2] == GYRO_MAX_THRESHOLD) {
        printf("Gyroscope data is invalid.\n");
    }
    // else {
    //     printf("Gyroscope (radians/s): (X: %d, Y: %d, Z: %d)\n", gyro_data[0], gyro_data[1], gyro_data[2]);
    // }

    const int16_t *mag_data = imu_module.GetRawMagnetometerData();
    if (mag_data[0] == MAG_MAX_THRESHOLD && mag_data[1] == MAG_MAX_THRESHOLD && mag_data[2] == MAG_MAX_THRESHOLD) {
        printf("Magnetometer data is invalid.\n");
    }
    // else {
    //     printf("Magnetometer (uTesla): (X: %d, Y: %d, Z: %d)\n", mag_data[0], mag_data[1], mag_data[2]);
    // }

    Gxyz[0] = GYRO_SENSITIVITY_250DPS * DEG_TO_RAD * static_cast<float>(gyro_data[0]); //- G_offset[0]);
    Gxyz[1] = GYRO_SENSITIVITY_250DPS * DEG_TO_RAD * static_cast<float>(gyro_data[1]); //- G_offset[1]);
    Gxyz[2] = GYRO_SENSITIVITY_250DPS * DEG_TO_RAD * static_cast<float>(gyro_data[2]); //- G_offset[2]);
    Axyz[0] = static_cast<float>(accel_data[0]) * ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STD;
    Axyz[1] = static_cast<float>(accel_data[1]) * ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STD;
    Axyz[2] = static_cast<float>(accel_data[2]) * ACCEL_MG_LSB_2G * SENSORS_GRAVITY_STD;
    Mxyz[0] = static_cast<float>(mag_data[0]) * MAG_UT_LSB;;
    Mxyz[1] = static_cast<float>(mag_data[1]) * MAG_UT_LSB;;
    Mxyz[2] = static_cast<float>(mag_data[2]) * MAG_UT_LSB;;

    //apply accel offsets (bias) and scale factors from Magneto
    uint8_t i;
    float temp[3];

    for (i = 0; i < 3; i++) temp[i] = (Axyz[i] - A_B[i]);
    Axyz[0] = A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2];
    Axyz[1] = A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2];
    Axyz[2] = A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2];
    vector_normalize(Axyz);

    //apply mag offsets (bias) and scale factors from Magneto

    for (i = 0; i < 3; i++) temp[i] = (Mxyz[i] - M_B[i]);
    Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
    Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
    Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
    vector_normalize(Mxyz);
}

// Mahony orientation filter, assumed World Frame NWU (xNorth, yWest, zUp)
// Modified from Madgwick version to remove Z component of magnetometer:
// The two reference vectors are now Up (Z, Acc) and West (Acc cross Mag)
// sjr 3/2021
// input vectors ax, ay, az and mx, my, mz MUST be normalized!
// gx, gy, gz must be in units of radians/second
//
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
  // Vector to hold integral error for Mahony method
  static float eInt[3] = {0.0, 0.0, 0.0};
  // short name local variable for readability
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm;
  float hx, hy, hz;  //observed West horizon vector W = AxM
  float ux, uy, uz, wx, wy, wz; //calculated A (Up) and W in body frame
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Measured horizon vector = a x m (in body frame)
  hx = ay * mz - az * my;
  hy = az * mx - ax * mz;
  hz = ax * my - ay * mx;
  // Normalise horizon vector
  norm = sqrt(hx * hx + hy * hy + hz * hz);
  if (norm == 0.0f) return; // Handle div by zero

  norm = 1.0f / norm;
  hx *= norm;
  hy *= norm;
  hz *= norm;

  // Estimated direction of Up reference vector
  ux = 2.0f * (q2q4 - q1q3);
  uy = 2.0f * (q1q2 + q3q4);
  uz = q1q1 - q2q2 - q3q3 + q4q4;

  // estimated direction of horizon (West) reference vector
  wx = 2.0f * (q2q3 + q1q4);
  wy = q1q1 - q2q2 + q3q3 - q4q4;
  wz = 2.0f * (q3q4 - q1q2);

  // Error is the summed cross products of estimated and measured directions of the reference vectors
  // It is assumed small, so sin(theta) ~ theta IS the angle required to correct the orientation error.

  ex = (ay * uz - az * uy) + (hy * wz - hz * wy);
  ey = (az * ux - ax * uz) + (hz * wx - hx * wz);
  ez = (ax * uy - ay * ux) + (hx * wy - hy * wx);

  if (Ki > 0.0f)
  {
    eInt[0] += ex;      // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
    // Apply I feedback
    gx += Ki * eInt[0];
    gy += Ki * eInt[1];
    gz += Ki * eInt[2];
  }


  // Apply P feedback
  gx = gx + Kp * ex;
  gy = gy + Kp * ey;
  gz = gz + Kp * ez;


 //update quaternion with integrated contribution
 // small correction 1/11/2022, see https://github.com/kriswiner/MPU9250/issues/447
gx = gx * (0.5*deltat); // pre-multiply common factors
gy = gy * (0.5*deltat);
gz = gz * (0.5*deltat);
float qa = q1;
float qb = q2;
float qc = q3;
q1 += (-qb * gx - qc * gy - q4 * gz);
q2 += (qa * gx + qc * gz - q4 * gy);
q3 += (qa * gy - qb * gz + q4 * gx);
q4 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}