// Hardware-in-the-loop I/O. With HIL_ENABLED the sensor tasks read from here instead of their
// hardware (GNSS bytes from a stream, IMU samples from the latest value) and main.cpp fills it
// from micro-ROS topics published by the simulator. Everything above the drivers is unchanged.

#ifndef HIL_H
#define HIL_H

#include "Arduino.h"
#include "../definitions.h"

struct hil_imu_sample_t {
  float acc_x, acc_y, acc_z;     // m/s^2, sensor frame, includes gravity (like the MPU6050)
  float gyro_x, gyro_y, gyro_z;  // rad/s, sensor frame
};

void hil_init();

// GNSS receiver byte stream (UBX frames). One writer (micro-ROS callback), one reader (gps task).
void hil_gnss_push(const uint8_t* data, size_t len);
int hil_gnss_available();
int hil_gnss_read();

// Latest IMU sample. Returns false until the first sample arrives.
void hil_imu_set(const hil_imu_sample_t& sample);
bool hil_imu_get(hil_imu_sample_t* sample);

#endif
