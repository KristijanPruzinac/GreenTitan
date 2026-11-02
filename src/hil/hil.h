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
  // Gyro z integrated by the simulator at its full rate (rad, wrapped) and the sample's sim
  // stamp (s); stamp 0 = not sent. Samples come over the link in bursts, so the driver gives the
  // mean rate since the previous read from these, as a real gyro read on the ESP would
  // (over ESP time, so a simulation off real time does not change the angle).
  double gyro_z_angle, stamp;
};

void hil_init();

// GNSS receiver byte stream (UBX frames). One writer (micro-ROS callback), one reader (gps task).
void hil_gnss_push(const uint8_t* data, size_t len);
int hil_gnss_available();
int hil_gnss_read();

// Latest IMU sample. Returns false until the first sample arrives, and while no new one has.
void hil_imu_set(const hil_imu_sample_t& sample);
bool hil_imu_get(hil_imu_sample_t* sample);

// ToF frames from hil/tof: blocks of 1 sensor byte (0 left, 1 right) + 64 zones x (range mm
// uint16 LE, status uint8), zone order as tof_frame_t. hil_tof_get returns the latest frame of
// that sensor once (false until a new one has arrived).
#define HIL_TOF_BLOCK (1 + 3 * TOF_ZONES)
void hil_tof_push(const uint8_t* data, size_t len);
bool hil_tof_get(int sensor, tof_frame_t* frame);

// Contact-plate and pack voltage (V), in place of the two ADC dividers. They come as a block on
// hil/tof (all 5 micro-ROS subscriptions of the build are taken): tag byte HIL_POWER_TAG, then
// dock V and pack V as float32 LE (the sim's power model sends one block per message at 10 Hz).
// hil_power_get returns false until the first block and when the last is older than 1 s.
#define HIL_POWER_TAG 0x50
#define HIL_POWER_BLOCK 9
void hil_power_set(float dock_v, float battery_v);
bool hil_power_get(float* dock_v, float* battery_v);

#endif
