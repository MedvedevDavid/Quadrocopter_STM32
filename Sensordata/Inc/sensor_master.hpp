#ifndef _SENSOR_MASTER_HPP_
#define _SENSOR_MASTER_HPP_

#include "MPU6050.hpp"

#include "Kalman.h"
#include "UART_Communication.hpp"

namespace sensormaster {

struct raw_angle {
  float x_angle, y_angle;
};

class SensorData {

  MPU6050::RawData_Def my_accel_raw, my_gyro_raw;
  MPU6050::ScaledData_Def my_accel_scaled, my_gyro_scaled;
  MPU6050::MPU6050 mpu6050_connector;

public:
  SensorData(I2C_HandleTypeDef *);
  void Get_Accel_Scale(MPU6050::ScaledData_Def *my_accel_scaled);
  void Get_Gyro_Scale(MPU6050::ScaledData_Def *my_gyro_scaled);
  raw_angle calc_xy_angles();
};

class SensorProcessing {
private:
  float previous_Time = HAL_GetTick();
  float corrent_Time = HAL_GetTick();
  float dt_in_sec;
  uint32_t timer_to_UART = 0;
  Kalman kalman_x, kalman_y;
  float filtered_anggle_x, filtered_anggle_y;
  MPU6050::RawData_Def my_accel_raw, my_gyro_raw;
  MPU6050::ScaledData_Def my_accel_scaled, my_gyro_scaled;
  SensorData *sensor_handdler;
  float magneto_yaw;

public:
  SensorProcessing(I2C_HandleTypeDef *);
  ~SensorProcessing();
  void sensor_data_processing();
  float get_filtered_angle_x();
  float get_filtered_angle_y();
  float get_magnetometer_yaw();
  void magnetometer_sensor_data_processing();
};

} // namespace sensormaster

#endif