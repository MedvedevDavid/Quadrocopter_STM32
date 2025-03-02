
#include "sensor_master.hpp"
#include "FREE_RTOS_operators.hpp"
#include "HMC5883L.hpp"
#include "MPU6050.hpp"
#include "OsFuncAdapter.hpp"
#include "i2cAdapter.hpp"
#include "parameters.hpp"
#include <memory>

namespace sensormaster {

SensorData::SensorData(I2C_HandleTypeDef *hi2c1) {
  I2Cdev_init(hi2c1);
  std::shared_ptr<I2cAdapter> i2c_adapter_entiti(new I2cAdapter(hi2c1));
  std::shared_ptr<OsFuncProvider> os_functions(new OsFuncProvider());

  MPU6050::MPU_ConfigTypeDef my_MPU_config;
  // 1. Init MPU and the I2C communication

  mpu6050_connector.MPU6050_Init(i2c_adapter_entiti, os_functions);
  // 2.Configure accelerometer and GYRO parameters
  my_MPU_config.Accel_Full_Scale = MPU6050::AFS_SEL_16g;
  my_MPU_config.ClockSource = MPU6050::Internal_8MHz;
  my_MPU_config.CONFIG_DLPF = MPU6050::DLPF_184A_188G_Hz;
  my_MPU_config.Gyro_Full_Scale = MPU6050::FS_SEL_1000;
  my_MPU_config.Sleep_Mode_Bit = 0;
  my_MPU_config.INTA_ENABLED = 1;
  my_MPU_config.Sample_Rate_Devider = 4;

  // neaded for magnetometer
  // my_MPU_config.Bypass_Mode = true;
  my_MPU_config.Bypass_Mode = false;
  mpu6050_connector.MPU6050_Config(&my_MPU_config);
  // HMC5883L_initialize();
}

void SensorData::Get_Accel_Scale(MPU6050::ScaledData_Def *my_accel_scaled) {
  mpu6050_connector.MPU6050_Get_Accel_Scale(my_accel_scaled);
  // check for if all returned values are numbers
  if ((my_accel_scaled->x = my_accel_scaled->x) and
      (my_accel_scaled->y = my_accel_scaled->y) and
      (my_accel_scaled->z = my_accel_scaled->z)) {
    this->my_accel_scaled = *my_accel_scaled;
  }
}

inline void
SensorData::Get_Gyro_Scale(MPU6050::ScaledData_Def *my_gyro_scaled) {
  mpu6050_connector.MPU6050_Get_Gyro_Scale(my_gyro_scaled);
}

raw_angle SensorData::calc_xy_angles(void) {
  raw_angle ret_Val;
  // Using x y and z from accelerometer, calculate x and y angles
  float accel_angle_x, accel_angle_y, result, x_val, y_val, z_val;
  double x2, y2, z2;

  x_val = my_accel_scaled.x;
  y_val = my_accel_scaled.y;
  z_val = my_accel_scaled.z;

  // Work out the squares
  x2 = static_cast<double>(x_val * x_val);
  y2 = static_cast<double>(y_val * y_val);
  z2 = static_cast<double>(z_val * z_val);

  // X Axis
  result = sqrt(y2 + z2);
  result = x_val / result;
  accel_angle_x = atan(result);

  // Y Axis
  result = sqrt(x2 + z2);
  result = y_val / result;
  accel_angle_y = atan(result);
  ret_Val.x_angle = accel_angle_x * 180 / pi;
  ret_Val.y_angle = accel_angle_y * 180 / pi;

  return (ret_Val);
}

SensorProcessing::SensorProcessing(I2C_HandleTypeDef *hi2c1) {
  sensor_handdler = new SensorData(hi2c1);
}

SensorProcessing::~SensorProcessing() { delete sensor_handdler; }

void SensorProcessing::sensor_data_processing() {

  sensor_handdler->Get_Accel_Scale(&my_accel_scaled);
  auto myangle = sensor_handdler->calc_xy_angles();
  sensor_handdler->Get_Gyro_Scale(&my_gyro_scaled);

  corrent_Time = HAL_GetTick();
  dt_in_sec = (corrent_Time - previous_Time) / 1000;

  filtered_anggle_x =
      kalman_x.getAngle(myangle.x_angle, my_gyro_scaled.x, dt_in_sec);
  filtered_anggle_y =
      kalman_y.getAngle(myangle.y_angle, my_gyro_scaled.y, dt_in_sec);

  previous_Time = corrent_Time;

  filtered_anggle_x += X_ANGLE_COMPENSATION;
  filtered_anggle_y += Y_ANGLE_COMPENSATION;
}

float SensorProcessing::get_filtered_angle_x() { return filtered_anggle_x; }

float SensorProcessing::get_filtered_angle_y() { return filtered_anggle_y; }

void SensorProcessing::magnetometer_sensor_data_processing() {
  int16_t x, y, z;

  if (HMC5883L_getReadyStatus()) {
    HMC5883L_getHeading(&x, &y, &z);
  }

  auto myangle = sensor_handdler->calc_xy_angles();

  float y_angle = myangle.y_angle * pi / 180.0;

  float x_angle = myangle.x_angle * pi / 180.0;

  float Mx = x * cos(y_angle) + z * sin(y_angle);
  float My = x * sin(x_angle) * sin(y_angle) + y * cos(x_angle) -
             z * sin(x_angle) * cos(y_angle);
  magneto_yaw = atan2(-My, Mx) * RAD_TO_DEG;
  // magneto_yaw = atan2(y,x)*RAD_TO_DEG;
}

inline float SensorProcessing::get_magnetometer_yaw() { return magneto_yaw; }

} // namespace sensormaster
