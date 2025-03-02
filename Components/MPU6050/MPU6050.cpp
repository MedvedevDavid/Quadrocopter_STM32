// Header files
#include "MPU6050.hpp"

namespace MPU6050 {
void MPU6050::MPU6050_Init(std::shared_ptr<I2C_Interface> I2Chnd,
                           std::shared_ptr<OsFunc_Interface> OsFuncHnd) {
  i2cHandler = I2Chnd;
  OsFunctions = OsFuncHnd;
}

inline void MPU6050::I2C_Read(uint8_t ADDR, uint8_t *i2cBif, uint8_t NofData) {

  uint8_t MPUADDR;
  // Need to Shift address to make it proper to i2c operation
  MPUADDR = (MPU_ADDR << 1);
  i2cBuf[0] = ADDR;
  i2cHandler->I2C_Master_Transmit(MPUADDR, i2cBuf, 1);
  OsFunctions->os_event_transmitt_wait(10);
  if (i2cHandler->I2C_Master_Receive(MPUADDR, i2cBif, NofData)) {
    OsFunctions->os_event_receive_wait(10);
  }
}

inline void MPU6050::I2C_Write8(uint8_t ADDR, uint8_t data) {

  i2cData[0] = ADDR;
  i2cData[1] = data;
  uint8_t MPUADDR = (MPU_ADDR << 1);
  i2cHandler->I2C_Master_Transmit(MPUADDR, i2cData, 2);
  OsFunctions->os_event_transmitt_wait(100);
}

// 4- MPU6050 Initialaztion Configuration
void MPU6050::MPU6050_Config(MPU_ConfigTypeDef *config) {
  uint8_t Buffer = 0; // NOLINT(clang-analyzer-deadcode.DeadStores)

  I2C_Write8(PWR_MAGT_1_REG, 0x80);
  OsFunctions->os_delay(100);

  Buffer = config->ClockSource & 0x07;
  Buffer |= (config->Sleep_Mode_Bit << 6) & 0x40;
  I2C_Write8(PWR_MAGT_1_REG, Buffer);
  OsFunctions->os_delay(100); // should wait 10ms after changeing the clock
  // setting.

  // Set the Digital Low Pass Filter
  Buffer = 0; // NOLINT(clang-analyzer-deadcode.DeadStores)
  Buffer = config->CONFIG_DLPF & 0x07;
  I2C_Write8(CONFIG_REG, Buffer);

  // Select the Gyroscope Full Scale Range
  Buffer = 0; // NOLINT(clang-analyzer-deadcode.DeadStores)
  Buffer = (config->Gyro_Full_Scale << 3) & 0x18;
  I2C_Write8(GYRO_CONFIG_REG, Buffer);

  // Select the Accelerometer Full Scale Range
  Buffer = 0; // NOLINT(clang-analyzer-deadcode.DeadStores)
  Buffer = (config->Accel_Full_Scale << 3) & 0x18;
  I2C_Write8(ACCEL_CONFIG_REG, Buffer);
  // Set SRD To Default
  MPU6050_Set_SMPRT_DIV(config->Sample_Rate_Devider);

  if (config->INTA_ENABLED) {
    I2C_Write8(INT_ENABLE_REG, 0x1);
  }

  if (config->Bypass_Mode) {
    I2C_Write8(INT_PIN_CFG_REG, I2C_BYPASS_EN);
    I2C_Write8(USER_CTRL_REG, 0);
  }

  // Accelerometer Scaling Factor, Set the Accelerometer and Gyroscope Scaling
  // Factor
  switch (config->Accel_Full_Scale) {
  case AFS_SEL_2g:
    accelScalingFactor = (2000.0f / 32768.0f);
    break;

  case AFS_SEL_4g:
    accelScalingFactor = (4000.0f / 32768.0f);
    break;

  case AFS_SEL_8g:
    accelScalingFactor = (8000.0f / 32768.0f);
    break;

  case AFS_SEL_16g:
    accelScalingFactor = (16000.0f / 32768.0f);
    break;

  default:
    break;
  }
  // Gyroscope Scaling Factor
  switch (config->Gyro_Full_Scale) {
  case FS_SEL_250:
    gyroScalingFactor = 250.0f / 32768.0f;
    break;

  case FS_SEL_500:
    gyroScalingFactor = 500.0f / 32768.0f;
    break;

  case FS_SEL_1000:
    gyroScalingFactor = 1000.0f / 32768.0f;
    break;

  case FS_SEL_2000:
    gyroScalingFactor = 2000.0f / 32768.0f;
    break;

  default:
    break;
  }
}

// 5- Get Sample Rate Divider
inline uint8_t MPU6050::MPU6050_Get_SMPRT_DIV(void) {
  uint8_t Buffer = 0;

  I2C_Read(SMPLRT_DIV_REG, &Buffer, 1);
  return Buffer;
}

// 6- Set Sample Rate Divider
inline void MPU6050::MPU6050_Set_SMPRT_DIV(uint8_t SMPRTvalue) {
  I2C_Write8(SMPLRT_DIV_REG, SMPRTvalue);
}

// 7- Get External Frame Sync.
inline uint8_t MPU6050::MPU6050_Get_FSYNC(void) {
  uint8_t Buffer = 0;

  I2C_Read(CONFIG_REG, &Buffer, 1);
  Buffer &= 0x38;
  return (Buffer >> 3);
}

// 8- Set External Frame Sync.
inline void MPU6050::MPU6050_Set_FSYNC(enum EXT_SYNC_SET_ENUM ext_Sync) {
  uint8_t Buffer = 0;
  I2C_Read(CONFIG_REG, &Buffer, 1);
  Buffer &= ~0x38;

  Buffer |= (ext_Sync << 3);
  I2C_Write8(CONFIG_REG, Buffer);
}

// 9- Get Accel Raw Data
void MPU6050::MPU6050_Get_Accel_RawData(RawData_Def *rawDef) {
  uint8_t i2cBuf[2]{};
  uint8_t AcceArr[6], GyroArr[6];

  I2C_Read(INT_STATUS_REG, i2cBuf, 1);
  if ((i2cBuf[0] & 0x01)) {
    I2C_Read(ACCEL_XOUT_H_REG, AcceArr, 6);

    // Accel Raw Data
    rawDef->x = ((AcceArr[0] << 8) + AcceArr[1]); // x-Axis
    rawDef->y = ((AcceArr[2] << 8) + AcceArr[3]); // y-Axis
    rawDef->z = ((AcceArr[4] << 8) + AcceArr[5]); // z-Axis
    // Gyro Raw Data
    I2C_Read(GYRO_XOUT_H_REG, GyroArr, 6);
    GyroRW[0] = ((GyroArr[0] << 8) + GyroArr[1]);
    GyroRW[1] = (GyroArr[2] << 8) + GyroArr[3];
    GyroRW[2] = ((GyroArr[4] << 8) + GyroArr[5]);
  }
}

// 10- Get Accel scaled data (g unit of gravity, 1g = 9.81m/s2)
void MPU6050::MPU6050_Get_Accel_Scale(ScaledData_Def *scaledDef) {

  RawData_Def AccelRData;
  MPU6050_Get_Accel_RawData(&AccelRData);

  // Accel Scale data
  scaledDef->x = ((AccelRData.x + 0.0f) * accelScalingFactor);
  scaledDef->y = ((AccelRData.y + 0.0f) * accelScalingFactor);
  scaledDef->z = ((AccelRData.z + 0.0f) * accelScalingFactor);
}

// 11- Get Accel calibrated data
inline void MPU6050::MPU6050_Get_Accel_Cali(ScaledData_Def *CaliDef) {
  ScaledData_Def AccelScaled;
  MPU6050_Get_Accel_Scale(&AccelScaled);

  // Accel Scale data
  CaliDef->x = (AccelScaled.x) - A_X_Bias; // x-Axis
  CaliDef->y = (AccelScaled.y) - A_Y_Bias; // y-Axis
  CaliDef->z = (AccelScaled.z) - A_Z_Bias; // z-Axis
}
// 12- Get Gyro Raw Data
inline void MPU6050::MPU6050_Get_Gyro_RawData(RawData_Def *rawDef) {

  // Accel Raw Data
  rawDef->x = GyroRW[0];
  rawDef->y = GyroRW[1];
  rawDef->z = GyroRW[2];
}

// 13- Get Gyro scaled data
void MPU6050::MPU6050_Get_Gyro_Scale(ScaledData_Def *scaledDef) {
  RawData_Def myGyroRaw;
  MPU6050_Get_Gyro_RawData(&myGyroRaw);

  // Gyro Scale data
  scaledDef->x = (myGyroRaw.x) * gyroScalingFactor; // x-Axis
  scaledDef->y = (myGyroRaw.y) * gyroScalingFactor; // y-Axis
  scaledDef->z = (myGyroRaw.z) * gyroScalingFactor; // z-Axis
}

// 14- Accel Calibration
inline void MPU6050::_Accel_Cali(float x_min, float x_max, float y_min,
                                 float y_max, float z_min, float z_max) {
  // 1* X-Axis calibrate
  A_X_Bias = (x_max + x_min) / 2.0f;

  // 2* Y-Axis calibrate
  A_Y_Bias = (y_max + y_min) / 2.0f;

  // 3* Z-Axis calibrate
  A_Z_Bias = (z_max + z_min) / 2.0f;
}
} // namespace MPU6050