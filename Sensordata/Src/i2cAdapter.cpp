#include "i2cAdapter.hpp"

I2cAdapter::I2cAdapter(I2C_HandleTypeDef *hi2c) { I2Cdev_hi2c = hi2c; }

bool I2cAdapter::I2C_Master_Transmit(uint16_t DevAddress, uint8_t *pData,
                                     uint16_t Size) {
  HAL_I2C_Master_Transmit_DMA(I2Cdev_hi2c, DevAddress, pData, Size);
  return (true);
}

bool I2cAdapter::I2C_Master_Receive(uint16_t DevAddress, uint8_t *pData,
                                    uint16_t Size) {

  return (HAL_I2C_Master_Receive_DMA(I2Cdev_hi2c, DevAddress, pData, Size) ==
          HAL_OK);
}
