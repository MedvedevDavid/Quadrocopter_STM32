#ifndef _I2CADAPTER_HPP_
#define _I2CADAPTER_HPP_

#include "I2C_Interface.hpp"
// TODO: include depending on chip family
//#include "stm32f1xx_hal.h"
//#include "stm32f2xx_hal.h"
//#include "stm32f3xx_hal.h"
extern "C"{
    #include "stm32f4xx_hal.h"
}


class I2cAdapter : public I2C_Interface{
    public:
        I2cAdapter(I2C_HandleTypeDef * hi2c);
        bool I2C_Master_Transmit(uint16_t DevAddress, uint8_t *pData, uint16_t Size) override;
        bool I2C_Master_Receive( uint16_t DevAddress, uint8_t *pData, uint16_t Size) override;

    private:
        I2C_HandleTypeDef * I2Cdev_hi2c;
};

#endif