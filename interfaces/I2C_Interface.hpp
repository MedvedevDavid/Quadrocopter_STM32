#ifndef _I2C_INTERFACE_HPP_
#define _I2C_INTERFACE_HPP_

#include <stdint.h>

class I2C_Interface{
    public:
        virtual bool I2C_Master_Transmit(uint16_t DevAddress, uint8_t *pData, uint16_t Size) = 0;
        virtual bool I2C_Master_Receive( uint16_t DevAddress, uint8_t *pData, uint16_t Size) = 0;
};

#endif