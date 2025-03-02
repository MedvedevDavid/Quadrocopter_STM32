#include "stm32f4xx_hal.h"
#include <iostream>
#include <cstdint>
#include <cstring>
std::string debugg_output;


HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{ 
    std::string str_holder ((char *)pData);
    debugg_output.resize(Size);
    debugg_output = str_holder;
    return (HAL_OK);
};