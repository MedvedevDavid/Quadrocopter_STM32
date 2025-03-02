#ifndef STM32F4_HAL_TASET_H
#define STM32F4_HAL_TASET_H
#include <string>

typedef struct __UART_HandleTypeDef
{
} UART_HandleTypeDef;

typedef enum 
{
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;

extern std::string debugg_output;

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);


#endif /* STM32F4_HAL_TASET_H */
