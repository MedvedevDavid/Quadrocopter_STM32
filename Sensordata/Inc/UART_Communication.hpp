#ifndef __UART_COMMUNICATION_HPP
#define __UART_COMMUNICATION_HPP
#include "stm32f4xx_hal.h"
#include <string>
#include <cstring>

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

class UartLogger
{
    private:
        UART_HandleTypeDef *my_UartPort;
        std::string data_to_be_delivered_buffer;
        std::string data_to_be_delivered;
        UartLogger (UART_HandleTypeDef *);
        static UartLogger *uartlogger_instance;
        bool datadelivery_ongoing;
        void append_to_buffer(std::string);
        void send_data_logger();
        void data_delivered();

    public:
        void delivery_ended();
        void dataTransferFinished();
        static UartLogger *getUartLoggerSingeleton(UART_HandleTypeDef *);
        static UartLogger *getUartLoggerSingeleton();
        template <typename T>
        void add_data_logger(T to_be_printed)
        {
            std::string str = std::to_string(to_be_printed);
            append_to_buffer(str);
            data_delivered();
        }
        void add_data_logger(const char* to_be_printed);
};

#endif //__UART_COMMUNICATION_HPP