#include <string>
#include <cstring>
#include "UART_Communication.hpp"
#include <cstdlib>
#include <cstdint>
#include "parameters.hpp"

void UartLogger::dataTransferFinished()
{
    delivery_ended();
    data_delivered();
}

UartLogger::UartLogger(UART_HandleTypeDef *given_port)
{
    datadelivery_ongoing = false;
    my_UartPort=given_port;
}

UartLogger *UartLogger::getUartLoggerSingeleton()
{
    return uartlogger_instance;
}


UartLogger *UartLogger::getUartLoggerSingeleton(UART_HandleTypeDef *given_port)
{
    if(uartlogger_instance == nullptr)
    {
        uartlogger_instance = new UartLogger(given_port);
    }

    return uartlogger_instance;
}


void UartLogger::add_data_logger(const char* to_be_printed)
{

    std::string str(to_be_printed);
    append_to_buffer(str);
    data_delivered();
}

void UartLogger::delivery_ended()
{

    datadelivery_ongoing = false;
}

void UartLogger::send_data_logger()
{
    data_to_be_delivered = data_to_be_delivered_buffer;
    data_to_be_delivered_buffer.clear();
    const char *message = data_to_be_delivered.c_str(); 
    HAL_UART_Transmit_DMA(my_UartPort, reinterpret_cast<const uint8_t*>(message), strlen(message));
}


void UartLogger::data_delivered()
{
    if (!datadelivery_ongoing && !data_to_be_delivered_buffer.empty())
    {
        datadelivery_ongoing = true;
        send_data_logger();
    }
}

void UartLogger::append_to_buffer(std::string string_to_be_appended)
{
    if (data_to_be_delivered_buffer.length()<MAXIMUM_BUFFER_SIZE)
    {
        data_to_be_delivered_buffer.append(string_to_be_appended);
    }
    
}

 UartLogger *UartLogger::uartlogger_instance = nullptr;