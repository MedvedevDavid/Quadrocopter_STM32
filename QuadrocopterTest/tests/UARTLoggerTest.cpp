#include <gtest/gtest.h>
#include "UART_Communication.hpp"


TEST(UARTLoggerTest, LoggerInitialization_negative){
    UART_HandleTypeDef testUART;
    UartLogger *Logger = UartLogger::getUartLoggerSingeleton(&testUART);
    Logger->add_data_logger("logg_it");
    EXPECT_EQ(debugg_output, "logg_it");
    Logger->dataTransferFinished();
}

TEST(UARTLoggerTest, LoggerInitialization){
    UART_HandleTypeDef testUART;
    UartLogger *Logger = UartLogger::getUartLoggerSingeleton(&testUART);
    Logger->add_data_logger("logg_it");
    EXPECT_EQ(debugg_output, "logg_it");
    Logger->dataTransferFinished();
    
    Logger->add_data_logger("logg_that");
    EXPECT_EQ(debugg_output, "logg_that");
    Logger->dataTransferFinished();
    Logger->add_data_logger(1123);
    EXPECT_EQ(debugg_output, "1123");
    Logger->dataTransferFinished();
    
}

TEST(UARTLoggerTest, LoggerMultiple_Messages){
    UART_HandleTypeDef testUART;
    UartLogger *Logger = UartLogger::getUartLoggerSingeleton(&testUART);
    Logger->add_data_logger("logg_it");
    EXPECT_EQ(debugg_output, "logg_it");
    Logger->add_data_logger("logg_it");
    Logger->add_data_logger("logg_that");
    Logger->dataTransferFinished();
    EXPECT_EQ(debugg_output, "logg_itlogg_that");
}

