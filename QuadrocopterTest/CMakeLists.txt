cmake_minimum_required(VERSION 3.12)

set(This QuadrocopterTests)

project(${This} C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

enable_testing()

add_subdirectory(googletest)
set(Headers 
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Inc/data_types.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Inc/parameters.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Controller/Inc/motor_controller.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Controller/Inc/controll_master.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Sensordata/Inc/UART_Communication.hpp
)

set(Sources 
    ${CMAKE_CURRENT_SOURCE_DIR}/../Controller/Src/motor_controller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Controller/Src/controll_master.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../Sensordata/Src/UART_Communication.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/stubs/stm32f4xx_hal.cpp
)
set(CUBEMX_INCLUDE_DIRECTORIES
    ${CMAKE_CURRENT_SOURCE_DIR}/../Core/Inc/
    ${CMAKE_CURRENT_SOURCE_DIR}/../Controller/Inc/
    ${CMAKE_CURRENT_SOURCE_DIR}/../Sensordata/Inc/
    ${CMAKE_CURRENT_SOURCE_DIR}/stubs/
    )
# Add header directories (AFTER add_executable !!)
include_directories(
    ${CUBEMX_INCLUDE_DIRECTORIES}
    ${PROJECT_INCLUDE_DIRECTORIES})
add_library(${This} STATIC ${Sources} ${Headers})
add_subdirectory(tests)