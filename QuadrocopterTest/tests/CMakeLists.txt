cmake_minimum_required(VERSION 3.12)

set(This ControllTest)

set(Sources 
    MotorControllTest.cpp
    ControllMasterTest.cpp
    UARTLoggerTest.cpp)

    
add_executable(${This} ${Sources})

target_link_libraries(${This} PUBLIC
    gtest_main
    QuadrocopterTests
)



add_test(
    NAME ${This}
    COMMAND ${This}
)