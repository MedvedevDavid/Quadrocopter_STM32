#ifndef __DATA_TYPES_HPP
#define __DATA_TYPES_HPP
#include <stdint.h>

struct sensor_data_type
{
    float angle_yaw;
    float angle_roll;
    float angle_pitch;
    float sampple_time;
};

enum class Controll_state 
{
    start,
    stop,
    running,
    calib,
    no_data
};

struct ControllSignal_data_type
{
    float acceleration;
    float angle_yaw;
    float angle_roll;
    float angle_pitch;
    Controll_state state;
};

struct controll_data_type
{
    float acceleration;
    float angle_yaw;
    float angle_roll;
    float angle_pitch;
    Controll_state PID_controll_state;
};

struct bldc_driver_data_type
{ 
    int16_t motor_1;
    int16_t motor_2;
    int16_t motor_3;
    int16_t motor_4;
};


#endif //__DATA_TYPES_HPP