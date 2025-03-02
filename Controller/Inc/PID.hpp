#ifndef __PID_HPP
#define __PID_HPP
#include "data_types.hpp"

class PID_controller
{
    float parameter_P = 0;
    float parameter_I = 0;
    float parameter_D = 0;
    float integrator_buffer =0;
    float dervator_buffer = 0;
    float previous_Time = 0;

    public:
        PID_controller(float, float, float, float);
        float get_PID_controll_signal(float controll_signal, float feedback_signal, float current_Time, Controll_state state, float acceleration);

};

class PID_controller_yaw : PID_controller
{
    float controll_signal_internal;
    public:
        float get_PID_controll_signal(float controll_signal, float feedback_signal, float current_Time, Controll_state state, float acceleration);
        PID_controller_yaw(float parameter_P, float parameter_I, float parameter_D, float current_Time) : PID_controller(parameter_P, parameter_I, parameter_D, current_Time){};
};

#endif //__PID_HPP