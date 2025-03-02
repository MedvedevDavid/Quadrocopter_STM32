#include "PID.hpp"
#include "parameters.hpp"

PID_controller::PID_controller(float parameter_P, float parameter_I, float parameter_D, float current_Time)
{
    this-> parameter_P = parameter_P;
    this-> parameter_I = parameter_I;
    this-> parameter_D = parameter_D;
    this-> previous_Time = current_Time;
}

float PID_controller::get_PID_controll_signal(float controll_signal, float feedback_signal, float current_Time, Controll_state state, float acceleration)
{
    float elapsed_Time = current_Time-previous_Time;
    previous_Time = current_Time;
    
    float error_signal = controll_signal - feedback_signal;
    float controll_signal_P = parameter_P * error_signal;

    float controll_signal_I;
    if (state!=Controll_state::running || acceleration < PID_INTEGRATION_START)
    {
        integrator_buffer = 0;
        controll_signal_I = 0;
    }
    else
    {
        integrator_buffer += error_signal*elapsed_Time;
        integrator_buffer = (integrator_buffer>MAX_INTEGRATOR_BUFFER) ? MAX_INTEGRATOR_BUFFER : integrator_buffer;
        integrator_buffer = (integrator_buffer<-MAX_INTEGRATOR_BUFFER) ? -MAX_INTEGRATOR_BUFFER : integrator_buffer;
        controll_signal_I = parameter_I * integrator_buffer;
    }
    

    float controll_signal_D = ((error_signal - dervator_buffer)*parameter_D)/elapsed_Time;
    dervator_buffer = error_signal;

    return (controll_signal_P+controll_signal_I-controll_signal_D);
}

float PID_controller_yaw::get_PID_controll_signal(float controll_signal, float feedback_signal, float current_Time, Controll_state state, float acceleration)
{
    if (state != Controll_state::running)
    {
        controll_signal_internal = feedback_signal;
    }
    else
    {
        controll_signal_internal += controll_signal * YAW_SCALING_FACTOR;
    }
    
    return(PID_controller::get_PID_controll_signal(controll_signal_internal, feedback_signal, current_Time, state, acceleration));
}