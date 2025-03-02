#ifndef __MOTOR_CONTROLLER_HPP
#define __MOTOR_CONTROLLER_HPP
#include "data_types.hpp"

namespace motor_controller
{

enum class Motor_controll_state
{
    iddle,
    running,
    stoped,
    calib,
    emergent
};

class Motor_controller
{
    private:
        Motor_controll_state motor_controll_state = Motor_controll_state::iddle;
        bldc_driver_data_type calculate_motor_controll_signal(int16_t controll_pitch, int16_t controll_roll, uint16_t acceleration);
        void calculate_motor_state(Controll_state PID_controll_state);

    public:
        bldc_driver_data_type update_motor_driver(controll_data_type controll_Date_from_queue);
};
}

#endif //__MOTOR_CONTROLLER_HPP