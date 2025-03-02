#include "motor_controller.hpp"
#include "parameters.hpp"

namespace motor_controller
{
    
bldc_driver_data_type Motor_controller::update_motor_driver(controll_data_type controll_Date_from_queue)
{
    calculate_motor_state(controll_Date_from_queue.PID_controll_state);

    int16_t controll_pitch = (static_cast<int16_t>(controll_Date_from_queue.angle_pitch * MOTOT_FORM_IDDLE_TO_MAX_TICK)/180);
    int16_t controll_roll = (static_cast<int16_t>(controll_Date_from_queue.angle_roll * MOTOT_FORM_IDDLE_TO_MAX_TICK)/180);
    int16_t controll_yaw;
    uint16_t controll_acceleration = static_cast<uint16_t>((controll_Date_from_queue.acceleration * ACCELERATION_MOTOR_CONST) + MOTOR_ACCELERATION_DECREASE);

    return calculate_motor_controll_signal( controll_pitch, controll_roll, controll_acceleration);
}


void Motor_controller::calculate_motor_state(Controll_state PID_controll_state)
{
    if(motor_controll_state == Motor_controll_state::iddle && PID_controll_state==Controll_state::calib)
    {
       motor_controll_state = Motor_controll_state::calib;
    }
    else if (motor_controll_state != Motor_controll_state::iddle && PID_controll_state==Controll_state::calib)
    {
        motor_controll_state = Motor_controll_state::emergent;
    }
    else if (motor_controll_state == Motor_controll_state::iddle && PID_controll_state==Controll_state::stop)
    {
        motor_controll_state = Motor_controll_state::stoped;
    }
    else if (motor_controll_state == Motor_controll_state::stoped && PID_controll_state==Controll_state::start)
    {
        motor_controll_state = Motor_controll_state::running;
    }
    else if (motor_controll_state == Motor_controll_state::running && PID_controll_state==Controll_state::stop)
    {
        motor_controll_state = Motor_controll_state::stoped;
    }
    else if (motor_controll_state == Motor_controll_state::calib && PID_controll_state==Controll_state::stop)
    {
        motor_controll_state = Motor_controll_state::stoped;
    }
}

bldc_driver_data_type Motor_controller::calculate_motor_controll_signal(int16_t controll_pitch, int16_t controll_roll, uint16_t acceleration)
{
    bldc_driver_data_type servo_driver_return;
    switch (motor_controll_state)
    {
        case Motor_controll_state::iddle:
            servo_driver_return.motor_1 = MOTOR_OFF;
            servo_driver_return.motor_2 = MOTOR_OFF;
            servo_driver_return.motor_3 = MOTOR_OFF;
            servo_driver_return.motor_4 = MOTOR_OFF;
            break;

        case Motor_controll_state::running:
            servo_driver_return.motor_1 = int16_t(acceleration + controll_pitch + controll_roll);
            servo_driver_return.motor_2 = int16_t(acceleration + controll_pitch - controll_roll);
            servo_driver_return.motor_3 = int16_t(acceleration - controll_pitch - controll_roll);
            servo_driver_return.motor_4 = int16_t(acceleration - controll_pitch + controll_roll);
            break;


        case Motor_controll_state::stoped:
            servo_driver_return.motor_1 = MOTOR_OFF;
            servo_driver_return.motor_2 = MOTOR_OFF;
            servo_driver_return.motor_3 = MOTOR_OFF;
            servo_driver_return.motor_4 = MOTOR_OFF;
            break;

        case Motor_controll_state::emergent:
            servo_driver_return.motor_1 = MOTOR_OFF;
            servo_driver_return.motor_2 = MOTOR_OFF;
            servo_driver_return.motor_3 = MOTOR_OFF;
            servo_driver_return.motor_4 = MOTOR_OFF;
            break;
        case Motor_controll_state::calib:
            servo_driver_return.motor_1 = PHISICAL_MOTOR_MAX;
            servo_driver_return.motor_2 = PHISICAL_MOTOR_MAX;
            servo_driver_return.motor_3 = PHISICAL_MOTOR_MAX;
            servo_driver_return.motor_4 = PHISICAL_MOTOR_MAX;
            break;
    }
    
    servo_driver_return.motor_1 = (servo_driver_return.motor_1>MOTOR_MAX) ? MOTOR_MAX : servo_driver_return.motor_1;
    servo_driver_return.motor_2 = (servo_driver_return.motor_2>MOTOR_MAX) ? MOTOR_MAX : servo_driver_return.motor_2;
    servo_driver_return.motor_3 = (servo_driver_return.motor_3>MOTOR_MAX) ? MOTOR_MAX : servo_driver_return.motor_3;
    servo_driver_return.motor_4 = (servo_driver_return.motor_4>MOTOR_MAX) ? MOTOR_MAX : servo_driver_return.motor_4;

    servo_driver_return.motor_1 = (servo_driver_return.motor_1<-MOTOR_MAX) ? -MOTOR_MAX : servo_driver_return.motor_1;
    servo_driver_return.motor_2 = (servo_driver_return.motor_2<-MOTOR_MAX) ? -MOTOR_MAX : servo_driver_return.motor_2;
    servo_driver_return.motor_3 = (servo_driver_return.motor_3<-MOTOR_MAX) ? -MOTOR_MAX : servo_driver_return.motor_3;
    servo_driver_return.motor_4 = (servo_driver_return.motor_4<-MOTOR_MAX) ? -MOTOR_MAX : servo_driver_return.motor_4;

    return servo_driver_return;
}

}