#ifndef __PARAMETERS_HPP
#define __PARAMETERS_HPP

// Sensor position parameters
constexpr float X_ANGLE_COMPENSATION = 3.31f;
constexpr float Y_ANGLE_COMPENSATION = -1.31f;

constexpr float RAD_TO_DEG = 57.295779513082320876798154814105f;
constexpr double pi = 3.14159;

// Sendor Parameters
constexpr float SENSOR_OFFSET = 0;

// Motor controll Parameters
constexpr int16_t MOTOR_OFF = -500;
constexpr int16_t MOTOR_MAX = 1000;
constexpr int16_t PHISICAL_MOTOR_MAX = 1000;
constexpr int16_t MOTOR_IDDLE_TICK = 1000;
constexpr int16_t MOTOT_FORM_IDDLE_TO_MAX_TICK = 1000;
constexpr int16_t MOTOR_ACCELERATION_DECREASE = -500;
constexpr int16_t ACCELERATION_MOTOR_CONST = 10;

constexpr uint32_t NEW_DATA_FORM_SENSOR = 0x00000001U;
constexpr uint32_t NEW_DATA_FORM_CONTROLL = 0x00000010U;

constexpr float MAXIMUM_ACCELERATION = 130;

// IBUS Parameters
constexpr int16_t IBUS_LENGTH = 0x20; // 32 bytes
constexpr int16_t IBUS_COMMAND40 =
    0x40; // Command to set servo or motor speed is always 0x40
constexpr int16_t IBUS_MAX_CHANNLES = 14;

constexpr uint16_t STICK_VALUE_AT_MIDDLE = 0x5DC;
constexpr uint16_t STICK_VALUE_AT_MINIMUM = 0x3EA;
constexpr uint16_t STICK_VALUE_FROM_MIN_TO_MAX = 0x3DB;
constexpr uint16_t STICK_VALUE_FROM_MIN_TO_MAX_HALF = 0x1F2;
// Controller Parameters
constexpr float MAXIMUM_CONTROLL_ANGLE = 60;

// UART Logger parameters
constexpr uint16_t MAXIMUM_BUFFER_SIZE = 100;
constexpr uint16_t LOGGING_MULTIPLIER = 100;

// PID parameters
constexpr float YAW_P = 2;
constexpr float YAW_I = 0;
constexpr float YAW_D = 0;

constexpr float YAW_SCALING_FACTOR = 0.00001;

constexpr float ROLL_P = 0.5;
constexpr float ROLL_I = 0;
constexpr float ROLL_D = 0.4;

constexpr float PITCH_P = 0.5;
constexpr float PITCH_I = 0;
constexpr float PITCH_D = 0.4;

constexpr float PID_INTEGRATION_START = MAXIMUM_ACCELERATION / 2;

constexpr float MAX_INTEGRATOR_BUFFER = 200;

#endif // __PARAMETERS_HPP