#include <gtest/gtest.h>
#include "motor_controller.hpp"
#include "parameters.hpp"

TEST(MotorControllTest, MotorcontrollCalculation){
    motor_controller::Motor_controller motor_Controller_onject;

    controll_data_type controllData;
    controllData.acceleration = 50;
    controllData.angle_yaw = 0;
    controllData.angle_roll = 0;
    controllData.angle_pitch = 0;
    controllData.PID_controll_state = Controll_state::stop;
    bldc_driver_data_type bldc_driver = motor_Controller_onject.update_motor_driver(controllData);
    EXPECT_EQ(-500, bldc_driver.motor_1);
    EXPECT_EQ(-500, bldc_driver.motor_2);
    EXPECT_EQ(-500, bldc_driver.motor_3);
    EXPECT_EQ(-500, bldc_driver.motor_4);


    controllData.PID_controll_state = Controll_state::start;
    bldc_driver = motor_Controller_onject.update_motor_driver(controllData);
    EXPECT_EQ(0, bldc_driver.motor_1);
    EXPECT_EQ(0, bldc_driver.motor_2);
    EXPECT_EQ(0, bldc_driver.motor_3);
    EXPECT_EQ(0, bldc_driver.motor_4);

    controllData.acceleration = 100;
    bldc_driver = motor_Controller_onject.update_motor_driver(controllData);
    EXPECT_EQ(500, bldc_driver.motor_1);
    EXPECT_EQ(500, bldc_driver.motor_2);
    EXPECT_EQ(500, bldc_driver.motor_3);
    EXPECT_EQ(500, bldc_driver.motor_4);

    controllData.angle_roll =    75;
    bldc_driver = motor_Controller_onject.update_motor_driver(controllData);
    EXPECT_EQ(916, bldc_driver.motor_1);
    EXPECT_EQ(84, bldc_driver.motor_2);
    EXPECT_EQ(84, bldc_driver.motor_3);
    EXPECT_EQ(916, bldc_driver.motor_4);

    controllData.PID_controll_state = Controll_state::stop;
    bldc_driver = motor_Controller_onject.update_motor_driver(controllData);
    EXPECT_EQ(-500, bldc_driver.motor_1);
    EXPECT_EQ(-500, bldc_driver.motor_2);
    EXPECT_EQ(-500, bldc_driver.motor_3);
    EXPECT_EQ(-500, bldc_driver.motor_4);

    controllData.acceleration = 50;
    controllData.angle_yaw = 0;
    controllData.angle_roll = 0;
    controllData.angle_pitch = 0;
    controllData.PID_controll_state = Controll_state::start;
    bldc_driver = motor_Controller_onject.update_motor_driver(controllData);
    EXPECT_EQ(0, bldc_driver.motor_1);
    EXPECT_EQ(0, bldc_driver.motor_2);
    EXPECT_EQ(0, bldc_driver.motor_3);
    EXPECT_EQ(0, bldc_driver.motor_4);
}

