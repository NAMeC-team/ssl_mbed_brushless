/*
 * Copyright (c) 2023, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CATIE_SIXTRON_MOTOR_SSL_BRUSHLESS_H
#define CATIE_SIXTRON_MOTOR_SSL_BRUSHLESS_H

// MBED LIBRARIES
#include "mbed.h"
#include "motor/motor.h"
#include "motor_sensor/motor_sensor.h"
#include "pid/pid.h"

// Hardware functions, different for each board
#include "hardware_specific.h"

namespace sixtron {

class MotorSSLBrushless: Motor {

public:
    MotorSSLBrushless(float rate_dt,
            PID_params motor_pid,
            MotorSensor *sensor,
            float max_pwm = MOTOR_MAX_PWM);

    void init() override;

    void start() override;

    void stop() override;

    void update() override;

    void setSpeed(float speed_ms) override;

    //    void setPWM(int pwm);

    float getSpeed() override;

    int get_last_hall_value();

private:
    static void init_interrupt();

    MotorSensor *_sensor;

    PID _pid;

    motor_status _currentStatus;
    float _targetSpeed = 0.0f;
    float _currentSpeed = 0.0f;
    float _motorPwm = 0.0f;
};

} // namespace sixtron

#endif // CATIE_SIXTRON_MOTOR_SSL_BRUSHLESS_H
