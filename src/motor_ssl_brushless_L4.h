/*
 * Copyright (c) 2023, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CATIE_SIXTRON_MOTOR_SSL_BRUSHLESS_L4_H
#define CATIE_SIXTRON_MOTOR_SSL_BRUSHLESS_L4_H

#include "mbed.h"
#include "motor/motor.h"
#include "pid/pid.h"

namespace sixtron {

#define DEFAULT_MOTOR_MAX_PWM 1.0f // max PWM with mbed is 1.0f

class MotorSSLBrushless: Motor {

public:
    MotorSSLBrushless(float rate_dt, PID_params motor_pid, float max_pwm = DEFAULT_MOTOR_MAX_PWM);

    void init() override;

    void start() override;

    void stop() override;

    void update() override;

    void setSpeed(float speed_ms) override;

    float getSpeed() override;

private:
    PID _pid;

    motor_status _currentStatus;
    float _targetSpeed = 0.0f;
    float _currentSpeed = 0.0f;
    float _motorPwm = 0.0f;
};

} // namespace sixtron

#endif // CATIE_SIXTRON_MOTOR_SSL_BRUSHLESS_L4_H
