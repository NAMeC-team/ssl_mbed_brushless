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

// DEFAULT_MAX_PWM: The max pwm the pid can output
// for 3.5m/s max no-load speed -> max_pwm = MOTOR_MAX_PWM / 5
// for 2.3m/s max no-load speed -> max_pwm = MOTOR_MAX_PWM / 7
// for 1.5m/s max no-load speed -> max_pwm = MOTOR_MAX_PWM / 10
#define DEFAULT_MAX_PWM (MOTOR_MAX_PWM / 7.0f)

// DEFAULT_MAX_SPEED: The max input speed (in m/s) the PID can process.
// Should be lesser than the MAX_PWM for better correction !
#define DEFAULT_MAX_SPEED (1.5f)

class MotorSSLBrushless: Motor {

public:
    MotorSSLBrushless(float rate_dt,
            PID_params motor_pid,
            MotorSensor *sensor,
            float max_pwm = DEFAULT_MAX_PWM,
            float max_speed_ms = DEFAULT_MAX_SPEED);

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
