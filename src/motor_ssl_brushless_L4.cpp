/*
 * Copyright (c) 2023, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#include "motor_ssl_brushless_L4.h"

namespace sixtron {

MotorSSLBrushlessL4::MotorSSLBrushlessL4(float rateHz, PID_params motor_pid, float max_pwm):
        _pid(motor_pid, rateHz) {

    _pid.setLimit(sixtron::PID_limit::output_limit_HL, max_pwm);

    _currentStatus = motor_status::stop;
}

void MotorSSLBrushlessL4::init() {
}

void MotorSSLBrushlessL4::start() {
    if (_currentStatus == motor_status::stop) {
        _currentStatus = motor_status::run;
    }
}

void MotorSSLBrushlessL4::stop() {
    if (_currentStatus == motor_status::run) {
        _currentStatus = motor_status::stop;
    }
}

void MotorSSLBrushlessL4::update() {
    // update magnetic sensor value
    _currentSpeed = _sensor->getSpeed();

    // update PID
    PID_args motor_pid_args;
    motor_pid_args.actual = _currentSpeed;
    motor_pid_args.target = _targetSpeed;
    _pid.compute(&motor_pid_args);

    _motorPwm = motor_pid_args.output;

    // update hardware
}

void MotorSSLBrushlessL4::setSpeed(float speed_ms) {
    _targetSpeed = speed_ms;
}

float MotorSSLBrushlessL4::getSpeed() {
    return _currentSpeed;
}

} // namespace sixtron
