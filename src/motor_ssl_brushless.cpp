/*
 * Copyright (c) 2023, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

// CATIE_SIXTRON_MOTOR_SSL_BRUSHLESS_CPP

#include "motor_ssl_brushless.h"

namespace sixtron {

static int _pwm_value = 0;
static int _sector = 0;
static int32_t _rotation = 0;
static int call_interrupt_at_next_pwm_update
        = 0; // This is a security, if pwm not 0 anymore, and interrupt would not launch because
             // motor don't move yet

typedef struct {
    GPIO_TypeDef *pwm_port;
    uint32_t pwm_pin;
    GPIO_TypeDef *en_port;
    uint32_t en_pin;
    volatile uint32_t *pwm_compare_register;
} motor_control_phase_drive_t;

static const motor_control_phase_drive_t motor_phases[3] = {
    {PWM_U_GPIO_Port, PWM_U_Pin, EN_U_GPIO_Port, EN_U_Pin, &(TIM1->CCR1)},
    {PWM_V_GPIO_Port, PWM_V_Pin, EN_V_GPIO_Port, EN_V_Pin, &(TIM1->CCR2)},
    {PWM_W_GPIO_Port, PWM_W_Pin, EN_W_GPIO_Port, EN_W_Pin, &(TIM1->CCR3)}
};

// internal function for BDLC
static inline int _motor_control_get_hall(void);
void _motor_control_update_sector(void);
static inline void _motor_control_set_HZ(motor_control_phase_drive_t phase);
static inline void _motor_control_set_ground(motor_control_phase_drive_t phase);
static inline void _motor_control_set_pwm(motor_control_phase_drive_t phase, uint32_t pwm_value);
void motor_control_stop(void);
void _motor_control_update_pwm(int sector, int pwm_value);
void custom_EXTI_IRQHandler(void);

MotorSSLBrushless::MotorSSLBrushless(float rate_dt,
        PID_params motor_pid,
        MotorSensor *sensor,
        float max_pwm,
        float max_speed_ms):
        _sensor(sensor), _pid(motor_pid, rate_dt) {

    _pid.setLimit(sixtron::PID_limit::output_limit_HL, max_pwm);
    _pid.setLimit(sixtron::PID_limit::input_limit_HL, max_speed_ms);

    _currentStatus = motor_status::stop;
}

void MotorSSLBrushless::init() {

    // Init GPIOS HALL and Enables
    init_gpios();

    // Init TIMER1 and thus all pwm
    init_pwms();

    // Init interruptions
    init_interrupt();

    // Force to call interrupt for the first update, because motor don't move yet
    call_interrupt_at_next_pwm_update = 1;

    // Update HALL sector at least one time
    _motor_control_update_sector();
    _motor_control_update_pwm(_sector, _pwm_value);
}

volatile int hall_value_debug_raw;

static inline int _motor_control_get_hall(void) {
    static const unsigned int hall_to_phase[6] = { 0, 2, 1, 4, 5, 3 };

    int hall_value = ((((HALL_U_GPIO_Port->IDR & HALL_U_Pin) != 0u) ? 1 : 0) << 2)
            | ((((HALL_V_GPIO_Port->IDR & HALL_V_Pin) != 0u) ? 1 : 0) << 1)
            | ((((HALL_W_GPIO_Port->IDR & HALL_W_Pin) != 0u) ? 1 : 0));

    hall_value_debug_raw = hall_value;

    if ((hall_value >= 1) && (hall_value <= 6)) { // hall value ok
        return hall_to_phase[hall_value - 1];
    } else { // not a valid value
        return -1;
    }
}

int MotorSSLBrushless::get_last_hall_value() {
    return hall_value_debug_raw;
}

void _motor_control_update_sector(void) {
    static int old_sector = 0;
    int delta;

    // should not happend
    //! \todo ADD TIMEOUT !!!!!!
    while ((_sector = _motor_control_get_hall()) == -1)
        ;

    delta = _sector - old_sector;
    old_sector = _sector;

    if (delta <= -3) {
        _rotation += delta + 6;
    } else if (delta >= 3) {
        _rotation += delta - 6;
    } else {
        _rotation += delta;
    }
}

static inline void _motor_control_set_HZ(motor_control_phase_drive_t phase) {
    *(phase.pwm_compare_register) = 0;
    phase.en_port->BSRR = (uint32_t)phase.en_pin << 16u;
}

static inline void _motor_control_set_ground(motor_control_phase_drive_t phase) {
    *(phase.pwm_compare_register) = 0;
    phase.en_port->BSRR = phase.en_pin;
}

static inline void _motor_control_set_pwm(motor_control_phase_drive_t phase, uint32_t pwm_value) {
    *(phase.pwm_compare_register) = pwm_value;
    phase.en_port->BSRR = phase.en_pin;
}

void motor_control_stop(void) {
    _motor_control_set_HZ(motor_phases[0]);
    _motor_control_set_HZ(motor_phases[1]);
    _motor_control_set_HZ(motor_phases[2]);
}

// static inline
void _motor_control_update_pwm(int sector, int pwm_value) {
    static const int grounded_pin[6] = { 1, 1, 0, 0, 2, 2 };
    static const int HZ_pin[6] = { 2, 0, 1, 2, 0, 1 };
    static const int pmw_pin[6] = { 0, 2, 2, 1, 1, 0 };

    static const int phase_order_direct_rotation[6] = { 1, 2, 3, 4, 5, 0 };
    static const int phase_order_indirect_rotation[6] = { 4, 5, 0, 1, 2, 3 };
    uint32_t pwm;
    int drive_sector;

    // reindexing phases according to rotation
    call_interrupt_at_next_pwm_update = 0;
    if (pwm_value > 0) {
        drive_sector = phase_order_direct_rotation[sector];
        pwm = (uint32_t)pwm_value;
    } else if (pwm_value < 0) {
        drive_sector = phase_order_indirect_rotation[sector];
        pwm = (uint32_t)(-pwm_value);
    } else {
        motor_control_stop();
        // force to call interrupt at next update, because motor will not move
        call_interrupt_at_next_pwm_update = 1;
        return;
    }

    _motor_control_set_HZ(motor_phases[HZ_pin[drive_sector]]);
    _motor_control_set_ground(motor_phases[grounded_pin[drive_sector]]);
    _motor_control_set_pwm(motor_phases[pmw_pin[drive_sector]], pwm);
}

void custom_EXTI_IRQHandler(void) {

    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_2);

    //    debug led
    //    if (HAL_GPIO_ReadPin(LED_GPIO_Port, LED_Pin) == GPIO_PIN_RESET) {
    //        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    //    } else {
    //        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    //    }

    _motor_control_update_sector();
    _motor_control_update_pwm(_sector, _pwm_value);
}

void MotorSSLBrushless::init_interrupt() {

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(HALL_U_EXTI_IRQn, 0, 0);
    NVIC_SetVector(HALL_U_EXTI_IRQn, (uint32_t)&custom_EXTI_IRQHandler);
    HAL_NVIC_EnableIRQ(HALL_U_EXTI_IRQn);

    HAL_NVIC_SetPriority(HALL_V_EXTI_IRQn, 0, 0);
    NVIC_SetVector(HALL_V_EXTI_IRQn, (uint32_t)&custom_EXTI_IRQHandler);
    HAL_NVIC_EnableIRQ(HALL_V_EXTI_IRQn);

    HAL_NVIC_SetPriority(HALL_W_EXTI_IRQn, 0, 0); //
    NVIC_SetVector(HALL_W_EXTI_IRQn, (uint32_t)&custom_EXTI_IRQHandler); //
    HAL_NVIC_EnableIRQ(HALL_W_EXTI_IRQn); //
}

void MotorSSLBrushless::start() {
    if (_currentStatus == motor_status::stop) {
        _currentStatus = motor_status::run;
    }
}

void MotorSSLBrushless::stop() {
    if (_currentStatus == motor_status::run) {
        _currentStatus = motor_status::stop;
    }
}

void MotorSSLBrushless::update() {
    // update magnetic sensor value
    _sensor->update();

    // update PID
    _currentSpeed = _sensor->getSpeed();
    PID_args motor_pid_args;
    motor_pid_args.actual = _currentSpeed;
    motor_pid_args.target = _targetSpeed;
    _pid.compute(&motor_pid_args);

    //    _motorPwm = motor_pid_args.output;

    _pwm_value = int(motor_pid_args.output);

    if (call_interrupt_at_next_pwm_update) {
        _motor_control_update_sector();
        _motor_control_update_pwm(_sector, _pwm_value);
    }

    // update hardware
}

void MotorSSLBrushless::setSpeed(float speed_ms) {
    _targetSpeed = speed_ms;
    call_interrupt_at_next_pwm_update = 1;
}

// void MotorSSLBrushless::setPWM(int pwm) {
//     _pwm_value = pwm;
//     if (call_interrupt_at_next_pwm_update) {
//         _motor_control_update_sector();
//         _motor_control_update_pwm(_sector, _pwm_value);
//     }
// }

float MotorSSLBrushless::getSpeed() {
    return _currentSpeed;
}

int MotorSSLBrushless::getPWM() {
    return _pwm_value;
}

} // namespace sixtron
