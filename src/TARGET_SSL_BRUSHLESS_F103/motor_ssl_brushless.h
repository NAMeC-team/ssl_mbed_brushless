/*
 * Copyright (c) 2023, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CATIE_SIXTRON_MOTOR_SSL_BRUSHLESS_F103_H
#define CATIE_SIXTRON_MOTOR_SSL_BRUSHLESS_F103_H

// MBED LIBRARIES
#include "mbed.h"
#include "motor/motor.h"
#include "pid/pid.h"

// STM32 LL LIBRARIES
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_cortex.h"
// #include "stm32f1xx_ll_crs.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_utils.h"

/**
 * Clock configuration from MBED:
 *-------------------------------------------------------------------------------------------
 * System clock source                | 1- PLL_HSE_EXTC  / DEVICE_USBDEVICE   | 3- PLL_HSI /
 *DEVICE_USBDEVICE | (external 8 MHz clock)                | (internal 8 MHz) | 2- PLL_HSE_XTAL /
 *DEVICE_USBDEVICE    | | (external 8 MHz xtal)                 |
 *-------------------------------------------------------------------------------------------
 * SYSCLK(MHz)                        | 72 / 72                               | 64 / 48
 *-------------------------------------------------------------------------------------------
 * AHBCLK (MHz)                       | 72 / 72                               | 64 / 48
 *-------------------------------------------------------------------------------------------
 * APB1CLK (MHz)                      | 36 / 36                               | 32 / 24
 *-------------------------------------------------------------------------------------------
 * APB2CLK (MHz)                      | 72 / 72                               | 64 / 48 --> BY
 *DEFAULT
 *-------------------------------------------------------------------------------------------
 **/

namespace sixtron {

#define DEFAULT_MOTOR_MAX_PWM 1.0f // max PWM with mbed is 1.0f

#define HALL_U_Pin GPIO_PIN_0 // PA0
#define HALL_U_GPIO_Port GPIOA
#define HALL_U_EXTI_IRQn EXTI0_IRQn
#define HALL_V_Pin GPIO_PIN_1 // PA1
#define HALL_V_GPIO_Port GPIOA
#define HALL_V_EXTI_IRQn EXTI1_IRQn
#define HALL_W_Pin GPIO_PIN_2 // PA2
#define HALL_W_GPIO_Port GPIOA
#define HALL_W_EXTI_IRQn EXTI2_IRQn

#define EN_U_Pin GPIO_PIN_6 // PC6
#define EN_U_GPIO_Port GPIOC
#define EN_V_Pin GPIO_PIN_7 // PC7
#define EN_V_GPIO_Port GPIOC
#define EN_W_Pin GPIO_PIN_8 // PC8
#define EN_W_GPIO_Port GPIOC

#define PWM_U_Pin GPIO_PIN_8 // PA8
#define PWM_U_GPIO_Port GPIOA
#define PWM_V_Pin GPIO_PIN_9 // PA9
#define PWM_V_GPIO_Port GPIOA
#define PWM_W_Pin GPIO_PIN_10 // PA10
#define PWM_W_GPIO_Port GPIOA

class MotorSSLBrushless: Motor {

public:
    MotorSSLBrushless(float rate_dt, PID_params motor_pid, float max_pwm = DEFAULT_MOTOR_MAX_PWM);

    void init() override;

    void start() override;

    void stop() override;

    void update() override;

    void setSpeed(float speed_ms) override;

    void setPWM(int pwm);

    float getSpeed() override;

    int get_last_hall_value();

private:
    static void init_gpios();
    static void init_pwms();
    static void init_interrupt();

    PID _pid;

    motor_status _currentStatus;
    float _targetSpeed = 0.0f;
    float _currentSpeed = 0.0f;
    float _motorPwm = 0.0f;
};

} // namespace sixtron

#endif // CATIE_SIXTRON_MOTOR_SSL_BRUSHLESS_F103_H
