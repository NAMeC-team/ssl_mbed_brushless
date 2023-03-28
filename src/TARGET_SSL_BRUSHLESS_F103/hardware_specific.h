/*
 * Copyright (c) 2023, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CATIE_SIXTRON_BRUSHLESS_F103_HARDWARE_SPECIFIC_H
#define CATIE_SIXTRON_BRUSHLESS_F103_HARDWARE_SPECIFIC_H

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

namespace sixtron {

/**
 * For STM32F103RBT6, when HSI is used as clock source generator,
 * Mbed will configure APB2CLK Clock frequency output at:
 *  - 48MHz if USBDEVICE is used (by default)
 *  - 64Mhz if not.
 *
 * To get 40kHz frequency, this should be configured:
 *  - 48MHz -> prescal = 0, period = 1200
 *  - 64MHz -> prescal = 0, period = 1600
 *
 * If you want to remove USBDEVICE to get a better period:
 *  - In custom_targets.json, remove the bloc: << "device_has_add": ["USBDEVICE"], >>
 *  - In mbed_app.json, remove: << "drivers-usb" >>
 *  - In main.cpp, remove any code using the USBSerial.
 *
 */

#if (DEVICE_USBDEVICE)
#define MOTOR_MAX_PWM 1200 // 48MHz/1200 = 40kHz
#else
#define MOTOR_MAX_PWM 1600 // 64MHz/1600 = 40kHz
#endif

#define LED_Pin GPIO_PIN_1 // PB1
#define LED_GPIO_Port GPIOB

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

void init_gpios();
void init_pwms();

} // namespace sixtron

#endif // CATIE_SIXTRON_BRUSHLESS_F103_HARDWARE_SPECIFIC_H
