/*
 * Copyright (c) 2023, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CATIE_SIXTRON_BRUSHLESS_L4A6_HARDWARE_SPECIFIC_H
#define CATIE_SIXTRON_BRUSHLESS_L4A6_HARDWARE_SPECIFIC_H

// STM32 LL LIBRARIES
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_cortex.h"
#include "stm32l4xx_ll_crs.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_exti.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_pwr.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_system.h"
#include "stm32l4xx_ll_tim.h"
#include "stm32l4xx_ll_utils.h"

/**
 * Clock configuration from MBED:
 *-----------------------------------------------------------------------------
 * System clock source | 1- USE_PLL_HSI (internal 16 MHz)
 *                     | 2- USE_PLL_MSI (internal 100kHz to 48 MHz) --> USED
 *-----------------------------------------------------------------------------
 * SYSCLK(MHz)         | 80
 * AHBCLK (MHz)        | 80
 * APB1CLK (MHz)       | 80
 * APB2CLK (MHz)       | 80
 * USB capable         | YES
 *-----------------------------------------------------------------------------
 **/

namespace sixtron {

extern SPI_HandleTypeDef hspi1;

#define MOTOR_MAX_PWM 2000 // 80MHz / 2000 = 40kHz

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
void init_spi_mainboard();

} // namespace sixtron

#endif // CATIE_SIXTRON_BRUSHLESS_L4A6_HARDWARE_SPECIFIC_H
