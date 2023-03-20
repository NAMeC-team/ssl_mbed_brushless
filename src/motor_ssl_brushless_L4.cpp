/*
 * Copyright (c) 2023, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#include "motor_ssl_brushless_L4.h"

namespace sixtron {

static int _pwm_value = 0;
static int _sector = 0;
static int32_t _rotation = 0;
static int call_interrupt_at_next_pwm_update
        = 0; // This is a security, if pwm not 0 anymore, and interrupt would not launch

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

MotorSSLBrushless::MotorSSLBrushless(float rate_dt, PID_params motor_pid, float max_pwm):
        _pid(motor_pid, rate_dt) {

    _pid.setLimit(sixtron::PID_limit::output_limit_HL, max_pwm);

    _currentStatus = motor_status::stop;
}

void MotorSSLBrushless::init() {

    // Init GPIOS HALL and Enables
    init_gpios();

    // Init TIMER1 and thus all pwm
    init_pwms();

    // Init interruptions
    init_interrupt();

    // enable all timer channels
    LL_TIM_EnableCounter(TIM1);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);

    // Be sure all pwm alpha are 0
    LL_TIM_OC_SetCompareCH1(TIM1, 0);
    LL_TIM_OC_SetCompareCH2(TIM1, 0);
    LL_TIM_OC_SetCompareCH3(TIM1, 0);

    // force preaload register sync
    LL_TIM_GenerateEvent_UPDATE(TIM1);

    // Outputs begin
    LL_TIM_EnableAllOutputs(TIM1);

    // Update HALL sector at least one time
    _motor_control_update_sector();
    _motor_control_update_pwm(_sector, _pwm_value);
}

void MotorSSLBrushless::init_gpios() {

    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Configure HALL inputs pins */
    GPIO_InitStruct.Pin = HALL_U_Pin | HALL_V_Pin | HALL_W_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Configure enables pins */
    GPIO_InitStruct.Pin = EN_U_Pin | EN_V_Pin | EN_W_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Configure enables to low */
    HAL_GPIO_WritePin(GPIOC, EN_U_Pin | EN_V_Pin | EN_W_Pin, GPIO_PIN_RESET);
}

void MotorSSLBrushless::init_pwms() {
    // TIMER 1 INITIALISATION (From STM32CubeMX)
    LL_TIM_InitTypeDef TIM_InitStruct = { 0 };
    LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = { 0 };
    LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = { 0 };

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

    TIM_InitStruct.Prescaler = 0;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 1000;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    TIM_InitStruct.RepetitionCounter = 0;
    LL_TIM_Init(TIM1, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIM1);
    LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
    TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.CompareValue = 0;
    TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
    TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH3);
    LL_TIM_SetOCRefClearInputSource(TIM1, LL_TIM_OCREF_CLR_INT_NC);
    LL_TIM_DisableExternalClock(TIM1);
    LL_TIM_ConfigETR(TIM1,
            LL_TIM_ETR_POLARITY_NONINVERTED,
            LL_TIM_ETR_PRESCALER_DIV1,
            LL_TIM_ETR_FILTER_FDIV1);
    LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
    LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_RESET);
    LL_TIM_DisableMasterSlaveMode(TIM1);
    TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
    TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
    TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
    TIM_BDTRInitStruct.DeadTime = 0;
    TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
    TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
    TIM_BDTRInitStruct.BreakFilter = LL_TIM_BREAK_FILTER_FDIV1;
    TIM_BDTRInitStruct.Break2State = LL_TIM_BREAK2_DISABLE;
    TIM_BDTRInitStruct.Break2Polarity = LL_TIM_BREAK2_POLARITY_HIGH;
    TIM_BDTRInitStruct.Break2Filter = LL_TIM_BREAK2_FILTER_FDIV1;
    TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
    LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);

    // PWM OUTPUT INIT
    LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    /**TIM1 GPIO Configuration
    PA8     ------> TIM1_CH1
    PA9     ------> TIM1_CH2
    PA10     ------> TIM1_CH3
    */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static inline int _motor_control_get_hall(void) {
    static const unsigned int hall_to_phase[6] = { 0, 2, 1, 4, 5, 3 };

    int hall_value = (LL_GPIO_IsInputPinSet(GPIOA, HALL_U_Pin) << 2)
            | (LL_GPIO_IsInputPinSet(GPIOA, HALL_V_Pin) << 1)
            | (LL_GPIO_IsInputPinSet(GPIOA, HALL_W_Pin));

    if ((hall_value >= 1) && (hall_value <= 6)) { // hall value ok
        return hall_to_phase[hall_value - 1];
    } else { // not a valid value
        return -1;
    }
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
    LL_GPIO_ResetOutputPin(phase.en_port, phase.en_pin);
}

static inline void _motor_control_set_ground(motor_control_phase_drive_t phase) {
    *(phase.pwm_compare_register) = 0;
    // LL_TIM_GenerateEvent_UPDATE(TIM1);
    LL_GPIO_SetOutputPin(phase.en_port, phase.en_pin);
}

static inline void _motor_control_set_pwm(motor_control_phase_drive_t phase, uint32_t pwm_value) {
    *(phase.pwm_compare_register) = pwm_value;
    // LL_TIM_GenerateEvent_UPDATE(TIM1);
    LL_GPIO_SetOutputPin(phase.en_port, phase.en_pin);
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

    _motor_control_update_sector();
    _motor_control_update_pwm(_sector, _pwm_value);
}

void MotorSSLBrushless::init_interrupt() {

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
    NVIC_SetVector(EXTI0_IRQn, (uint32_t)&custom_EXTI_IRQHandler);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
    NVIC_SetVector(EXTI1_IRQn, (uint32_t)&custom_EXTI_IRQHandler);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);

    HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
    NVIC_SetVector(EXTI2_IRQn, (uint32_t)&custom_EXTI_IRQHandler);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
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
    //    _currentSpeed = _sensor->getSpeed();

    // update PID
    PID_args motor_pid_args;
    motor_pid_args.actual = _currentSpeed;
    motor_pid_args.target = _targetSpeed;
    _pid.compute(&motor_pid_args);

    _motorPwm = motor_pid_args.output;

    // update hardware
}

void MotorSSLBrushless::setSpeed(float speed_ms) {
    _targetSpeed = speed_ms;
}

void MotorSSLBrushless::setPWM(int pwm) {
    _pwm_value = pwm;
    if (call_interrupt_at_next_pwm_update) {
        _motor_control_update_sector();
        _motor_control_update_pwm(_sector, _pwm_value);
    }
}

float MotorSSLBrushless::getSpeed() {
    return _currentSpeed;
}

} // namespace sixtron
