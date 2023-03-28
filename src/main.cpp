/*
 * Copyright (c) 2023, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#include "USBSerial.h"
#include "mbed.h"
#include "motor_sensor_mbed_AS5047p.h"
#include "pid.h"

static DigitalOut led(LED1);

#define CONTROL_RATE 10ms
#define CONTROL_FLAG 0x01
Ticker controlTicker;
// EventFlags controlFlag; // can't use EventFlags for this application.
static volatile int controlFlag;

void controlFlagUpdate() {
    //    controlFlag.set(CONTROL_FLAG);
    controlFlag = 1;
}

// Set up printf over USB
USBSerial usb_serial(false);

FileHandle *mbed::mbed_override_console(int fd) {
    return &usb_serial;
}

// Sensor
#define ENC_RESOLUTION 16383
#define MOTOR_REDUCTION 1
#define ENC_WHEEL_RADIUS (0.058f / 2.0f)
static SPI spi_sensor(ENC_MOSI, ENC_MISO, ENC_SCK); // mosi, miso, sclk
sixtron::MotorSensorMbedAS5047P *sensor;

// SPI Driver with nanoPB
#include "hardware_specific.h"
#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "ssl_data.pb.h"
#define MAX_BUFF_INDEX (BrushlessToMainBoard_size + 5)
static uint8_t receive_buffer[MAX_BUFF_INDEX];
static uint8_t transmit_buffer[MAX_BUFF_INDEX];
uint32_t error_counter = 0;
volatile int newCommandReceivedFlag = 0;
MbedCRC<POLY_32BIT_ANSI, 32> ct;
Timeout spi_timeout;

// Motor
#include "motor_ssl_brushless.h"
sixtron::MotorSSLBrushless *motor;
float target_speed = 0; // in m/s

// Setup terminal custom printf fonction
static char terminal_buff[80];
static int32_t terminal_length;

static void terminal_printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    terminal_length = vsprintf(terminal_buff, fmt, args);
    va_end(args);
    usb_serial.write(terminal_buff, terminal_length);
}

static float dt_pid = 0.0f, hz_pid = 0.0f;

// Update target speed
void update_speed(Commands cmd, float speed) {
    if (cmd == Commands_RUN) {
        target_speed = speed;
    } else if (cmd == Commands_STOP) {
        target_speed = 0.0f;
    }

    motor->setSpeed(target_speed);
}

MainBoardToBrushless main_board_message;

void process_new_msg() {

    uint32_t CRCValue = 0;

    main_board_message = MainBoardToBrushless_init_zero;

    if (receive_buffer[0] != 0) {
        pb_istream_t stream = pb_istream_from_buffer(&receive_buffer[5], receive_buffer[0]);

        bool ret = pb_decode(&stream, MainBoardToBrushless_fields, &main_board_message);

        ct.compute((uint8_t *)receive_buffer + 5, receive_buffer[0], &CRCValue);

        if (memcmp(&receive_buffer[1], &CRCValue, sizeof(CRCValue)) == 0 && ret) {
            update_speed(main_board_message.command, main_board_message.speed);
        } else {
            error_counter += 1;
        }

    } else {
        //        update_speed(main_board_message.command, main_board_message.speed);
    }

    // Prepare transmit buffer
    BrushlessToMainBoard brushless_message = BrushlessToMainBoard_init_zero;

    /* Create a stream that will write to our buffer. */
    pb_ostream_t tx_stream = pb_ostream_from_buffer(transmit_buffer + 5, BrushlessToMainBoard_size);

    brushless_message.error_count = error_counter;

    /* Now we are ready to encode the message! */
    pb_encode(&tx_stream, BrushlessToMainBoard_fields, &brushless_message);
    size_t message_length = tx_stream.bytes_written;

    ct.compute((uint8_t *)transmit_buffer + 5, message_length, &CRCValue);
    memcpy(&transmit_buffer[1], &CRCValue, sizeof(CRCValue));

    /* add message length */
    transmit_buffer[0] = message_length;
}

void custom_SPI1_IRQHandler() {

    HAL_SPI_IRQHandler(&sixtron::hspi1);
}

// This is a workaround for F103 boards, sometimes they stuck for no reasons ...
void restart_SPIT_IT() {
    HAL_SPI_Abort_IT(&sixtron::hspi1);
    HAL_SPI_TransmitReceive_IT(
            &sixtron::hspi1, transmit_buffer, receive_buffer, sizeof(receive_buffer));
    update_speed(Commands_STOP, 0.0f); // stop motor just in case
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {

    HAL_SPI_TransmitReceive_IT(
            &sixtron::hspi1, transmit_buffer, receive_buffer, sizeof(receive_buffer));
    newCommandReceivedFlag = 1;
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
    // F103 unstuck hack
    spi_timeout.detach();
    spi_timeout.attach(&restart_SPIT_IT, 100ms);
}

int main() {

    led = 0;
    controlFlag = 0;

    // Setup USB Serial
    usb_serial.init();
    usb_serial.connect();

    // Setup DRV SPI
    sixtron::init_spi_mainboard();
    HAL_NVIC_SetPriority(SPI1_IRQn, 1, 0);
    NVIC_SetVector(SPI1_IRQn, (uint32_t)&custom_SPI1_IRQHandler);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);

    // Start SPI IT
    HAL_SPI_TransmitReceive_IT(
            &sixtron::hspi1, transmit_buffer, receive_buffer, sizeof(receive_buffer));

    // Setup control ticker
    controlTicker.attach(&controlFlagUpdate, CONTROL_RATE);

    // Convert current rate of the loop in seconds (float)
    auto f_secs = std::chrono::duration_cast<std::chrono::duration<float>>(CONTROL_RATE);
    dt_pid = f_secs.count();

    // Intialisation of the sensor
    sensor = new sixtron::MotorSensorMbedAS5047P(&spi_sensor,
            ENC_CS,
            dt_pid,
            ENC_RESOLUTION,
            ENC_RESOLUTION * MOTOR_REDUCTION,
            ENC_WHEEL_RADIUS,
            DIR_NORMAL);
    sensor->init();

    // Initialisation of the motor
    sixtron::PID_params pid_motor_params;
    pid_motor_params.Kp = 250.0f;
    pid_motor_params.Ki = 500.0f;
    pid_motor_params.Kd = 0.00f;
    pid_motor_params.ramp = 3.0f * dt_pid;
    motor = new sixtron::MotorSSLBrushless(dt_pid, pid_motor_params, sensor);
    motor->init(); // see "motor_ssl_brushless.h" for max speed and max pwm.

    // Blink init
    for (int i = 0; i < 10; i++) {
        led = 1;
        wait_us(50000);
        led = 0;
        wait_us(50000);
    }

    // Debug
    int printf_incr = 0;

    // MBED BARE METAL LOOP, SO NO THREADS POSSIBLE, ALL MUST BE POOLED OR INTERRUPTED
    while (true) {

        // If needed, process new protobuf msg
        if (newCommandReceivedFlag) {
            newCommandReceivedFlag = 0;
            process_new_msg();
        }

        // Wait for motor update ticker flag
        //        controlFlag.wait_any(CONTROL_FLAG); // Can't pool SPISlave with this.
        //        if (controlFlag.get() == CONTROL_FLAG) { // This doesn't seem to work...
        if (controlFlag) {
            controlFlag = 0;

            // Do control loop
            motor->update();

            //            if (printf_incr > 10) {
            //                printf_incr = 0;
            //                terminal_printf(
            //                        "AS5047: %8lld\tspeed: %5dmm/s
            //                        (pwm=%4d)\ttarget:%5dmm/s\thall: %d\r",
            //                        sensor->getTickCount(),
            //                        int32_t(sensor->getSpeed() * 1000.0f),
            //                        motor->getPWM(),
            //                        int32_t(target_speed * 1000.0f),
            //                        motor->get_last_hall_value());
            //
            //            } else {
            //                printf_incr++;
            //            }
        }
    }
}
