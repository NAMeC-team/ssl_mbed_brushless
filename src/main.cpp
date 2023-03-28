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
#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "ssl_data.pb.h"
static SPISlave spi_mainboard(DRV_MOSI, DRV_MISO, DRV_CLK, DRV_CS); // mosi, miso, sclk, ssel
uint8_t receive_buffer[BrushlessToMainBoard_size + 5];
uint8_t transmit_buffer[BrushlessToMainBoard_size + 5];
uint32_t error_counter = 0;
bool spi_alive = false;

// Motor
#include "motor_ssl_brushless.h"
sixtron::MotorSSLBrushless *motor;

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

int main() {

    led = 0;
    controlFlag = 0;

    // Setup USB Serial
    usb_serial.init();
    usb_serial.connect();

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

    wait_us(1000000);
    motor->setSpeed(0.8f);

    int printf_incr = 0;
    while (true) {

        // Wait for motor update ticker flag
        //        controlFlag.wait_any(CONTROL_FLAG); // Can't pool SPISlave with this.
        //        if (controlFlag.get() == CONTROL_FLAG) { // This doesn't seem to work...
        if (controlFlag) {
            controlFlag = 0;

            // Do control loop
            motor->update();

            if (printf_incr > 10) {
                printf_incr = 0;
                terminal_printf("AS5047 sensor value: %8lld\tspeed: %6dmm/s (pwm=%4d)\thall: %d\r",
                        sensor->getTickCount(),
                        int32_t(sensor->getSpeed() * 1000.0f),
                        motor->getPWM(),
                        motor->get_last_hall_value());

            } else {
                printf_incr++;
            }
        }
    }
}
