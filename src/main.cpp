/*
 * Copyright (c) 2023, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#include "USBSerial.h"
#include "mbed.h"
#include "motor.h"
#include "pid.h"

static DigitalOut led1(PB_1);

#define CONTROL_RATE 1s
#define CONTROL_FLAG 0x01
Ticker controlTicker;
EventFlags controlFlag;

void controlFlagUpdate() {
    controlFlag.set(CONTROL_FLAG);
}

// Set up printf over USB
USBSerial usb_serial(false);

FileHandle *mbed::mbed_override_console(int fd) {
    return &usb_serial;
}

// Setup terminal custom printf fonction
static char terminal_buff[64];
static int32_t terminal_length;

static void terminal_printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    terminal_length = vsprintf(terminal_buff, fmt, args);
    va_end(args);
    usb_serial.write(terminal_buff, terminal_length);
}

int main() {
    // Setup USB Serial
    usb_serial.init();
    usb_serial.connect();

    // Setup control ticker
    controlTicker.attach(&controlFlagUpdate, CONTROL_RATE);

    int printf_incr = 0;
    while (true) {
        // Wait for ticker flag
        controlFlag.wait_any(CONTROL_FLAG);

        // Do control loop
        led1 = !led1;
        if (led1) {
            terminal_printf("Alive! (i=%d)\n", printf_incr);
            printf_incr++;
        }
    }
}
