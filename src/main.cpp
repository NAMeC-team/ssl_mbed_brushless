/*
 * Copyright (c) 2023, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#include "USBSerial.h"
#include "mbed.h"

static DigitalOut led1(PB_1);

// Set up printf over USB
USBSerial usb_serial(false);

FileHandle *mbed::mbed_override_console(int fd)
{
    return &usb_serial;
}

// Setup terminal custom printf fonction
static char terminal_buff[64];
static int32_t terminal_length;

static void terminal_printf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    terminal_length = vsprintf(terminal_buff, fmt, args);
    va_end(args);
    usb_serial.write(terminal_buff, terminal_length);
}

int main()
{
    // Setup USB Serial
    usb_serial.init();
    usb_serial.connect();

    int printf_incr = 0;
    while (true) {
        led1 = !led1;
        if (led1) {
            terminal_printf("Alive! (i=%d)\n", printf_incr);
            printf_incr++;
        }
        ThisThread::sleep_for(1s);
    }
}
