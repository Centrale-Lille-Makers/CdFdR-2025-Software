/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Arduino.h"
#include "driver/gpio.h"

#include <TMCStepper.h>

#define M_EN_PIN 4 // Enable
#define M_RX 18    // TMC2208/TMC2224 SoftwareSerial receive pin
#define M_TX 17    // TMC2208/TMC2224 SoftwareSerial transmit pin
#define M_SERIAL_PORT Serial1
#define M_DIR_PIN 7   // Direction
#define M_STEP_PIN 15 // Step
#define M_STEPbis_PIN 5 // Debug
#define M1_DRIVER_ADDRESS 0b01
#define R_SENSE 0.11f
#define microstep 64 //8->32 16->16 64->128 32->64 wut ???

// HardwareSerial mySerial(1);

extern "C" void app_main(void)
{
    initArduino();
    printf("Hello world!\n");

    pinMode(M_EN_PIN, OUTPUT);
    pinMode(M_STEP_PIN, OUTPUT);
    pinMode(M_STEPbis_PIN, OUTPUT);
    pinMode(M_DIR_PIN, OUTPUT);
    digitalWrite(M_EN_PIN, LOW);

    for (uint16_t i = 200*microstep; i > 0; i--)
    {
        digitalWrite(M_STEP_PIN, HIGH);
        digitalWrite(M_STEPbis_PIN, HIGH);
        delayMicroseconds(1000000/(200*microstep*2));
        digitalWrite(M_STEP_PIN, LOW);
        digitalWrite(M_STEPbis_PIN, LOW);
        delayMicroseconds(1000000/(200*microstep*2));
    }
}
