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
#define M_RX            18 // TMC2208/TMC2224 SoftwareSerial receive pin
#define M_TX            17 // TMC2208/TMC2224 SoftwareSerial transmit pin
#define M_SERIAL_PORT Serial1
#define M1_DIR_PIN 5 // Direction
#define M1_STEP_PIN 6 // Step
#define M1_DRIVER_ADDRESS 0b00
#define R_SENSE 0.11f

//HardwareSerial mySerial(1);
TMC2209Stepper driver(&Serial1, R_SENSE, M1_DRIVER_ADDRESS);

extern "C" void app_main(void)
{
    initArduino();
    printf("Hello world!\n");

    pinMode(M_EN_PIN, OUTPUT);
    pinMode(M1_STEP_PIN, OUTPUT);
    pinMode(M1_DIR_PIN, OUTPUT);
    digitalWrite(M_EN_PIN, LOW);
    digitalWrite(M1_STEP_PIN, LOW);
    
    Serial1.begin(115200, SERIAL_8N1, M_RX, M_TX);

    driver.begin();
    driver.toff(5);
    driver.rms_current(500); // mA
    driver.pwm_autoscale(true);
 
    printf("Counting to infinity\n");
    int i = 0;
    for (;;)
    {
        // Run 5000 steps and switch direction in software
        driver.microsteps(4);
        for (uint16_t i = 5000; i > 0; i--)
        {
            digitalWrite(M1_STEP_PIN, HIGH);
            delayMicroseconds(160);
            digitalWrite(M1_STEP_PIN, LOW);
            delayMicroseconds(160);
        }
        driver.microsteps(16);
        for (uint16_t i = 5000; i > 0; i--)
        {
            digitalWrite(M1_STEP_PIN, HIGH);
            delayMicroseconds(160);
            digitalWrite(M1_STEP_PIN, LOW);
            delayMicroseconds(160);
        }

        printf("i = %d\n", ++i);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
