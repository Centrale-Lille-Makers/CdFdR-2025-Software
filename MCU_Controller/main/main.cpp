#include <stdio.h>
#include <cinttypes>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Arduino.h"
#include "driver/gpio.h"

#include "TMCStepper.h"
#include "FastAccelStepper.h"

#define M_EN GPIO_NUM_4                // Enable is common to all four drive motors
#define M_RX 18               // For setting TMC2209 parameters
#define M_TX 17               // For setting TMC2209 parameters
#define M_SERIAL_PORT Serial1 // For setting TMC2209 parameters
#define M_R_SENSE 0.11f

#define M1_DRIVER_ADDRESS 0b00
#define M1_DIR_PIN 5
#define M1_STEP_PIN 6
#define M1_MICROSTEP 32 // 8->32 16->16 64->128 32->64 wut ???

#define M2_DRIVER_ADDRESS 0b01
#define M2_DIR_PIN 7
#define M2_STEP_PIN 15
#define M2_MICROSTEP 64

#define M3_DRIVER_ADDRESS 0b10
#define M3_DIR_PIN 8
#define M3_STEP_PIN 3
#define M3_MICROSTEP 128

#define M4_DRIVER_ADDRESS 0b11
#define M4_DIR_PIN 46
#define M4_STEP_PIN 9
#define M4_MICROSTEP 16

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = NULL;
FastAccelStepper *stepper2 = NULL;
FastAccelStepper *stepper3 = NULL;
FastAccelStepper *stepper4 = NULL;

void setup()
{
    printf("START\n");

    engine.init(0);

    printf("Engine initialized\n");

    stepper1 = engine.stepperConnectToPin(M1_STEP_PIN);
    stepper2 = engine.stepperConnectToPin(M2_STEP_PIN);
    stepper3 = engine.stepperConnectToPin(M3_STEP_PIN);
    stepper4 = engine.stepperConnectToPin(M4_STEP_PIN);

    printf("Steppers connected\n");

#define STEPS_HZ 200 * 5
#define ACCEL_TIME_S 1

    stepper1->setDirectionPin(M1_DIR_PIN);
    stepper1->setSpeedInHz(STEPS_HZ * M1_MICROSTEP);
    stepper1->setAcceleration(STEPS_HZ * M1_MICROSTEP / ACCEL_TIME_S);
    stepper1->attachToPulseCounter(); // Needed, because getCurrentPosition() is not precise when using RMT driver (wich is mandatory with esp-idf=5)

    stepper2->setDirectionPin(M2_DIR_PIN);
    stepper2->setSpeedInHz(STEPS_HZ * M2_MICROSTEP);
    stepper2->setAcceleration(STEPS_HZ * M2_MICROSTEP / ACCEL_TIME_S);
    stepper2->attachToPulseCounter();

    stepper3->setDirectionPin(M3_DIR_PIN);
    stepper3->setSpeedInHz(STEPS_HZ * M3_MICROSTEP);
    stepper3->setAcceleration(STEPS_HZ * M3_MICROSTEP / ACCEL_TIME_S);
    stepper3->attachToPulseCounter();

    stepper4->setDirectionPin(M4_DIR_PIN);
    stepper4->setSpeedInHz(STEPS_HZ * M4_MICROSTEP);
    stepper4->setAcceleration(STEPS_HZ * M4_MICROSTEP / ACCEL_TIME_S);
    stepper4->attachToPulseCounter();

    printf("Steppers initialized\n");

    gpio_reset_pin(M_EN);
    gpio_set_direction(M_EN, GPIO_MODE_OUTPUT);
    gpio_set_level(M_EN, 0);

    printf("Steppers enabled\n");
}

extern "C" void app_main(void)
{
    initArduino();
    setup();
    int32_t target = 0;
    while (true)
    {
        while (stepper2->isRunning())
        {
            //      esp_task_wdt_reset();
            printf("pos=%" PRId32, stepper2->getCurrentPosition());
            int16_t pcnt = stepper2->readPulseCounter();
            printf("  pcnt=%d", pcnt);
            printf("\n");
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        printf("done\n");
        vTaskDelay(pdMS_TO_TICKS(500));
        printf("move\n");
        target = 200 - target;
        stepper1->moveTo(target*M1_MICROSTEP);
        stepper2->moveTo(target*M2_MICROSTEP);
        stepper3->moveTo(target*M3_MICROSTEP);
        stepper4->moveTo(target*M4_MICROSTEP);
    }
    // WARNING: if program reaches end of function app_main() the MCU will
    // restart.
}
