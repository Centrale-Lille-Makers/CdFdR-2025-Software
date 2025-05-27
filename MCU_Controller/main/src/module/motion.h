#pragma once

#include "TMCStepper.h"
#include "FastAccelStepper.h"

typedef struct {
    float x; // mm
    float y; // mm
    float r; // rotation en degrés
} position;

class Motion {
public:
    Motion();
    void disable_motors();

private:
    TMC2209Stepper *M1_driver;
    TMC2209Stepper *M2_driver;
    TMC2209Stepper *M3_driver;
    TMC2209Stepper *M4_driver;

    FastAccelStepperEngine engine = FastAccelStepperEngine();
    FastAccelStepper *M1_stepper = NULL;
    FastAccelStepper *M2_stepper = NULL;
    FastAccelStepper *M3_stepper = NULL;
    FastAccelStepper *M4_stepper = NULL;
    gpio_num_t en_pin;

    void set_RMS(uint16_t current);
    void set_microstep(uint16_t ms);
};