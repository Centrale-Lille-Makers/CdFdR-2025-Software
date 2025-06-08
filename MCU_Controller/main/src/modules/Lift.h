#pragma once

#include <stdint.h>
#include "driver/gpio.h"
#include "libs/LedcStepper.h"
#include "IHM.h"
#include "config.h"
// #include "TMCStepper.h"

class Lift
{
public:
    Lift(uint8_t step_pin, uint8_t dir_pin, uint8_t en_pin, uint8_t suction_pin, uint8_t magnet_pin, uint8_t microsteps, char tag);
    void enable_motor();
    void disable_suction();
    void enable_suction();
    void disable_magnets();
    void enable_magnets();
    void go_to(double h, bool wait = true);
    void wait();
    double get_position();
    void reset_position();
    void reset_all();
    void calibrate(IHM *ihm);
    void stop_motor();

private:
    // TMC2209Stepper *driver;
    LedcStepper *_stepper;
    gpio_num_t _suction_pin;
    gpio_num_t _magnet_pin;
    int _steps_per_mm;
    int _speed_stp;
    char _tag;
};