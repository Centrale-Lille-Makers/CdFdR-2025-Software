#pragma once

#include <stdint.h>
#include "driver/gpio.h"
#include "libs/ledc_stepper.h"
#include "module/ihm.h"
#include "config.h"
#include "TMCStepper.h"

class lift {
public:
    lift(uint8_t step_pin, uint8_t dir_pin, uint8_t en_pin, uint8_t suction_pin, uint8_t magnet_pin, uint8_t uart_adress, char tag);
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
    void calibrate(IHM ihm);
    void stop_motor();

private:
    TMC2209Stepper *driver;
    ledc_stepper *stepper;
    gpio_num_t _suction_pin;
    gpio_num_t _magnet_pin;
    static constexpr int steps_per_mm = M_CHARIOT_STEPS_PER_TURN * M_CHARIOT_MICROSTEP/M_CHARIOT_TRAVEL_PER_TURN;
    static constexpr int speed_stp = M_CHARIOT_SPEED_MM*steps_per_mm;
    char tag;
};