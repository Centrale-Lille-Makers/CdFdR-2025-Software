#pragma once

// #include "TMCStepper.h"
#include "FastAccelStepper.h"
#include "IHM.h"
#include "config.h"

typedef struct
{
    double x; // mm
    double y; // mm
    double r; // rotation en degrés
} position_t;

typedef struct
{
    int32_t steps;  // full steps
    uint32_t speed; // steps/s
    int32_t accel;  // steps/s^2
    uint32_t jerk;  // steps to full accel
} stepInstruction_t;

class Motion
{
public:
    Motion();
    void disable_motors();
    void enable_motors();
    void calibrate(IHM *ihm);
    void stop();
    void resume();
    position_t get_pos();
    void correct_pos(position_t relative_pos_error);
    void set_pos(position_t absolute_pos);
    void translate(int distance, double alpha, int speed_robot = MAX_SPEED_ROBOT, int accel_robot = MAX_ACCEL_ROBOT, bool blocking = true);
    void rotate(double dtheta, int rotation_speed_robot = MAX_SPEED_ROBOT, int rotation_accel_robot = MAX_ACCEL_ROBOT, bool blocking = true);
    void wait();
    position_t relative_pos_futur(); // returns the position in 1sec if stop was issued now

private:
    // TMC2209Stepper *M1_driver;
    // TMC2209Stepper *M2_driver;
    // TMC2209Stepper *M3_driver;
    // TMC2209Stepper *M4_driver;

    FastAccelStepperEngine _stepper_engine = FastAccelStepperEngine();
    void execute_moves(double c1, double c2, double c3, double c4, int pos, int speed, int accel, bool blocking = true);
    void execute_steps();
    FastAccelStepper *_M1_stepper = NULL;
    FastAccelStepper *_M2_stepper = NULL;
    FastAccelStepper *_M3_stepper = NULL;
    FastAccelStepper *_M4_stepper = NULL;

    stepInstruction_t _current_objective[4];
    position_t _pos;     // position that was *last time the steps were reset*
    bool _moved = false; // if a move was made, meaning pos is not updated anymore.

    gpio_num_t _en_pin;

    // void set_RMS(uint16_t current);
    // void set_microstep(uint16_t ms);
    position_t _objective_pos = {0, 0, 0};
    void update_pos();
    position_t get_pos_delta(uint8_t type = 0);
    bool _stopped = false;
    bool is_running();
    SemaphoreHandle_t _stepping_mutex;
};