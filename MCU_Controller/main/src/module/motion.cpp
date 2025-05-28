#include "motion.h"
#include "config.h"

static const char *TAG = "motion";


Motion::Motion() {
    // M1_driver = new TMC2209Stepper(&M_DRIVE_SERIAL, M_R_SENSE, M1_DRIVER_ADDRESS);
    // M2_driver = new TMC2209Stepper(&M_DRIVE_SERIAL, M_R_SENSE, M2_DRIVER_ADDRESS);
    // M3_driver = new TMC2209Stepper(&M_DRIVE_SERIAL, M_R_SENSE, M3_DRIVER_ADDRESS);
    // M4_driver = new TMC2209Stepper(&M_DRIVE_SERIAL, M_R_SENSE, M4_DRIVER_ADDRESS);

    // M1_driver->begin();
    // M2_driver->begin();
    // M3_driver->begin();
    // M4_driver->begin();
    
    // M1_driver->toff(5);
    // M2_driver->toff(5);
    // M3_driver->toff(5);
    // M4_driver->toff(5);
    
    // set_RMS(M_DRIVE_CURRENT_MA);
    // set_microstep(M_DRIVE_MICROSTEP);


    M1_stepper = engine.stepperConnectToPin(M1_STP_PIN);
    M1_stepper->setDirectionPin(M1_DIR_PIN);
    //M1_stepper->attachToPulseCounter(); We will try not tu have to use it
    
    M2_stepper = engine.stepperConnectToPin(M2_STP_PIN);
    M2_stepper->setDirectionPin(M2_DIR_PIN);
    //M2_stepper->attachToPulseCounter();

    M3_stepper = engine.stepperConnectToPin(M3_STP_PIN);
    M3_stepper->setDirectionPin(M3_DIR_PIN);

    M4_stepper = engine.stepperConnectToPin(M4_STP_PIN);
    M4_stepper->setDirectionPin(M4_DIR_PIN);

    en_pin = (gpio_num_t)M_DRIVE_EN_PIN;
    gpio_reset_pin(en_pin);
    gpio_set_direction(en_pin, GPIO_MODE_OUTPUT);
    
    disable_motors();
    ESP_LOGI(TAG, "created");
}

void Motion::disable_motors()
{
    gpio_set_level(en_pin, 1);
}

void Motion::calibrate(IHM *ihm)
{
    ihm->writeMsg("c mot  y");
    ihm->wGetButton(7);
    ihm->writeMsg("accepted");
    vTaskDelay(pdMS_TO_TICKS(1000));
    ihm->writeMsg("");
}

void Motion::set_RMS(uint16_t current)
{
    M1_driver->rms_current(current);
    M2_driver->rms_current(current);
    M3_driver->rms_current(current);
    M4_driver->rms_current(current);
}

void Motion::set_microstep(uint16_t ms)
{
    M1_driver->microsteps(ms);
    M2_driver->microsteps(ms);
    M3_driver->microsteps(ms);
    M4_driver->microsteps(ms);
}