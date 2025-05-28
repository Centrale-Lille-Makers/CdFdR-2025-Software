#include <stdio.h>
#include <cinttypes>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Arduino.h"
#include "driver/gpio.h"

// #include "module/motion.h"
#include "libs/ld19p.h"
#include "module/IHM.h"
#include "module/lift.h"
#include "module/motion.h"
#include "config.h"

static const char *TAG = "main";


Motion *motion = NULL;
lift *liftG = NULL;
lift *liftD = NULL;
IHM *ihm = NULL;
ld19p *lidar = NULL;

void setup()
{
    ESP_LOGI(TAG, "START");

    // Initializing UART coms for steppers
    //M_DRIVE_SERIAL.begin(115200, SERIAL_8N1, M_DRIVE_RX, M_DRIVE_TX);
    //M_CHARIOT_SERIAL.begin(115200, SERIAL_8N1, M_CHARIOT_RX, M_CHARIOT_TX);
    //vTaskDelay(pdMS_TO_TICKS(100)); // for the UART com to settle (not proved to be usefull, but if it aint brock dont fix it)

    //ESP_LOGI(TAG, "Motor serial initialized");

    //motion = new Motion();
    //ESP_LOGI(TAG, "Motion initialized");
    
    liftG = new lift(M5_STP_PIN, M5_DIR_PIN, M_CHARIOT_EN_PIN, PUMP_VALVE1_PIN, ELECTRO1_PIN, M5_MICROSTEP, 'G');
    liftD = new lift(M6_STP_PIN, M6_DIR_PIN, M_CHARIOT_EN_PIN, PUMP_VALVE2_PIN, ELECTRO2_PIN, M6_MICROSTEP, 'D');
    ESP_LOGI(TAG, "Lifts initialized");
    
    ihm = new IHM(CLK_IHM_PIN, DIO_IHM_PIN, STB_IHM_PIN);
    ESP_LOGI(TAG, "IHM initialized");
    
    lidar = new ld19p(LIDAR_SERIAL, LIDAR_RX_PIN);
    ESP_LOGI(TAG, "LiDAR initialized");
    
    // to change

#define STEPS_HZ 200 * 5
#define ACCEL_TIME_S 1

    // stepper1->setSpeedInHz(STEPS_HZ * M1_MICROSTEP);
    // stepper1->setAcceleration(STEPS_HZ * M1_MICROSTEP / ACCEL_TIME_S);
    // stepper1->attachToPulseCounter(); // Needed, because getCurrentPosition() is not precise when using RMT driver (wich is mandatory with esp-idf=5)

    // stepper2->setSpeedInHz(STEPS_HZ * M2_MICROSTEP);
    // stepper2->setAcceleration(STEPS_HZ * M2_MICROSTEP / ACCEL_TIME_S);
    // stepper2->attachToPulseCounter();

    // stepper3->setDirectionPin(M3_DIR_PIN);
    // stepper3->setSpeedInHz(STEPS_HZ * M3_MICROSTEP);
    // stepper3->setAcceleration(STEPS_HZ * M3_MICROSTEP / ACCEL_TIME_S);
    // //stepper3->attachToPulseCounter();

    // stepper4->setDirectionPin(M4_DIR_PIN);
    // stepper4->setSpeedInHz(STEPS_HZ * M4_MICROSTEP);
    // stepper4->setAcceleration(STEPS_HZ * M4_MICROSTEP / ACCEL_TIME_S);
    // //stepper4->attachToPulseCounter();
}

extern "C" void app_main(void)
{
    initArduino();
    setup();

    //ledc_stepper stepper(M5_STP_PIN, M5_DIR_PIN, M_CHARIOT_EN_PIN, true);

    // TMC2209Stepper driver(&M_CHARIOT_SERIAL, M_R_SENSE, M5_DRIVER_ADDRESS);
    // driver.begin();
    // driver.toff(5);
    // driver.rms_current(M_CHARIOT_CURRENT_MA);
    // driver.microsteps(M_CHARIOT_MICROSTEP);

    // stepper.reset_position();
    // stepper.enable();
    // while (1) {
    // stepper.go_to(200*32, 200*32);
    // stepper.go_to(0, 200*32);
    // }

    ihm->setLED(0, 1);
    //liftG->calibrate(ihm);
    liftG->reset_all();
    ihm->setLED(1, 1);
    //liftD->calibrate(ihm);
    liftD->reset_all();
    ihm->setLED(2, 1);
    //motion->calibrate(ihm);
    
    while (1) {
        liftD->go_to(80, false);
        liftG->go_to(80);
        liftD->wait();
        vTaskDelay(pdMS_TO_TICKS(1000));
        liftD->disable_suction();
        liftD->disable_magnets();
        liftG->disable_suction();
        liftG->disable_magnets();

        liftG->go_to(40, false);
        liftD->go_to(10);
        liftG->wait();
        liftD->enable_suction();
        liftG->enable_suction();
        liftG->enable_magnets();
        liftD->enable_magnets();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // WARNING: if program reaches end of function app_main() the MCU will
    // restart.
}
