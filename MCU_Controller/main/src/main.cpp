#include <stdio.h>
#include <cinttypes>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Arduino.h"
#include "driver/gpio.h"

// #include "module/motion.h"
#include "modules/IHM.h"
#include "modules/Lift.h"
#include "modules/Motion.h"
#include "modules/Lidar.h"
#include "config.h"

static const char *TAG = "main";

void waitTiretteAndStart();
void calibrate_pos();
void strat1();
void goForward();
void goRotate();
void step1(int l);
void step1_1(double h);
void calibrage();

int start = -1;
Motion *motion = NULL;
Lift *liftG = NULL;
Lift *liftD = NULL;
IHM *ihm = NULL;
Lidar *lidar = NULL;

void setup()
{
    ESP_LOGI(TAG, "START");

    // Initializing UART coms for steppers
    // M_DRIVE_SERIAL.begin(115200, SERIAL_8N1, M_DRIVE_RX, M_DRIVE_TX);
    // M_CHARIOT_SERIAL.begin(115200, SERIAL_8N1, M_CHARIOT_RX, M_CHARIOT_TX);
    // vTaskDelay(pdMS_TO_TICKS(100)); // for the UART com to settle (not proved to be usefull, but if it aint brock dont fix it)
    gpio_reset_pin((gpio_num_t)M_CHARIOT_RX);
    gpio_set_direction((gpio_num_t)M_CHARIOT_RX, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)M_CHARIOT_RX, 0);
    gpio_reset_pin((gpio_num_t)M_CHARIOT_TX);
    gpio_set_direction((gpio_num_t)M_CHARIOT_TX, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)M_CHARIOT_TX, 0);

    // ESP_LOGI(TAG, "Motor serial initialized");

    motion = new Motion();
    ESP_LOGI(TAG, "Motion initialized");

    liftG = new Lift(M5_STP_PIN, M5_DIR_PIN, M_CHARIOT_EN_PIN, PUMP_VALVE1_PIN, ELECTRO1_PIN, M5_MICROSTEP, 'G');
    liftD = new Lift(M6_STP_PIN, M6_DIR_PIN, M_CHARIOT_EN_PIN, PUMP_VALVE2_PIN, ELECTRO2_PIN, M6_MICROSTEP, 'D');
    ESP_LOGI(TAG, "Lifts initialized");

    ihm = new IHM(CLK_IHM_PIN, DIO_IHM_PIN, STB_IHM_PIN);
    ESP_LOGI(TAG, "IHM initialized");

    // Ld19p = new ld19p(LIDAR_SERIAL, LIDAR_RX_PIN);
    // ESP_LOGI(TAG, "Ld19p initialized");

    lidar = new Lidar(BALISE_WIDTH, LIDAR_SERIAL, LIDAR_RX_PIN, motion);
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

    ihm->set_LED(0, 1);
    liftG->calibrate(ihm);
    ihm->set_LED(1, 1);
    liftD->calibrate(ihm);
    ihm->set_LED(2, 1);
    motion->calibrate(ihm);
    ihm->set_LED(3, 1);

    ihm->write_msg("Prog.    ");
    uint8_t btns = ihm->wait_get_buttons();

    int pos = 0;
    while ((btns & 1) == 0)
    {
        btns >>= 1;
        ++pos;
    }
    btns = pos;
    ihm->show_score(btns);
    ihm->write_msg("Sel");

    switch (btns)
    {
    case 0:
        calibrage();
        break;
    case 1:
        calibrage();
        break;
    case 2:
        calibrage();
        break;
    case 4:
    case 5:
    case 6:
    case 7:
    case 3:
        calibrage();
        break;
    default:
        break;
    }

    waitTiretteAndStart();

    switch (btns)
    {
    case 0:
        motion->translate(450, 90, 200, 200, true);
        motion->translate(50, 0, 200, 200, true);
        liftG->go_to(145 - LIFT_0);              // Lever large
        motion->translate(70, 90, 30, 30, true); // Collage robot
        liftG->go_to(130 - LIFT_0);              // Contact ventouse
        liftG->enable_suction();
        vTaskDelay(pdMS_TO_TICKS(500));
        liftG->go_to(142 - LIFT_0);                 // Lever planche
        motion->translate(20, 270, 200, 100, true); // S''eloigner un peu
        liftG->go_to(170 - LIFT_0);                 // Lever planche
        motion->translate(80, 270, 200, 100, true); // S''eloigner
        motion->rotate(180, 150, 150);              // rotation

        vTaskDelay(pdMS_TO_TICKS(1000));
        // Ca marche
        liftD->go_to(145 - LIFT_0);                 // Lever large
        motion->translate(85, 270, 200, 100, true); // collage robot 2
        liftD->go_to(130 - LIFT_0);                 // Contact ventouse 2
        liftD->enable_suction();
        vTaskDelay(pdMS_TO_TICKS(1000));
        liftD->go_to(180 - LIFT_0);                // Lever planche 2
        motion->translate(50, 90, 200, 100, true); // S''eloigner
        motion->translate(50, 0, 200, 200, true);  // Déplacement latéral

        liftD->go_to(130 - LIFT_0);
        liftD->enable_magnets();
        motion->translate(110, 270, 200, 100, true); // Avancer pour contct cannette
        liftD->go_to(135 - LIFT_0);
        vTaskDelay(pdMS_TO_TICKS(500));
        motion->translate(150, 90, 200, 100, true); // Reculer

        motion->rotate(180, 150, 150);              // rotation
        motion->translate(70, 90, 200, 100, true);  // Contact cannettes 2
        motion->translate(100, 0, 200, 200, true);  // Déplacement latéral
        motion->translate(70, 270, 200, 100, true); // Avancer pour contct cannette 2
        liftG->go_to(155 - LIFT_0);
        liftG->enable_magnets();
        vTaskDelay(pdMS_TO_TICKS(500));
        motion->translate(70, 270, 200, 100, true); // Reculer

        vTaskDelay(pdMS_TO_TICKS(5000));
        liftD->go_to(130 - LIFT_0); // En bas
        liftG->go_to(130 - LIFT_0); // En bas

        liftD->disable_magnets();
        liftG->disable_magnets();
        liftG->disable_suction();
        liftD->disable_suction();
        break;
    case 1:
        // liftG->go_to(145-LIFT_0); // Lever large
        // motion->translate(450, 90, 200, 200, true);
        // liftG->go_to(130-LIFT_0); // Contact ventouse
        // motion->translate(50, 0, 200, 200, true);
        // motion->translate(70, 90, 30, 30, true); // Collage robot
        // liftG->enable_suction();
        // vTaskDelay(pdMS_TO_TICKS(500));
        // liftG->go_to(142-LIFT_0); // Lever planche
        // motion->translate(20, 270, 200, 100, true); // S''eloigner un peu
        // liftG->go_to(170-LIFT_0); // Lever planche
        // motion->translate(80, 270, 200, 100, true); // S''eloigner
        // motion->rotate(180, 150, 150); // rotation

        liftG->go_to(145 - LIFT_0, false); // Lever large
        motion->translate(450, 90, 200, 200, true);
        liftG->wait();
        motion->translate(50, 0, 200, 200, true);
        motion->translate(70, 90, 30, 30, true); // Collage robot
        liftG->go_to(135 - LIFT_0);              // Contact ventouse
        liftG->enable_suction();
        vTaskDelay(pdMS_TO_TICKS(500));
        liftG->go_to(142 - LIFT_0);                 // Lever planche
        motion->translate(20, 270, 200, 100, true); // S''eloigner un peu
        liftG->go_to(170 - LIFT_0, false);          // Lever planche
        motion->translate(80, 270, 200, 100, true); // S''eloigner
        motion->rotate(180, 150, 150);              // rotation

        vTaskDelay(pdMS_TO_TICKS(5000));
        break;
    case 2:
        motion->translate(450, 90, 200, 200, true);
        motion->translate(50, 0, 200, 200, true);
        motion->translate(70, 90, 30, 30, true);    // Collage robot
        motion->translate(20, 270, 200, 100, true); // S''eloigner un peu
        motion->translate(80, 270, 200, 100, true); // S''eloigner
        motion->rotate(180, 150, 150);              // rotation

        motion->translate(85, 270, 200, 100, true); // collage robot 2
        motion->translate(50, 90, 200, 100, true);  // S''eloigner
        motion->translate(50, 0, 200, 200, true);   // Déplacement latéral

        liftD->go_to(130 - LIFT_0);
        liftD->enable_magnets();
        motion->translate(110, 270, 200, 100, true); // Avancer pour contct cannette
        vTaskDelay(pdMS_TO_TICKS(500));
        liftD->go_to(150 - LIFT_0);
        motion->translate(150, 90, 200, 100, true); // Reculer

        motion->rotate(180, 150, 150);              // rotation
        motion->translate(70, 90, 200, 100, true);  // Contact cannettes 2
        motion->translate(100, 0, 200, 200, true);  // Déplacement latéral
        motion->translate(70, 270, 200, 100, true); // Avancer pour contct cannette 2
        liftG->go_to(130 - LIFT_0);
        liftG->enable_magnets();
        vTaskDelay(pdMS_TO_TICKS(500));
        liftD->go_to(150 - LIFT_0);
        motion->translate(70, 270, 200, 100, true); // Reculer

        vTaskDelay(pdMS_TO_TICKS(5000));
        liftD->go_to(130 - LIFT_0); // En bas
        liftG->go_to(130 - LIFT_0); // En bas

        break;
    case 4:
        liftD->go_to(150 - LIFT_0, false);
        motion->translate(350, 90, 200, 100, true);
        motion->translate(365, 180, 200, 100, true);
        motion->translate(355, 270, 60, 40, true);
        ihm->show_score(4);
        motion->translate(405, 90, 200, 100, true);
        motion->translate(200, 180, 200, 100, true);
        motion->translate(700, 90, 200, 100, true);
        motion->translate(500, 0, 200, 100, true);
        motion->translate(1050, 277, 60, 40, true);
        ihm->show_score(8);
        motion->translate(200, 90, 200, 100, true);
        motion->translate(850, 180, 200, 100, true);
        motion->translate(1000, 90, 200, 100, true);
        ihm->show_score(18);

        break;
    case 5:
        liftD->go_to(150 - LIFT_0, false);
        motion->translate(350, 90, 200, 100, true);
        motion->translate(365, 0, 200, 100, true);
        motion->translate(355, 270, 60, 40, true);
        ihm->show_score(4);
        motion->translate(405, 90, 200, 100, true);
        motion->translate(200, 0, 200, 100, true);
        motion->translate(700, 90, 200, 100, true);
        motion->translate(500, 180, 200, 100, true);
        motion->translate(1050, 263, 60, 40, true);
        ihm->show_score(8);
        motion->translate(200, 90, 200, 100, true);
        motion->translate(850, 0, 200, 100, true);
        motion->translate(1000, 90, 200, 100, true);
        ihm->show_score(18);

        break;
    // case 5:
    //     liftD->enable_magnets();
    //     liftG->enable_magnets();
    //     for (int i = 0; i < 10; i++)
    //     {

    //         liftD->go_to(130-LIFT_0); // En bas
    //         liftG->go_to(130-LIFT_0); // En bas
    //         vTaskDelay(pdMS_TO_TICKS(5000));
    //         liftD->go_to(150-LIFT_0); // En bas
    //         liftG->go_to(150-LIFT_0); // En bas
    //         vTaskDelay(pdMS_TO_TICKS(5000));
    //     }
    //     liftD->disable_magnets();
    //     liftG->disable_magnets();

    //     break;
    case 6:
        liftD->go_to(155 - LIFT_0, false);
        motion->translate(350, 90, 200, 100, true);
        motion->translate(365, 0, 200, 100, true);
        motion->translate(355, 270, 60, 40, true);
        ihm->show_score(4);
        motion->translate(405, 90, 200, 100, true);
        motion->translate(400, 65, 200, 100, true);
        motion->rotate(720, 75, 50);
        motion->rotate(-720, 75, 50);
        motion->translate(650, 65, 200, 100, true);
        ihm->show_score(14);
        break;
    case 7:
        liftD->go_to(155 - LIFT_0, false);
        motion->translate(350, 90, 200, 100, true);
        motion->translate(365, 180, 200, 100, true);
        motion->translate(355, 270, 60, 40, true);
        ihm->show_score(4);
        motion->translate(405, 90, 200, 100, true);
        motion->translate(400, 115, 200, 100, true);
        motion->rotate(-720, 75, 50);
        motion->rotate(720, 75, 50);
        motion->translate(650, 115, 200, 100, true);
        ihm->show_score(14);
        break;
    default:
        break;
    }

    // while (1) {
    //     liftD->go_to(80, false);
    //     liftG->go_to(80);
    //     liftD->wait();
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    //     liftD->disable_suction();
    //     liftD->disable_magnets();
    //     liftG->disable_suction();
    //     liftG->disable_magnets();

    //     liftD->go_to(10, false);
    //     liftG->go_to(40);
    //     liftG->enable_suction();
    //     liftG->enable_magnets();
    //     liftD->wait();
    //     liftD->enable_suction();
    //     liftD->enable_magnets();
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }

    ESP_LOGI(TAG, "Done");

    // WARNING: if program reaches end of function app_main() the MCU will
    // restart.
}

void waitTiretteAndStart()
{
    ihm->write_msg("Ready   ");
    lidar->start_detection();
    gpio_reset_pin((gpio_num_t)TIRETTE_PIN);
    gpio_set_direction((gpio_num_t)TIRETTE_PIN, GPIO_MODE_INPUT);
    while (!gpio_get_level((gpio_num_t)TIRETTE_PIN))
    {
        while (!gpio_get_level((gpio_num_t)TIRETTE_PIN))
        {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    start = millis() + 100;
    // ihm->show_progress_live(start);
    ihm->show_score(0);
}

void calibrate_pos()
{
    // to write lol
}

void strat1()
{
    // to write
}

// void goForward() {
//     motion->translate(1000, 270);
// }

// void goRotate() {
//     motion->rotate(360*5, 150, 150);
// }

void calibrage()
{
    motion->translate(100, 90);
}

void step1(int l)
{
    motion->translate(l, 90, 100, 100, true);
}

void step1_1(double h)
{
    liftD->go_to(h - LIFT_0, true);
}