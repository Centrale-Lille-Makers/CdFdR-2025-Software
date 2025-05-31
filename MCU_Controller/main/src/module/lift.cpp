#include "lift.h"

static const char *TAG = "lift";


lift::lift(uint8_t step_pin, uint8_t dir_pin, uint8_t en_pin, uint8_t suction_pin, uint8_t magnet_pin, uint8_t microsteps, char tag)
{
    steps_per_mm = (M_CHARIOT_STEPS_PER_TURN * microsteps)/M_CHARIOT_TRAVEL_PER_TURN;
    ESP_LOGI(TAG, "steps_per_mm: %d", steps_per_mm);
    speed_stp = M_CHARIOT_SPEED_MM*steps_per_mm;
    // driver = new TMC2209Stepper(&M_CHARIOT_SERIAL, M_R_SENSE, uart_adress);
    // driver->begin();
    // driver->toff(5);
    // driver->rms_current(M_CHARIOT_CURRENT_MA);
    // driver->microsteps(M_CHARIOT_MICROSTEP);

    stepper = new ledc_stepper(step_pin, dir_pin, en_pin, true);

    _suction_pin = (gpio_num_t)suction_pin;
    gpio_reset_pin(_suction_pin);
    gpio_set_direction(_suction_pin, GPIO_MODE_OUTPUT);
    disable_suction();

    _magnet_pin = (gpio_num_t)magnet_pin;
    gpio_reset_pin(_magnet_pin);
    gpio_set_direction(_magnet_pin, GPIO_MODE_OUTPUT);
    disable_magnets();

    this->tag = tag;
}

void lift::enable_motor()
{
    stepper->enable();
}

void lift::disable_suction()
{
    gpio_set_level(_suction_pin, 0);
}

void lift::enable_suction()
{
    gpio_set_level(_suction_pin, 1);
}

void lift::disable_magnets()
{
    gpio_set_level(_magnet_pin, 0);
}

void lift::enable_magnets()
{
    gpio_set_level(_magnet_pin, 1);
}

void lift::go_to(double h, bool wait)
{
    ESP_LOGI(TAG, "Going to %fmm, %fsteps", h, h*steps_per_mm);
    if (h < 0) {
        ESP_LOGE(TAG, "Lift height negative: %f", h);
        return;
    }
    stepper->go_to(h*steps_per_mm, speed_stp, wait);
}

void lift::wait()
{
    stepper->wait();
}

double lift::get_position()
{
    return stepper->get_position()/steps_per_mm;
}

void lift::reset_position()
{
    stepper->reset_position();
}

void lift::reset_all()
{
    stepper->disable();
    disable_magnets();
    disable_suction();
}

void lift::calibrate(IHM *ihm)
{
    UBaseType_t prvPriority = uxTaskPriorityGet(NULL);
    vTaskPrioritySet(NULL, PRIORITY_CAL_LIFT);

    ihm->writeMsg((String("c l") + tag + " D V").c_str());
    //driver->rms_current(M_CHARIOT_CAL_CURRENT_MA);
    enable_motor();
    ESP_LOGI(TAG, "Calibration started");

    bool goingDown = false;
    while (1) {
        ESP_LOGI(TAG, "steps_per_mm: %d", steps_per_mm);
        uint8_t btns = ihm->getButtons();
        ESP_LOGI(TAG, "Cal: btns pressed: %d", btns);
        if (btns & (uint8_t)(1 << (8 - 1))) {
            vTaskDelay(pdMS_TO_TICKS(50)); // debounce delay
            if (!(ihm->getButtons() & (uint8_t)(1 << (8 - 1)))) continue;
            if (goingDown) stop_motor();
            ESP_LOGI(TAG, "Calibration accepted");
            ihm->writeMsg("accepted");
            vTaskDelay(pdMS_TO_TICKS(100));
            //driver->rms_current(M_CHARIOT_CURRENT_MA);
            reset_position();
            ESP_LOGI(TAG, "Calibration saved");
            go_to(10);
            vTaskDelay(pdMS_TO_TICKS(500));
            ihm->writeMsg("");
            vTaskPrioritySet(NULL, prvPriority);
            ESP_LOGI(TAG, "Ended calibration");
            return;
        } else if (btns & (uint8_t)(1 << (6 - 1))) {
            if (!goingDown) {
                stepper->step_speed(-(M_CHARIOT_CAL_SPEED*steps_per_mm));
                goingDown = true;
                ESP_LOGI(TAG, "Cal: Started going down");
            }
        } else if (goingDown) {
                stop_motor();
                goingDown = false;
                ESP_LOGI(TAG, "Cal: Stopped going down");
        }
        
        ESP_LOGI(TAG, "Cal: Started waiting");
        vTaskDelay(pdMS_TO_TICKS(50));
        ESP_LOGI(TAG, "Cal: Done waiting");
    }
}

void lift::stop_motor()
{
    stepper->stop();
}
