#include "IHM.h"

static const char *TAG = "IHM";

IHM::IHM(uint8_t clk_pin, uint8_t dio_pin, uint8_t stb_pin) : TM1638plus(stb_pin, clk_pin, dio_pin, true)
{
    _access_mutex = xSemaphoreCreateMutex();
    xSemaphoreTake(_access_mutex, portMAX_DELAY);
    TM1638plus::displayBegin();
    xSemaphoreGive(_access_mutex);
    reset();
    xSemaphoreTake(_access_mutex, portMAX_DELAY);
    TM1638plus::brightness(1);
    xSemaphoreGive(_access_mutex);
}

void IHM::write_msg(const char *msg)
{
    xSemaphoreTake(_access_mutex, portMAX_DELAY);
    TM1638plus::displayText(msg);
    xSemaphoreGive(_access_mutex);
}

void IHM::show_score(unsigned long score)
{
    xSemaphoreTake(_access_mutex, portMAX_DELAY);
    TM1638plus::displayIntNum(score, false, TMAlignTextRight);
    xSemaphoreGive(_access_mutex);
}

void IHM::show_progress(uint8_t progress)
{
    xSemaphoreTake(_access_mutex, portMAX_DELAY);
    TM1638plus::setLEDs(((uint16_t)((1 << (progress / 31)) - 1)) << 8);
    xSemaphoreGive(_access_mutex);
}

void IHM::show_progress_live(int start_time)
{
    _progress_start_time = start_time;
    ESP_LOGI(TAG, "Starting Live progress");
    xTaskCreate(showProgressTaskEntryPoint, "show_progress", 2048 * 4, this, 1, &_show_progress_task_handle);
}

void IHM::show_progress_stop()
{
    if (_show_progress_task_handle != nullptr)
    {
        ESP_LOGI(TAG, "Stopping Live progress");
        xTaskNotifyGive(_show_progress_task_handle);
        _show_progress_task_handle = nullptr;
        xSemaphoreTake(_access_mutex, portMAX_DELAY);
        TM1638plus::setLEDs(0x0000);
        xSemaphoreGive(_access_mutex);
    }
}

bool IHM::get_button(uint8_t button)
{
    xSemaphoreTake(_access_mutex, portMAX_DELAY);
    uint8_t btns = readButtons();
    xSemaphoreGive(_access_mutex);
    return (btns >> button) & 0x01;
}

uint8_t IHM::get_buttons()
{
    xSemaphoreTake(_access_mutex, portMAX_DELAY);
    uint8_t btns = TM1638plus::readButtons();
    xSemaphoreGive(_access_mutex);
    return btns;
}

void IHM::wait_button(uint8_t button)
{
    bool btn = false;
    while (!btn)
    {
        while (!btn)
        {
            vTaskDelay(pdMS_TO_TICKS(_POLL_DELAY));
            btn = get_button(button);
        }
        vTaskDelay(pdMS_TO_TICKS(_DEBOUNCE_DELAY));
        btn = get_button(button);
    }
}

uint8_t IHM::wait_get_buttons()
{
    int8_t btns = 0;
    while (!btns)
    {
        while (!btns)
        {
            vTaskDelay(pdMS_TO_TICKS(_POLL_DELAY));
            btns = get_buttons();
        }
        vTaskDelay(pdMS_TO_TICKS(_DEBOUNCE_DELAY));
        btns = get_buttons();
    }
    return btns;
}

bool IHM::wait_choose_side()
{
    return wait_two_choice("side b y", 6, 8);
}

bool IHM::wait_choose_camp()
{
    return wait_two_choice("camp 1 2", 6, 8);
}

void IHM::showProgressTaskEntryPoint(void *pvParameters)
{
    IHM *self = static_cast<IHM *>(pvParameters);
    self->show_progress_task();
}

void IHM::show_progress_task()
{
    ESP_LOGI(TAG, "Started Live progress");
    while (millis() - _progress_start_time < 100000)
    {
        vTaskDelay(pdMS_TO_TICKS(2000));
        ESP_LOGI(TAG, "Live progress update");
        if (ulTaskNotifyTake(pdTRUE, 0) > 0)
        {
            ESP_LOGI(TAG, "Live progress stopped");
            vTaskDelete(NULL);
        }
        // show_score((millis() - _progress_start_time));
        show_progress((millis() - _progress_start_time) * 255 / 100000);
    }
    while (1)
    {
        static uint8_t leds = 0;
        vTaskDelay(pdMS_TO_TICKS(500));
        if (ulTaskNotifyTake(pdTRUE, 0) > 0)
        {
            ESP_LOGI(TAG, "Live progress stopped");
            vTaskDelete(NULL);
        }
        xSemaphoreTake(_access_mutex, portMAX_DELAY);
        TM1638plus::setLEDs(((uint16_t)leds) << 8);
        xSemaphoreGive(_access_mutex);
        leds = leds ^ 0b11111111;
    }
}

void IHM::clear_digit(u_int8_t i)
{
    xSemaphoreTake(_access_mutex, portMAX_DELAY);
    TM1638plus::display7Seg(i, 0b00000000);
    xSemaphoreGive(_access_mutex);
}

void IHM::reset()
{
    xSemaphoreTake(_access_mutex, portMAX_DELAY);
    TM1638plus::reset();
    xSemaphoreGive(_access_mutex);
    ESP_LOGI(TAG, "Resetted");
}

void IHM::set_LED(uint8_t position, uint8_t value)
{
    xSemaphoreTake(_access_mutex, portMAX_DELAY);
    TM1638plus::setLED(position, value);
    xSemaphoreGive(_access_mutex);
}

bool IHM::wait_two_choice(const char *msg, uint8_t choice1, uint8_t choice2)
{
    uint8_t btns;
    reset();
    ESP_LOGI(TAG, "Started choice '%s'", msg);
    write_msg(msg);
    do
    {
        btns = wait_get_buttons();
    } while (!(btns & ((1 << (choice1 - 1)) | (1 << (choice2 - 1)))));
    bool r = btns & (1 << (choice2 - 1));
    ESP_LOGI(TAG, "Chose %d", r);
    if (r)
    {
        xSemaphoreTake(_access_mutex, portMAX_DELAY);
        TM1638plus::setLED(choice2, 1);
        xSemaphoreGive(_access_mutex);
        clear_digit(7 - (choice1 - 1));
    }
    else
    {
        xSemaphoreTake(_access_mutex, portMAX_DELAY);
        TM1638plus::setLED(choice1, 1);
        xSemaphoreGive(_access_mutex);
        clear_digit(7 - (choice2 - 1));
    }

    ESP_LOGI(TAG, "Waiting %d ms", _CHOICE_DELAY);
    // vTaskDelay(pdMS_TO_TICKS(_CHOICE_DELAY));
    ESP_LOGI(TAG, "Ended choice '%s'", msg);
    return r;
}
