#include "ihm.h"

static const char *TAG = "IHM";


IHM::IHM(uint8_t clk_pin, uint8_t dio_pin, uint8_t stb_pin) : TM1638plus(stb_pin, clk_pin, dio_pin, true)
{
    tm1638_mutex = xSemaphoreCreateMutex();
    xSemaphoreTake(tm1638_mutex, portMAX_DELAY);
    TM1638plus::displayBegin();
    xSemaphoreGive(tm1638_mutex);
    reset();
    xSemaphoreTake(tm1638_mutex, portMAX_DELAY);
    TM1638plus::brightness(1);
    xSemaphoreGive(tm1638_mutex);
}

void IHM::writeMsg(const char *msg)
{
    xSemaphoreTake(tm1638_mutex, portMAX_DELAY);
    TM1638plus::displayText(msg);
    xSemaphoreGive(tm1638_mutex);
}

void IHM::showScore(unsigned long score)
{
    xSemaphoreTake(tm1638_mutex, portMAX_DELAY);
    TM1638plus::displayIntNum(score, false, TMAlignTextRight);
    xSemaphoreGive(tm1638_mutex);
}

void IHM::showProgress(uint_fast8_t progress)
{
    xSemaphoreTake(tm1638_mutex, portMAX_DELAY);
    TM1638plus::setLEDs(((uint16_t)((1 << (progress / 31)) - 1)) << 8);
    xSemaphoreGive(tm1638_mutex);
}

void IHM::showProgressLive(int start)
{
    showProgressStart = start;
    ESP_LOGI(TAG, "Starting Live progress");
    xTaskCreate(showProgressTaskEntryPoint, "showProgress", 2048*4, this, 1, &showProgressTaskHandle);
}

void IHM::showProgressStop()
{
    if (showProgressTaskHandle != nullptr)
    {
        ESP_LOGI(TAG, "Stopping Live progress");
        xTaskNotifyGive(showProgressTaskHandle);
        showProgressTaskHandle = nullptr;
        xSemaphoreTake(tm1638_mutex, portMAX_DELAY);
        TM1638plus::setLEDs(0x0000);
        xSemaphoreGive(tm1638_mutex);
    }
}

bool IHM::getButton(uint8_t s)
{
    xSemaphoreTake(tm1638_mutex, portMAX_DELAY);
    uint8_t btns = readButtons();
    xSemaphoreGive(tm1638_mutex);
    return (btns >> s) & 0x01;
}

uint8_t IHM::getButtons()
{
    xSemaphoreTake(tm1638_mutex, portMAX_DELAY);
    uint8_t btns = TM1638plus::readButtons();
    xSemaphoreGive(tm1638_mutex);
    return btns;
}

void IHM::wGetButton(uint8_t s)
{
    bool btn = false;
    while (!btn)
    {
        while (!btn)
        {
            vTaskDelay(pdMS_TO_TICKS(poll_delay));
            btn = getButton(s);
        }
        vTaskDelay(pdMS_TO_TICKS(debounce_delay));
        btn = getButton(s);
    }
}

uint8_t IHM::wGetButtons()
{
    int8_t btns = 0;
    while (!btns)
    {
        while (!btns)
        {
            vTaskDelay(pdMS_TO_TICKS(poll_delay));
            xSemaphoreTake(tm1638_mutex, portMAX_DELAY);
            btns = TM1638plus::readButtons();
            xSemaphoreGive(tm1638_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(debounce_delay));
        xSemaphoreTake(tm1638_mutex, portMAX_DELAY);
        btns = TM1638plus::readButtons();
        xSemaphoreGive(tm1638_mutex);
    }
    return btns;
}

bool IHM::wChooseSide()
{
    return wTwoChoice("side 1 2", 6, 8);
}

bool IHM::wChooseCamp()
{
    return wTwoChoice("camp 1 2", 6, 8);
}

void IHM::showProgressTaskEntryPoint(void *pvParameters)
{
    IHM *self = static_cast<IHM *>(pvParameters);
    self->showProgressTask();
}

void IHM::showProgressTask()
{
    ESP_LOGI(TAG, "Started Live progress");
    while (millis() - showProgressStart < 100000)
    {
        vTaskDelay(pdMS_TO_TICKS(2000));
        ESP_LOGI(TAG, "Live progress update");
        if (ulTaskNotifyTake(pdTRUE, 0) > 0) {
            ESP_LOGI(TAG, "Live progress stopped");
            vTaskDelete(NULL);
        }
        //showScore((millis() - showProgressStart));
        showProgress((millis() - showProgressStart) * 255 / 100000);
    }
    while (1)
    {
        static uint8_t leds = 0;
        vTaskDelay(pdMS_TO_TICKS(500));
        if (ulTaskNotifyTake(pdTRUE, 0) > 0) {
            ESP_LOGI(TAG, "Live progress stopped");
            vTaskDelete(NULL);
        }
        xSemaphoreTake(tm1638_mutex, portMAX_DELAY);
        TM1638plus::setLEDs(((uint16_t)leds) << 8);
        xSemaphoreGive(tm1638_mutex);
        leds = leds ^ 0b11111111;
    }
}

void IHM::clearDig(u_int8_t i)
{
    xSemaphoreTake(tm1638_mutex, portMAX_DELAY);
    TM1638plus::display7Seg(i, 0b00000000);
    xSemaphoreGive(tm1638_mutex);
}

void IHM::reset()
{
    xSemaphoreTake(tm1638_mutex, portMAX_DELAY);
    TM1638plus::reset();
    xSemaphoreGive(tm1638_mutex);
    ESP_LOGI(TAG, "Resetted");
}

void IHM::setLED(uint8_t position, uint8_t value)
{
    xSemaphoreTake(tm1638_mutex, portMAX_DELAY);
    TM1638plus::setLED(position, value);
    xSemaphoreGive(tm1638_mutex);
}

bool IHM::wTwoChoice(const char *msg, uint8_t choice1, uint8_t choice2)
{
    uint8_t btns;
    reset();
    ESP_LOGI(TAG, "Started choice '%s'", msg);
    writeMsg(msg);
    do
    {
        btns = wGetButtons();
    } while (!(btns & ((1 << (choice1 - 1)) | (1 << (choice2 - 1)))));
    bool r = btns & (1 << (choice2 - 1));
    ESP_LOGI(TAG, "Chose %d", r);
    if (r)
    {
        xSemaphoreTake(tm1638_mutex, portMAX_DELAY);
        TM1638plus::setLED(choice2, 1);
        xSemaphoreGive(tm1638_mutex);
        clearDig(7 - (choice1 - 1));
    }
    else
    {
        xSemaphoreTake(tm1638_mutex, portMAX_DELAY);
        TM1638plus::setLED(choice1, 1);
        xSemaphoreGive(tm1638_mutex);
        clearDig(7 - (choice2 - 1));
    }
    
    ESP_LOGI(TAG, "Waiting %d ms", choice_delay);
    //vTaskDelay(pdMS_TO_TICKS(choice_delay));
    ESP_LOGI(TAG, "Ended choice '%s'", msg);
    return r;
}
