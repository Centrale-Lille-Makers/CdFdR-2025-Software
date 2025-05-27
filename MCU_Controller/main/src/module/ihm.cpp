#include "ihm.h"

static const char *TAG = "IHM";


IHM::IHM(uint8_t clk_pin, uint8_t dio_pin, uint8_t stb_pin) : TM1638(clk_pin, dio_pin, stb_pin)
{
    tm1638_mutex = xSemaphoreCreateMutex();
    reset();
    TM1638::displaySetBrightness(PULSE1_16);
}

void IHM::writeMsg(String msg)
{
    for (u_int8_t i = 0; i <= 7; ++i)
    {
        if (i >= msg.length() || msg[i] == ' ')
        {
            clearDig(7 - i);
            continue;
        }

        char c = tolower(msg[i]);

        xSemaphoreTake(tm1638_mutex, portMAX_DELAY);
        if ('a' <= c && c <= 'z')
            TM1638::displayDig(7 - i, _letters[c - 97]);
        else
            TM1638::displayVal(7 - i, msg[i] - 48);
        xSemaphoreGive(tm1638_mutex);
    }
}

void IHM::showScore(uint_fast16_t score)
{
    for (int i = 0; i <= 7; ++i)
    {
        if (score == 0) clearDig(i);
        else {
            xSemaphoreTake(tm1638_mutex, portMAX_DELAY);
            TM1638::displayVal(i, score % 10);
            xSemaphoreGive(tm1638_mutex);
        }
        score /= 10;
    }
}

void IHM::showProgress(uint_fast8_t progress)
{
    xSemaphoreTake(tm1638_mutex, portMAX_DELAY);
    TM1638::writeLeds((1 << progress / 31) - 1);
    xSemaphoreGive(tm1638_mutex);
}

void IHM::showProgressLive(int start)
{
    showProgressStart = start;
    ESP_LOGI(TAG, "Starting Live progress");
    xTaskCreate(showProgressTaskEntryPoint, "showProgress", 2048, this, 1, &showProgressTaskHandle);
}

void IHM::showProgressStop()
{
    if (showProgressTaskHandle != nullptr)
    {
        ESP_LOGI(TAG, "Stopping Live progress");
        xTaskNotifyGive(showProgressTaskHandle);
        showProgressTaskHandle = nullptr;
        xSemaphoreTake(tm1638_mutex, portMAX_DELAY);
        TM1638::writeLeds(0);
        xSemaphoreGive(tm1638_mutex);
    }
}

void IHM::wGetButton(uint8_t s)
{
    bool btn = false;
    while (!btn)
    {
        while (!btn)
        {
            vTaskDelay(pdMS_TO_TICKS(poll_delay));
            xSemaphoreTake(tm1638_mutex, portMAX_DELAY);
            btn = TM1638::getButton(intToButton(s));
            xSemaphoreGive(tm1638_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(debounce_delay));
        xSemaphoreTake(tm1638_mutex, portMAX_DELAY);
        btn = TM1638::getButton(intToButton(s));
        xSemaphoreGive(tm1638_mutex);
    }
}

uint8_t IHM::wGetButtons()
{
    int8_t btn = 0;
    while (!btn)
    {
        while (!btn)
        {
            vTaskDelay(pdMS_TO_TICKS(poll_delay));
            xSemaphoreTake(tm1638_mutex, portMAX_DELAY);
            btn = TM1638::getButtons();
            xSemaphoreGive(tm1638_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(debounce_delay));
        xSemaphoreTake(tm1638_mutex, portMAX_DELAY);
        btn = TM1638::getButtons();
        xSemaphoreGive(tm1638_mutex);
    }
    return btn;
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
        TM1638::writeLeds(leds);
        xSemaphoreGive(tm1638_mutex);
        leds = leds ^ 0b11111111;
    }
}

button_t IHM::intToButton(u_int8_t i)
{
    switch (i)
    {
    case 0:
        return S1;
    case 1:
        return S2;
    case 2:
        return S3;
    case 3:
        return S4;
    case 4:
        return S5;
    case 5:
        return S6;
    case 6:
        return S7;
    case 7:
        return S8;
    default:
        return static_cast<button_t>(-1);
    }
}

void IHM::clearDig(u_int8_t i)
{
    xSemaphoreTake(tm1638_mutex, portMAX_DELAY);
    TM1638::displayDig(i, 0b00000000);
    xSemaphoreGive(tm1638_mutex);
}

void IHM::reset()
{
    xSemaphoreTake(tm1638_mutex, portMAX_DELAY);
    TM1638::reset();
    xSemaphoreGive(tm1638_mutex);
    ESP_LOGI(TAG, "Resetted");
}

bool IHM::wTwoChoice(String msg, uint8_t choice1, uint8_t choice2)
{
    uint8_t btns;
    reset();
    ESP_LOGI(TAG, "Started choice '%s'", msg.c_str());
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
        TM1638::writeLed(choice2, ON);
        xSemaphoreGive(tm1638_mutex);
        clearDig(7 - (choice1 - 1));
    }
    else
    {
        xSemaphoreTake(tm1638_mutex, portMAX_DELAY);
        TM1638::writeLed(choice1, ON);
        xSemaphoreGive(tm1638_mutex);
        clearDig(7 - (choice2 - 1));
    }
    
    ESP_LOGI(TAG, "Waiting %d ms", choice_delay);
    //vTaskDelay(pdMS_TO_TICKS(choice_delay));
    ESP_LOGI(TAG, "Ended choice '%s'", msg.c_str());
    return r;
}
