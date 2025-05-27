#pragma once

#include <stdint.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "freertos/FreeRTOS.h"

class ledc_stepper
{
    friend bool IRAM_ATTR pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx);
    friend void stopTask(void * pvParameter );
public:
    ledc_stepper(uint8_t step_pin, uint8_t dir_pin, uint8_t en_pin);
    void step(int32_t position, uint32_t speed, bool wait = true, bool reset = false);
    void go_to(int32_t position, uint32_t speed, bool wait = true);
    void step_speed(int32_t speed);
    void stop(bool stop_waiting = true);
    void disable();
    void enable();
    int get_position();
    void reset_position();
    void wait();
    
private:
    TaskHandle_t xTaskToNotify = NULL;
    TaskHandle_t xStopTask;
    TaskHandle_t xWaitTaskHandle = NULL;
    bool waiting = false;
    static uint8_t next_timer_channel;
    uint8_t _step_pin;
    gpio_num_t _dir_pin;
    gpio_num_t _en_pin;
    uint32_t _pos{0};
    ledc_timer_t _timer;
    ledc_channel_t _channel;
    bool enabled = false;
    bool running = false;
    int position = 0;
    int pcnt_watch_point = 0;

    pcnt_unit_handle_t pcnt_unit = NULL;
    pcnt_channel_handle_t pcnt_chan = NULL;
};
