#pragma once

#include "TM1638plus.h"

class IHM : protected TM1638plus
{
private:
    static constexpr int _POLL_DELAY = 100;    // delay in ms
    static constexpr int _DEBOUNCE_DELAY = 50; // delay in ms
    static constexpr int _CHOICE_DELAY = 1000; // delay in ms

public:
    IHM(uint8_t clk_pin, uint8_t dio_pin, uint8_t stb_pin);
    void write_msg(const char *msg);
    void show_score(unsigned long score);
    void show_progress(uint8_t progress); // number between 0 and 255
    void show_progress_live(int start_time);
    void show_progress_stop();
    bool get_button(uint8_t button);
    uint8_t get_buttons();
    void wait_button(uint8_t button);
    uint8_t wait_get_buttons();
    bool wait_choose_side();
    bool wait_choose_camp();
    void reset();
    void set_LED(uint8_t position, uint8_t value);

private:
    SemaphoreHandle_t _access_mutex;
    static void showProgressTaskEntryPoint(void *pvParameters);
    TaskHandle_t _show_progress_task_handle = nullptr;
    void show_progress_task();
    int _progress_start_time = 0;
    void clear_digit(u_int8_t i);
    bool wait_two_choice(const char *msg, uint8_t choice1, uint8_t choice2);
};