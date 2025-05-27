#pragma once

#include "libs/TM1638.h"


class IHM : protected TM1638
{
private:
    static constexpr uint_fast8_t _letters[26] = {
        0b01011111, 0b01111100, 0b01011000, 0b01011110,
        0b01111001, 0b01110001, 0b00111101, 0b01110100,
        0b00010001, 0b00001101, 0b01110101, 0b00111000,
        0b01010101, 0b01010100, 0b01011100, 0b01110011,
        0b01100111, 0b01010000, 0b00101101, 0b01111000,
        0b00011100, 0b00101010, 0b01101010, 0b00010100,
        0b01101110, 0b00011011};

    static constexpr int poll_delay = 100; // delay en ms
    static constexpr int debounce_delay = 50; // delay en ms
    static constexpr int choice_delay = 1000; // delay en ms
    SemaphoreHandle_t tm1638_mutex;

public:
    IHM(uint8_t clk_pin, uint8_t dio_pin, uint8_t stb_pin);
    void writeMsg(String msg);
    void showScore(uint_fast16_t score);
    void showProgress(uint_fast8_t progress); //number between 0 and 255
    void showProgressLive(int start);
    void showProgressStop();
    void wGetButton(uint8_t s);
    button_t wGetButton();
    uint8_t wGetButtons();
    bool wChooseSide();
    bool wChooseCamp();
    void reset();

private:
    static void showProgressTaskEntryPoint(void *pvParameters);
    TaskHandle_t showProgressTaskHandle = nullptr;
    void showProgressTask();
    int showProgressStart = 0;
    button_t intToButton(u_int8_t i);
    void clearDig(u_int8_t i);
    bool wTwoChoice(String msg, uint8_t choice1, uint8_t choice2);
};