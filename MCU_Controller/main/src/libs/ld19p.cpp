#include "ld19p.h"

#include "esp_log.h"
#include "driver/ledc.h"

static const char *TAG = "ld19p";

/*
I was not able to control the speed of the motor, the lidar won't switch to pwm motor control despite sending it a 30khz pwm signal for 150ms
*/

#define HEADER 0x54

ld19p::ld19p(uart_port_t uart_num, uint8_t rx_pin)
{
    uart_config_t uart_config = {
        .baud_rate = 230400,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(uart_num, UART_PIN_NO_CHANGE, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)); // using UART_PIN_NO_CHANGE for pins not used

    // Setup UART buffered IO with event queue
    const int uart_buffer_size = ((500 + 10)* 47)/12; // There are 500 points per revolutions at 10Hz, and each packet of 47 bytes contains 12 points
    QueueHandle_t uart_queue;
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size,
                                        0, 10, &uart_queue, 0));

    ESP_LOGI(TAG, "LiDAR is ready on GPIO%d", rx_pin);
}

// ld19p::ld19p(uart_port_t uart_num, uint8_t rx_pin, uint8_t pwm_pin)
// {
//     motor_control = true;

//     ledc_timer_config_t ledc_timer = {
//         .speed_mode       = LEDC_LOW_SPEED_MODE,
//         .duty_resolution  = LEDC_TIMER_10_BIT,
//         .timer_num        = LEDC_TIMER_0,
//         .freq_hz          = 30000,
//         .clk_cfg          = LEDC_AUTO_CLK
//     };
//     ledc_timer_config(&ledc_timer);

//     ledc_channel_config_t ledc_channel = {
//         .gpio_num   = pwm_pin,
//         .speed_mode = LEDC_LOW_SPEED_MODE,
//         .channel    = LEDC_CHANNEL_0,
//         .timer_sel  = LEDC_TIMER_0,
//         .duty       = 512, // starting duty cycle of 50% to trigger motor controls
//         .hpoint     = 0
//     };
//     ledc_channel_config(&ledc_channel);

//     vTaskDelay(pdMS_TO_TICKS(500));

//     stop();

//     ESP_LOGI(TAG, "LiDAR pwm controls initialized on pin %d", pwm_pin);

//     ld19p(uart_num, rx_pin);
// }

// void ld19p::set_speed_freq(uint8_t speed_freq)
// {
//     if (!check_motor_control()) return;
//     this->speed_freq = speed_freq;
//     ESP_LOGI(TAG, "LiDAR speed set to %d freq", speed_freq);
// }

// uint8_t ld19p::get_speed_freq()
// {
//     return speed_freq;
// }

// void ld19p::set_duty_cycle(uint32_t duty_cycle)
// {
//     ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_cycle);
//     ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
// }

// bool ld19p::check_motor_control()
// {
//     if (!motor_control) {
//         ESP_LOGE(TAG, "LiDAR motor controls not initialized !");
//         return false;
//     }
//     return true;
// }

// void ld19p::start()
// {
//     if (!check_motor_control()) return;
//     set_duty_cycle(duty_cycle);
//     ESP_LOGI(TAG, "LiDAR motor started");
// }

// void ld19p::stop()
// {
//     if (!check_motor_control()) return;
//     set_duty_cycle(0);
//     ESP_LOGI(TAG, "LiDAR motor stopped");
// }

uint8_t ld19p::CalCRC8(uint8_t *p, uint8_t len)
{
    uint8_t crc = 0;
    uint16_t i;
    for (i = 0; i < len; i++)
    {
        crc = CrcTable[(crc ^ *p++) & 0xff];
    }
    return crc;
}