#include "Ld19p.h"

#include "esp_log.h"
#include "driver/ledc.h"
#include "config.h"

static const char *TAG = "Ld19p";

/*
I was not able to control the speed of the motor, the ld19p won't switch to pwm motor control despite sending it a 30khz pwm signal for 150ms
*/

#define HEADER 0x54
#define VERLEN 0x2C
#define POINT_PER_PACKET 12
#define PACKET_SIZE 47
#define MIN_INTERESTED_SIZE ((551 * PACKET_SIZE) / POINT_PER_PACKET) // There are 500 points per revolutions at 10Hz, and each packet of 47 bytes contains 12 points
#define UART_BUFFER MIN_INTERESTED_SIZE * 2

void uartPartialFlush(void *pvParameter)
{
    Ld19p *ld19p = static_cast<Ld19p *>(pvParameter);
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(80));

        ld19p->partialFlush();
        // ESP_LOGI(TAG, "Flush Task Watermark %d", uxTaskGetStackHighWaterMark(NULL));
    }
}

Ld19p::Ld19p(uart_port_t uart_num, uint8_t rx_pin)
{
    _uart_num = uart_num;
    uart_config_t uart_config = {
        .baud_rate = 230400,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0};
    ESP_ERROR_CHECK(uart_param_config(_uart_num, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(_uart_num, UART_PIN_NO_CHANGE, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)); // using UART_PIN_NO_CHANGE for pins not used

    ESP_ERROR_CHECK(uart_driver_install(_uart_num, UART_BUFFER,
                                        0, 10, NULL, 0));

    xTaskCreate(uartPartialFlush, "ld19p_partialFlushTask", 8192 * 4, (void *)this, PRIORITY_LD19P, &_partial_flush_handle);

    ESP_LOGI(TAG, "Ld19p is ready on GPIO%d, UART%d", rx_pin, _uart_num);
}

void Ld19p::partialFlush()
{
    size_t size;
    uart_get_buffered_data_len(_uart_num, &size);
    int flush_size = size - (size_t)MIN_INTERESTED_SIZE;
    // ESP_LOGI(TAG, "Flushing: %d from %d", flush_size, size);
    if (flush_size > 0)
    {
        uint8_t *tmp = new uint8_t[flush_size];
        uart_read_bytes(_uart_num, tmp, flush_size, pdMS_TO_TICKS(1));
        delete[] tmp;
    }
    uart_get_buffered_data_len(_uart_num, &size);
    // ESP_LOGI(TAG, "Flushing: remaining %d", size);
}

uint16_t Ld19p::angleStep(uint16_t start_angle, uint16_t end_angle)
{
    if (start_angle <= end_angle)
    {
        return (end_angle - start_angle) / (POINT_PER_PACKET - 1);
    }
    else
    {
        return (36000 + end_angle - start_angle) / (POINT_PER_PACKET - 1);
    }
}

int Ld19p::getFullTour(LidarPoint_t points[], int size)
{
    if (size == 0)
        return 0;

    ESP_LOGI(TAG, "Watermark:%d", uxTaskGetStackHighWaterMark(NULL));
    partialFlush();
    ESP_LOGI(TAG, "Started getFullTour");
    int pt_nb = 0;
    uint8_t data[MIN_INTERESTED_SIZE];
    ESP_LOGI(TAG, "Watermark:%d", uxTaskGetStackHighWaterMark(NULL));
    int len = uart_read_bytes(_uart_num, data, MIN_INTERESTED_SIZE, pdMS_TO_TICKS(150));
    if (len == 0)
    {
        ESP_LOGE(TAG, "Got no points from ld19p");
        return 0;
    }
    int i = 0;
    while (i + PACKET_SIZE <= len)
    {
        if (data[i] == HEADER && data[i + 1] == VERLEN)
        {
            // ESP_LOGI(TAG, "Found Header");
            int prev_d = 0;
            if (CalCRC8(&data[i + 2], 44) == data[i + 46])
            {
                // ESP_LOGI(TAG, "CRC correct");
                const LiDARFrameTypeDef *frame = reinterpret_cast<const LiDARFrameTypeDef *>(&data[i]);
                // ESP_LOGI(TAG, "Header: %d, verlen: %d", frame->header, frame->ver_len);
                uint16_t step = angleStep(frame->start_angle, frame->end_angle);
                for (int y = 0; y < POINT_PER_PACKET; ++y)
                {
                    points[pt_nb].angle = (frame->start_angle + step * y) % 36000;
                    points[pt_nb].distance = frame->point[y].distance;
                    int next_d = (points[pt_nb].angle + 36000 - points[0].angle) % 36000;
                    // ESP_LOGI(TAG, "pt_num: %d, pt_angle: %d, pt_d: %d, next_d: %d", pt_nb, points[pt_nb].angle, points[pt_nb].distance, next_d);
                    if (next_d < prev_d)
                        return pt_nb;
                    ++pt_nb;
                    if (pt_nb >= size)
                        return size;
                    prev_d = next_d;
                }
                i += PACKET_SIZE;
            }
            else
                ESP_LOGE(TAG, "CRC incorrect");
        }
        else
            ++i;
    }

    return pt_nb;
}

uint8_t Ld19p::CalCRC8(uint8_t *p, uint8_t len)
{
    uint8_t crc = 0xD8; // precalculated the CRC of header
    for (uint16_t i = 0; i < len; ++i)
    {
        crc = CRC_TABLE[(crc ^ *p++) & 0xff];
    }
    return crc;
}
