#include "ledc_stepper.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "soc/pcnt_periph.h"
#include "soc/gpio_sig_map.h"
#include <rom/gpio.h>

#include "config.h"

static const char *TAG = "ledc_stepper";

// Why the hell, does espressif think, that the unit and channel id are not
// needed ? Without unit/channel ID, the needed parameter for
// gpio_matrix_in/gpio_iomux_in cannot be derived.
//
// Here we declare the private pcnt_chan_t structure, which is not save.
struct pcnt_unit_t
{
    /*pcnt_group_t*/ void *group;
    portMUX_TYPE spinlock;
    int unit_id;
    // remainder of struct not needed
};
struct pcnt_chan_t
{
    pcnt_unit_t *unit;
    int channel_id;
    // remainder of struct not needed
};

uint8_t ledc_stepper::next_timer_channel = 0;

#define HALF_DUTY 128

bool IRAM_ATTR pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    ledc_stepper *stepper = static_cast<ledc_stepper *>(user_ctx);
    vTaskNotifyGiveFromISR(stepper->xTaskToNotify, NULL);
    return false;
}

void stopTask(void *pvParameter)
{
    ledc_stepper *stepper = static_cast<ledc_stepper *>(pvParameter);
    while (1)
    {
        xTaskNotifyWait(0x00, ULONG_MAX, NULL, portMAX_DELAY);
        stepper->stop();
    }
}

ledc_stepper::ledc_stepper(uint8_t step_pin, uint8_t dir_pin, uint8_t en_pin)
{
    _step_pin = step_pin;
    _dir_pin = (gpio_num_t)dir_pin;
    _en_pin = (gpio_num_t)en_pin;

    if (next_timer_channel >= LEDC_TIMER_MAX)
        ESP_LOGE(TAG, "No more timers available");
    if (next_timer_channel >= LEDC_CHANNEL_MAX)
        ESP_LOGE(TAG, "No more channels available");

    _timer = (ledc_timer_t)next_timer_channel;
    _channel = (ledc_channel_t)next_timer_channel;
    ++next_timer_channel;

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = _timer,
        .freq_hz = 100,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .gpio_num = _step_pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = _channel,
        .timer_sel = _timer,
        .duty = 0,
        .hpoint = 0};
    ledc_channel_config(&ledc_channel);

    ledc_fade_func_install(0);

    pcnt_unit_config_t unit_config = {
        .low_limit = -32000,
        .high_limit = 32000};
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = _step_pin,
        .level_gpio_num = _dir_pin,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan, PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_KEEP));

    pcnt_event_callbacks_t cbs = {
        .on_reach = pcnt_on_reach,
    };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, this));

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

    int unit_id = pcnt_unit->unit_id;
    int channel_id = pcnt_chan->channel_id;
    // int signal = pcnt_periph_signals.groups[0]
    //                .units[unit_id]
    //                .channels[channel_id]
    //                .pulse_sig;

    // gpio_matrix_in(step_pin, signal, 0);
    // gpio_iomux_in(step_pin, signal);

    gpio_reset_pin(_dir_pin);
    gpio_set_direction(_dir_pin, GPIO_MODE_OUTPUT);

    int control = pcnt_periph_signals.groups[0]
                      .units[unit_id]
                      .channels[channel_id]
                      .control_sig;
    gpio_iomux_out(dir_pin, 0x100, false);
    gpio_matrix_in(dir_pin, control, 0);
    gpio_iomux_in(dir_pin, control);

    gpio_set_direction((gpio_num_t)_step_pin, GPIO_MODE_INPUT_OUTPUT);
    gpio_matrix_out(_step_pin, LEDC_LS_SIG_OUT0_IDX + _channel, 0, 0);

    gpio_reset_pin(_en_pin);
    gpio_set_direction(_en_pin, GPIO_MODE_OUTPUT);

    disable();

    xTaskCreate(stopTask, "ledc_stepperStopTask", 1024U, (void *)this, PRIORITY_LIFT_STEPPERS, &xStopTask);
}

void ledc_stepper::step(int32_t position, uint32_t speed, bool wait, bool reset)
{
    UBaseType_t prvPriority = uxTaskPriorityGet(NULL);
    vTaskPrioritySet(NULL, PRIORITY_LIFT_STEPPERS);

    if (!enabled)
        ESP_LOGE(TAG, "Stepper not enabled !");
    if (running) {
        if ((!reset) && pcnt_watch_point) {
            position += pcnt_watch_point;
        }
        stop(false);
    }
    gpio_set_level(_dir_pin, (position < 0) ? 1 : 0);

    pcnt_unit_add_watch_point(pcnt_unit, position);
    pcnt_watch_point = position;
    pcnt_unit_clear_count(pcnt_unit);

    ledc_set_freq(LEDC_LOW_SPEED_MODE, _timer, speed);

    xTaskToNotify = wait ? xTaskGetCurrentTaskHandle() : xStopTask;
    running = true;
    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, _channel, HALF_DUTY, 0);
    ESP_LOGI(TAG, "activated ledc output");
    if (wait)
    {
        ESP_LOGI(TAG, "waiting");
        xTaskNotifyWait(0x00, ULONG_MAX, NULL, portMAX_DELAY);
        ESP_LOGI(TAG, "stopped waiting");
        stop();
        ESP_LOGI(TAG, "stopped ledc output");
    }
    vTaskPrioritySet(NULL, prvPriority);
}

void ledc_stepper::go_to(int32_t position, uint32_t speed, bool wait)
{
    UBaseType_t prvPriority = uxTaskPriorityGet(NULL);
    vTaskPrioritySet(NULL, PRIORITY_LIFT_STEPPERS);

    if (!enabled)
        ESP_LOGE(TAG, "Stepper not enabled !");
    
    if (running) stop(false);

    step(this->position, speed, wait, true);

    vTaskPrioritySet(NULL, prvPriority);
}

void ledc_stepper::step_speed(int32_t speed)
{
    UBaseType_t prvPriority = uxTaskPriorityGet(NULL);
    vTaskPrioritySet(NULL, PRIORITY_LIFT_STEPPERS);

    if (!enabled)
        ESP_LOGE(TAG, "Stepper not enabled !");
    if (running)
        stop(false);
    running = true;
    gpio_set_level(_dir_pin, (speed < 0) ? 1 : 0);
    ESP_ERROR_CHECK(ledc_set_freq(LEDC_LOW_SPEED_MODE, _timer, abs(speed)));
    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, _channel, HALF_DUTY, 0);

    vTaskPrioritySet(NULL, prvPriority);
}

void ledc_stepper::stop(bool stop_waiting)
{
    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, _channel, 0, 0);
    running = false;
    int count;
    pcnt_unit_get_count(pcnt_unit, &count);
    pcnt_unit_clear_count(pcnt_unit);
    position += count;
    if (pcnt_watch_point) {
        pcnt_unit_remove_watch_point(pcnt_unit, pcnt_watch_point);
        pcnt_watch_point = 0;
    }
    if (stop_waiting && waiting) {
        xTaskNotifyGive(xWaitTaskHandle);
    }
}

void ledc_stepper::wait() {
    if (running) {
        xWaitTaskHandle =  xTaskGetCurrentTaskHandle();
        waiting = true;
        xTaskNotifyWait(0x00, ULONG_MAX, NULL, portMAX_DELAY);
        waiting = false;
    }
}

void ledc_stepper::disable()
{
    if (enabled)
        gpio_set_level(_en_pin, 1);
    enabled = false;
}

void ledc_stepper::enable()
{
    if (!enabled)
        gpio_set_level(_en_pin, 0);
    enabled = true;
}

int ledc_stepper::get_position()
{
    int count = 0;
    if (running)
        pcnt_unit_get_count(pcnt_unit, &count);
    return position + count;
}

void ledc_stepper::reset_position()
{
    position = 0;
}
