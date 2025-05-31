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


#define PCNT_HIGH_LIMIT 32000
#define PCNT_LOW_LIMIT -32000

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
SemaphoreHandle_t ledc_stepper::pcnt_mutex= xSemaphoreCreateMutex();

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
        stepper->stop(true, true);
        ESP_LOGI(TAG, "stopTaskWatermark: %d", uxTaskGetStackHighWaterMark(NULL));
    }
}

ledc_stepper::ledc_stepper(uint8_t step_pin, uint8_t dir_pin, uint8_t en_pin, bool invert_dir_pin)
{
    _step_pin = step_pin;
    _dir_pin = (gpio_num_t)dir_pin;
    this->invert_dir_pin = invert_dir_pin;
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
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false};
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .gpio_num = _step_pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = _channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = _timer,
        .duty = 0,
        .hpoint = 0,
        .flags = {.output_invert = 1}};
    ledc_channel_config(&ledc_channel);

    ledc_fade_func_install(0);

    pcnt_unit_config_t unit_config = {
        .low_limit = PCNT_LOW_LIMIT,
        .high_limit = PCNT_HIGH_LIMIT,
        .intr_priority = 0,
        .flags = {.accum_count = 1}};
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = _step_pin,
        .level_gpio_num = _dir_pin,
        .flags = {
            .invert_edge_input = 0,
            .invert_level_input = 0,
            .virt_edge_io_level = 0,
            .virt_level_io_level = 0,
            .io_loop_back = 0,
        }};
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
    if (invert_dir_pin) ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    else ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan, PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_KEEP));

    pcnt_event_callbacks_t cbs = {
        .on_reach = pcnt_on_reach,
    };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, this));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, PCNT_LOW_LIMIT));

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

    xTaskCreate(stopTask, "ledc_stepperStopTask", 8192*4, (void *)this, PRIORITY_LIFT_STEPPERS, &xStopTask);
    xTaskToNotify = xStopTask; //default value
}

void ledc_stepper::step(int32_t position, uint32_t speed, bool wait, bool reset)
{
    if (position == 0) return;
    UBaseType_t prvPriority = uxTaskPriorityGet(NULL);
    vTaskPrioritySet(NULL, PRIORITY_LIFT_STEPPERS);

    if (!enabled)
        ESP_LOGE(TAG, "Stepper not enabled !");
    if (running) {
        if ((!reset) && pcnt_objective) {
            position += pcnt_objective;
        }
        stop(false);
    }
    gpio_set_level(_dir_pin, (position < 0) ? !invert_dir_pin : invert_dir_pin);

    pcnt_objective = position;
    if (position > 0) pcnt_watch_point = (position < PCNT_HIGH_LIMIT) ? position : PCNT_HIGH_LIMIT;
    else pcnt_watch_point = (position > PCNT_LOW_LIMIT) ? position : PCNT_LOW_LIMIT;
    if (pcnt_watch_point != PCNT_HIGH_LIMIT && pcnt_watch_point != PCNT_LOW_LIMIT) {
        xSemaphoreTake(pcnt_mutex, portMAX_DELAY);
        pcnt_unit_add_watch_point(pcnt_unit, pcnt_watch_point);
        pcnt_unit_clear_count(pcnt_unit);
        xSemaphoreGive(pcnt_mutex);
    }

    ledc_set_freq(LEDC_LOW_SPEED_MODE, _timer, speed);

    xTaskToNotify = wait ? xTaskGetCurrentTaskHandle() : xStopTask;
    running = true;
    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, _channel, HALF_DUTY, 0);
    ESP_LOGI(TAG, "activated ledc output");
    if (wait)
    {
        do {
            ESP_LOGI(TAG, "waiting");
            xTaskNotifyWait(0x00, ULONG_MAX, NULL, portMAX_DELAY);
            ESP_LOGI(TAG, "stopped waiting");
            stop();
            ESP_LOGI(TAG, "stopped ledc output");
        } while (running);
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
    step(position-this->position, speed, wait, true);

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
    last_speed = speed;
    gpio_set_level(_dir_pin, (speed < 0) ? !invert_dir_pin : invert_dir_pin);
    ESP_ERROR_CHECK(ledc_set_freq(LEDC_LOW_SPEED_MODE, _timer, abs(speed)));
    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, _channel, HALF_DUTY, 0);

    vTaskPrioritySet(NULL, prvPriority);
}

void ledc_stepper::stop(bool stop_waiting, bool from_ISR)
{
    xSemaphoreTake(pcnt_mutex, portMAX_DELAY);
    UBaseType_t prvPriority = uxTaskPriorityGet(NULL);
    vTaskPrioritySet(NULL, PRIORITY_LIFT_STOP_ISR);

    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, _channel, 0, 0);
    running = false;
    ESP_LOGI(TAG, "Calling stop() - pcnt_unit: %p, group: %p", pcnt_unit, pcnt_unit->group);
    int count;
    pcnt_unit_get_count(pcnt_unit, &count);
    pcnt_unit_clear_count(pcnt_unit);
    position += count;
    ESP_LOGI(TAG, "Stopping - count:%d, position:%d, pcntwatchpoint:%d, pcn_objective:%d", count, position, pcnt_watch_point, pcnt_objective);
    
    // because counter overflown while speed target
    if (!pcnt_watch_point && from_ISR) {
        running = true;
        ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, _channel, HALF_DUTY, 0);
    }
    
    if (pcnt_watch_point) {
        // account for possible overflow        
        if (pcnt_watch_point != PCNT_HIGH_LIMIT && pcnt_watch_point != PCNT_LOW_LIMIT) pcnt_unit_remove_watch_point(pcnt_unit, pcnt_watch_point);
        if (!(pcnt_watch_point == pcnt_objective)) {
            pcnt_objective -= pcnt_watch_point;
            if (pcnt_objective > 0) pcnt_watch_point = (pcnt_objective < PCNT_HIGH_LIMIT) ? pcnt_objective : PCNT_HIGH_LIMIT;
            else pcnt_watch_point = (pcnt_objective > PCNT_LOW_LIMIT) ? pcnt_objective : PCNT_LOW_LIMIT;
            ESP_LOGD(TAG, "Trying to add watchpoint: %d", pcnt_watch_point);
            if (pcnt_watch_point != PCNT_HIGH_LIMIT && pcnt_watch_point != PCNT_LOW_LIMIT) {
                pcnt_unit_add_watch_point(pcnt_unit, pcnt_watch_point);
                pcnt_unit_clear_count(pcnt_unit);
            }
            running = true;
            ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, _channel, HALF_DUTY, 0);
            vTaskPrioritySet(NULL, prvPriority);
            xSemaphoreGive(pcnt_mutex);
            return;
        }
        pcnt_watch_point = 0;
    }
    if (stop_waiting && waiting) {
        xTaskNotifyGive(xWaitTaskHandle);
    }
    vTaskPrioritySet(NULL, prvPriority);
    xSemaphoreGive(pcnt_mutex);
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
    if (running) {
        xSemaphoreTake(pcnt_mutex, portMAX_DELAY);
        int count = 0;
        pcnt_unit_get_count(pcnt_unit, &count);
        xSemaphoreGive(pcnt_mutex);
        return position + count;
    }
    return position;
}

void ledc_stepper::reset_position()
{
    position = 0;
}
