#include "Motion.h"
#include "config.h"
#include "Lidar.h"

static const char *TAG = "motion";

Motion::Motion()
{
    // M1_driver = new TMC2209Stepper(&M_DRIVE_SERIAL, M_R_SENSE, M1_DRIVER_ADDRESS);
    // M2_driver = new TMC2209Stepper(&M_DRIVE_SERIAL, M_R_SENSE, M2_DRIVER_ADDRESS);
    // M3_driver = new TMC2209Stepper(&M_DRIVE_SERIAL, M_R_SENSE, M3_DRIVER_ADDRESS);
    // M4_driver = new TMC2209Stepper(&M_DRIVE_SERIAL, M_R_SENSE, M4_DRIVER_ADDRESS);

    // M1_driver->begin();
    // M2_driver->begin();
    // M3_driver->begin();
    // M4_driver->begin();

    // M1_driver->toff(5);
    // M2_driver->toff(5);
    // M3_driver->toff(5);
    // M4_driver->toff(5);

    // set_RMS(M_DRIVE_CURRENT_MA);
    // set_microstep(M_DRIVE_MICROSTEP);

    _stepper_engine.init();
    _M1_stepper = _stepper_engine.stepperConnectToPin(M1_STP_PIN);
    _M1_stepper->setDirectionPin(M1_DIR_PIN);
    //_M1_stepper->attachToPulseCounter(); We will try not tu have to use it

    _M2_stepper = _stepper_engine.stepperConnectToPin(M2_STP_PIN);
    _M2_stepper->setDirectionPin(M2_DIR_PIN);
    //_M2_stepper->attachToPulseCounter();

    _M3_stepper = _stepper_engine.stepperConnectToPin(M3_STP_PIN);
    _M3_stepper->setDirectionPin(M3_DIR_PIN);

    _M4_stepper = _stepper_engine.stepperConnectToPin(M4_STP_PIN);
    _M4_stepper->setDirectionPin(M4_DIR_PIN);

    _en_pin = (gpio_num_t)M_DRIVE_EN_PIN;
    gpio_reset_pin(_en_pin);
    gpio_set_direction(_en_pin, GPIO_MODE_OUTPUT);

    _current_objective[0].steps = 0;
    _current_objective[1].steps = 0;
    _current_objective[2].steps = 0;
    _current_objective[3].steps = 0;

    disable_motors();

    _stepping_mutex = xSemaphoreCreateMutex();

    ESP_LOGI(TAG, "created");
}

void Motion::disable_motors()
{
    gpio_set_level(_en_pin, 1);
}

void Motion::enable_motors()
{
    gpio_set_level(_en_pin, 0);
}

void Motion::calibrate(IHM *ihm)
{
    ihm->write_msg("c mot  y");
    ihm->wait_button(7);
    enable_motors();
    _pos.x = 0;
    _pos.y = 0;
    _pos.r = 0;
    _objective_pos = _pos;
    ihm->write_msg("accepted");
    vTaskDelay(pdMS_TO_TICKS(1000));
    ihm->write_msg("");
}

void Motion::stop()
{
    if (!_stopped)
    {
        ESP_LOGI(TAG, "Stopping");
        UBaseType_t prvPriority = uxTaskPriorityGet(NULL);
        vTaskPrioritySet(NULL, PRIORITY_SYNC_MOTORS);
        _stopped = true;
        // _M1_stepper->setAcceleration(_current_objective[0].accel*3);
        // _M2_stepper->setAcceleration(_current_objective[1].accel*3);
        // _M3_stepper->setAcceleration(_current_objective[2].accel*3);
        // _M4_stepper->setAcceleration(_current_objective[3].accel*3);
        _M1_stepper->stopMove();
        _M2_stepper->stopMove();
        _M3_stepper->stopMove();
        _M4_stepper->stopMove();
        vTaskPrioritySet(NULL, prvPriority);
    }
}

void Motion::resume()
{
    if (_stopped)
    {
        ESP_LOGI(TAG, "Resuming");
        execute_steps();
        _stopped = false;
    }
}

void Motion::set_pos(position_t absolute_pos)
{
    _pos = absolute_pos;
}

void Motion::execute_moves(double c1, double c2, double c3, double c4, int steps, int speed, int accel, bool blocking) // absolue
{
    ESP_LOGI(TAG, "Executing moves");
    _moved = true;
    xSemaphoreTake(_stepping_mutex, portMAX_DELAY);
    _current_objective[0].steps = c1 * steps * M1_MICROSTEP;
    _current_objective[1].steps = c2 * steps * M2_MICROSTEP;
    _current_objective[2].steps = c3 * steps * M3_MICROSTEP;
    _current_objective[3].steps = c4 * steps * M4_MICROSTEP;

    _current_objective[0].speed = fabs(c1) * speed * M1_MICROSTEP;
    _current_objective[1].speed = fabs(c2) * speed * M2_MICROSTEP;
    _current_objective[2].speed = fabs(c3) * speed * M3_MICROSTEP;
    _current_objective[3].speed = fabs(c4) * speed * M4_MICROSTEP;

    _current_objective[0].accel = fabs(c1) * accel * M1_MICROSTEP;
    _current_objective[1].accel = fabs(c2) * accel * M2_MICROSTEP;
    _current_objective[2].accel = fabs(c3) * accel * M3_MICROSTEP;
    _current_objective[3].accel = fabs(c4) * accel * M4_MICROSTEP;

    _current_objective[0].jerk = fabs(c1) * M1_MICROSTEP;
    _current_objective[1].jerk = fabs(c2) * M2_MICROSTEP;
    _current_objective[2].jerk = fabs(c3) * M3_MICROSTEP;
    _current_objective[3].jerk = fabs(c4) * M4_MICROSTEP;

    execute_steps();
    xSemaphoreGive(_stepping_mutex);
    if (blocking)
    {
        ESP_LOGI(TAG, "Waiting move executions");
        wait();
    }
    ESP_LOGI(TAG, "Done executing moves");
}

void Motion::execute_steps()
{
    // ESP_LOGI(TAG, "Executing steps");
    // ESP_LOGI(TAG, "Stepper 1: steps: %d, speed: %d, accel: %d, liear_accel: %d", (int)_current_objective[0].steps, (int)_current_objective[0].speed, (int)_current_objective[0].accel, (int)_current_objective[0].jerk);
    _M1_stepper->setSpeedInHz(_current_objective[0].speed);
    _M2_stepper->setSpeedInHz(_current_objective[1].speed);
    _M3_stepper->setSpeedInHz(_current_objective[2].speed);
    _M4_stepper->setSpeedInHz(_current_objective[3].speed);
    _M1_stepper->setAcceleration(_current_objective[0].accel);
    _M2_stepper->setAcceleration(_current_objective[1].accel);
    _M3_stepper->setAcceleration(_current_objective[2].accel);
    _M4_stepper->setAcceleration(_current_objective[3].accel);
    _M1_stepper->setLinearAcceleration(_current_objective[0].jerk);
    _M2_stepper->setLinearAcceleration(_current_objective[1].jerk);
    _M3_stepper->setLinearAcceleration(_current_objective[2].jerk);
    _M4_stepper->setLinearAcceleration(_current_objective[3].jerk);

    UBaseType_t prvPriority = uxTaskPriorityGet(NULL);
    vTaskPrioritySet(NULL, PRIORITY_SYNC_MOTORS);
    _moved = true;
    _M1_stepper->moveTo(_current_objective[0].steps);
    _M2_stepper->moveTo(_current_objective[1].steps);
    _M3_stepper->moveTo(_current_objective[2].steps);
    _M4_stepper->moveTo(_current_objective[3].steps);
    vTaskPrioritySet(NULL, prvPriority);
}

position_t Motion::relative_pos_futur()
{
    position_t delta = {0, 0, 0};
    if (_stopped)
    {
        delta = get_pos_delta(2);
    }
    else if (_moved)
    {
        delta = get_pos_delta(1);
    }
    position_t pos = get_pos();

    delta.r = atan2(delta.y, delta.x) * 180 / PI - pos.r;
    delta.x = sqrt(delta.x * delta.x + delta.y * delta.y);
    return delta; // x = distance, r = angle.
}

void Motion::wait()
{
    while (is_running())
    {
        position_t pos = get_pos();
        ESP_LOGE(TAG, "Waiting. Current pos - x: %f, y: %f, r: %f", pos.x, pos.y, pos.r);
        pos = get_pos_delta(1);
        ESP_LOGE(TAG, "Waiting. Futur pos - x: %f, y: %f, r: %f", pos.x, pos.y, pos.r);
        pos = get_pos_delta(2);
        ESP_LOGE(TAG, "Waiting. If had moved pos - x: %f, y: %f, r: %f", pos.x, pos.y, pos.r);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    update_pos();
    ESP_LOGE(TAG, "Done waiting. Current pos - x: %f, y: %f, r: %f", _pos.x, _pos.y, _pos.r);
}

void Motion::update_pos()
{
    if (is_running() || !_moved)
        return;
    xSemaphoreTake(_stepping_mutex, portMAX_DELAY);
    _pos = _objective_pos;
    _M1_stepper->setCurrentPosition(0);
    _M2_stepper->setCurrentPosition(0);
    _M3_stepper->setCurrentPosition(0);
    _M4_stepper->setCurrentPosition(0);
    _moved = false;
    xSemaphoreGive(_stepping_mutex);
}

position_t Motion::get_pos()
{
    position_t pos;
    pos.x = _pos.x;
    pos.y = _pos.y;
    pos.r = _pos.r;
    if (_moved)
    {
        position_t delta = get_pos_delta();
        pos.x += delta.x;
        pos.y += delta.y;
        pos.r += delta.r;
    }
    return pos;
}

position_t Motion::get_pos_delta(uint8_t type)
{
    double d;
    if (_current_objective[0].steps != 0)
    {
        int32_t steps_pos = 0;
        switch (type)
        {
        case 0:
            steps_pos = _M1_stepper->getCurrentPosition();
            break;
        case 1:
            steps_pos = _M1_stepper->stepsToStop() * ((_current_objective[0].steps > 0) - (_current_objective[0].steps < 0));
            break;
        case 2:
            steps_pos = (_current_objective[0].steps - _M1_stepper->getCurrentPosition() < _current_objective[0].speed) ? _current_objective[0].steps - _M1_stepper->getCurrentPosition() : _current_objective[0].speed * ((_current_objective[0].steps > 0) - (_current_objective[0].steps < 0));
            break;
        default:
            break;
        }
        d = (double)steps_pos / (double)_current_objective[0].steps;
    }
    else if (_current_objective[1].steps != 0)
    {
        int32_t steps_pos = 0;
        switch (type)
        {
        case 0:
            steps_pos = _M2_stepper->getCurrentPosition();
            break;
        case 1:
            steps_pos = _M2_stepper->stepsToStop() * ((_current_objective[1].steps > 0) - (_current_objective[1].steps < 0));
            break;
        case 2:
            steps_pos = (_current_objective[1].steps - _M2_stepper->getCurrentPosition() < _current_objective[1].speed) ? _current_objective[1].steps - _M2_stepper->getCurrentPosition() : _current_objective[1].speed * ((_current_objective[1].steps > 0) - (_current_objective[1].steps < 0));
            break;
        default:
            break;
        }
        d = (double)steps_pos / (double)_current_objective[1].steps;
    }
    else
    {
        ESP_LOGE(TAG, "Asked get_pos_delta but havent moved ?");
        d = 0;
    }
    position_t delta;
    delta.r = (_objective_pos.r - _pos.r) * d;
    delta.x = (_objective_pos.x - _pos.x) * d;
    delta.y = (_objective_pos.y - _pos.y) * d;
    return delta;
}

bool Motion::is_running()
{
    return _stopped || _M1_stepper->isRunning() || _M2_stepper->isRunning() || _M3_stepper->isRunning() || _M4_stepper->isRunning();
}

// void Motion::set_RMS(uint16_t current)
// {
//     M1_driver->rms_current(current);
//     M2_driver->rms_current(current);
//     M3_driver->rms_current(current);
//     M4_driver->rms_current(current);
// }

// void Motion::set_microstep(uint16_t ms)
// {
//     M1_driver->microsteps(ms);
//     M2_driver->microsteps(ms);
//     M3_driver->microsteps(ms);
//     M4_driver->microsteps(ms);
// }

void Motion::translate(int distance, double alpha, int speed_robot, int accel_robot, bool blocking)
{
    ESP_LOGE(TAG, "Translate - d: %d, alpha: %f", distance, alpha);
    double A = M_DRIVE_STEPS_PER_TURN / (2 * PI * WHEEL_RADIUS) * ODOM_CORRECTION_TRANSLATION;
    int dist_robot_step = (int)distance * A;
    int speed_robot_step = (int)speed_robot * A;
    int accel_robot_step = (int)accel_robot * A;
    double a = cos(alpha * PI / 180) * 0.70710678;
    double b = sin(alpha * PI / 180) * 0.70710678;
    double c1 = -a + b;
    double c2 = -a - b;
    double c3 = +a - b;
    double c4 = +a + b;
    // ESP_LOGI(TAG, "c1: %f, c2: %f, c3: %f, c4: %f", c1, c2, c3, c4);
    ESP_LOGI(TAG, "Waiting before executing moves");
    wait();
    _objective_pos = _pos;
    _objective_pos.x += -distance * sin((alpha + _pos.r) * PI / 180);
    _objective_pos.y += distance * cos((alpha + _pos.r) * PI / 180);
    execute_moves(c1, c2, c3, c4, dist_robot_step, speed_robot_step, accel_robot_step, blocking);
}

void Motion::rotate(double dtheta, int rotation_speed_robot, int rotation_accel_robot, bool blocking)
{
    ESP_LOGE(TAG, "Rotating - dtheta: %f", dtheta);
    double A = ROBOT_RADIUS * M_DRIVE_STEPS_PER_TURN / (360 * WHEEL_RADIUS) * ODOM_CORRECTION_ROTATION;
    int dist_motors_step = (int)(dtheta * A);
    int speed_motors_step = (int)(rotation_speed_robot * A);
    int accel_motors_step = (int)(rotation_accel_robot * A);

    ESP_LOGI(TAG, "Waiting before executing moves");
    wait();
    _objective_pos = _pos;
    _objective_pos.r += dtheta;
    execute_moves(1, 1, 1, 1, dist_motors_step, speed_motors_step, accel_motors_step, blocking);
}