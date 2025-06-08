#pragma once

#include <stdint.h>
#include "driver/gpio.h"
#include "libs/Ld19p.h"
#include "config.h"
#include <cmath>
#include "esp_log.h"
#include "Motion.h"

typedef struct
{
    position_t pos;
    uint16_t distance; // mm
    double angle;      // rad
    double largeur;    // mm
} detected_object_t;

class Lidar
{
    friend void detect_task(void *pvParameter);

private:
    static constexpr int _DETECT_POLL_DELAY = 200; // delay in ms

public:
    Lidar(int diam_balise, uart_port_t uart_num, uint8_t rx_pin, Motion *motion);
    position_t update_robot_pos(position_t robot_pos);
    bool set_number_of_beacons(int number_of_beacons);
    bool is_in_zone(position_t object_abs_pos, int zone, float delta);
    int detect_object(LidarPoint_t raw_data[], int data_size, detected_object_t objects_detected[550], position_t robot_pos, int largeur_object_min, int largeur_objet_max, int zone_in, int zone_out, int delta_zone);
    void init_balise_pos(position_t robot_pos, int number_of_attempts_max);
    bool get_number_of_beacons(int number_of_beacons);
    void start_detection();
    void stop_detection();

private:
    TaskHandle_t _detect_task_handle;
    position_t _balise_position[NUMBER_OF_BEACONS_MAX];
    int _number_of_beacons;
    int _diam_balise;
    Ld19p *_ld19p;
    Motion *_motion;
    bool point_is_obstacle(LidarPoint_t point, LidarPoint_t futur_pos);
    bool detect(); // detects if a cluster of pointsIsObstacle exists, and notifies motion.
};