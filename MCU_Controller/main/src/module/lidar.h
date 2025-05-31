#pragma once

//#define PI 3.141592653589793

#include <stdint.h>
#include "driver/gpio.h"
#include "module/lidar.h"
#include "libs/ld19p.h" 
#include "config.h"
#include <cmath>
#include "esp_log.h"
#include "motion.h"

typedef struct {
    position_t pos; 
    uint16_t distance; // mm
    double angle; // rad
    double largeur; // mm
} detected_object_t;


class lidar {
    friend void detectTask(void * pvParameter );
public:
    lidar(int diam_balise, uart_port_t uart_num, uint8_t rx_pin, Motion *motion);
    position_t update_robot_pos(position_t robot_pos);
    int number_of_beacons;
    bool set_number_of_beacons(int number_of_beacons);
    position_t balise_position[NUMBER_OF_BEACONS_MAX];
    bool is_in_zone(position_t object_abs_pos, int zone, float delta);
    int detect_object(LidarPoint_t raw_data[], int data_size, detected_object_t objects_detected[550], position_t robot_pos, int largeur_object_min, int largeur_objet_max, int zone_in, int zone_out, int delta_zone);
    void init_balise_pos(position_t robot_pos, int number_of_attempts_max);
    bool get_number_of_beacons(int number_of_beacons);
    void startDetection();
    void stopDetection();

    bool detect(); // detects if a cluster of pointsIsObstacle exists, and notifies motion.
    
private:
    bool pointIsObstacle(LidarPoint_t point, LidarPoint_t futur_pos);
    TaskHandle_t detectTaskHandle;
    SemaphoreHandle_t detectTaskSemaphore;
    bool detection_enabled = true;
    int diam_balise;
    ld19p* _ld19p;
    Motion *motion;

};