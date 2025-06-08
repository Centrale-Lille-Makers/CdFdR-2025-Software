#include "Lidar.h"

static const char *TAG = "Lidar";

void detect_task(void *pvParameter)
{
    Lidar *lidar = static_cast<Lidar *>(pvParameter);
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(lidar->_DETECT_POLL_DELAY));
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
        ESP_LOGI(TAG, "Detecting...");
        if (lidar->detect())
            lidar->_motion->stop();
        else
            lidar->_motion->resume();
        ESP_LOGI(TAG, "detectTaskWatermark: %d", uxTaskGetStackHighWaterMark(NULL));
    }
}

Lidar::Lidar(int diam_balise, uart_port_t uart_num, uint8_t rx_pin, Motion *motion)
{
    _diam_balise = diam_balise; // pour déterminer les clusters de points
    _number_of_beacons = 0;
    for (int i = 0; i < NUMBER_OF_BEACONS_MAX; i++)
    {
        _balise_position[i] = {0, 0, 0};
    }

    _ld19p = new Ld19p(uart_num, rx_pin);
    _motion = motion;

    xTaskCreate(detect_task, "ledc_stepperStopTask", 8192 * 4, (void *)this, PRIORITY_DETECT_TASK, &_detect_task_handle);
}

inline double deg_to_rad(double alpha)
{
    return PI / 180 * alpha;
}

position_t get_abs_pos(position_t robot_pos, LidarPoint_t p)
{
    double angle_rel = deg_to_rad(p.angle / 100);
    double x_rel = p.distance * cos(angle_rel);
    double y_rel = p.distance * sin(angle_rel);
    double theta = deg_to_rad(robot_pos.r);
    double x_abs = robot_pos.x + x_rel * cos(theta) - y_rel * sin(theta);
    double y_abs = robot_pos.y + x_rel * sin(theta) + y_rel * cos(theta);
    return {x_abs, y_abs, 0};
}

bool Lidar::is_in_zone(position_t object_abs_pos, int zone, float delta)
{

    if (zone == 2)
    {
        // Intérieur de l'arène
        return (object_abs_pos.x >= delta && object_abs_pos.x <= ARENA_LENGTH - delta && object_abs_pos.y >= delta && object_abs_pos.y <= ARENA_WIDTH - delta);
    }
    if (zone == 1)
    {
        // Intérieur + Bordure de l'arène
        return (object_abs_pos.x >= -delta && object_abs_pos.x <= ARENA_LENGTH + delta && object_abs_pos.y >= -delta && object_abs_pos.y <= ARENA_WIDTH + delta);
    }
    if (zone == 0)
    {
        return true;
    }
    return false;
}

int Lidar::detect_object(LidarPoint_t raw_data[], int data_size, detected_object_t objects_detected[550], position_t robot_pos, int largeur_objet_min, int largeur_objet_max, int zone_in, int zone_out, int delta_zone)
{
    int number_of_detected_obj = 0;

    LidarPoint_t balise_buffer[550];
    int taille_objet = 0;
    for (int i = 0; i < data_size; i++)
    {
        position_t object_abs_pos = get_abs_pos(robot_pos, raw_data[i]);
        int next = (i + 1) % data_size;
        if (abs(raw_data[i].distance - raw_data[next].distance) < _diam_balise)
        {
            balise_buffer[taille_objet++] = raw_data[i];
            ESP_LOGI(TAG, "<detect_object> detection point, point %d de l'objet %d :(angle=%d, dist=%d)", taille_objet, number_of_detected_obj, raw_data[i].angle, raw_data[i].distance);
        }
        else if (is_in_zone(object_abs_pos, zone_in, delta_zone) && !is_in_zone(object_abs_pos, zone_out, delta_zone))
        {
            // objet détecté dans la zone
            balise_buffer[taille_objet++] = raw_data[i];
            LidarPoint_t first_point = balise_buffer[0];
            LidarPoint_t final_point = raw_data[i];
            uint16_t distance_objet = (first_point.distance + final_point.distance) / 2;
            double angle_objet_rad = deg_to_rad(((first_point.angle + final_point.angle) / 2) / 100.0);
            double largeur_objet_rad = distance_objet * sin(deg_to_rad(abs(first_point.angle - final_point.angle) / 100));
            if (largeur_objet_rad >= largeur_objet_min && largeur_objet_rad <= largeur_objet_max && number_of_detected_obj < 550)
            {
                objects_detected[number_of_detected_obj++] = {{robot_pos.x + distance_objet * cos(deg_to_rad(robot_pos.r) + angle_objet_rad),
                                                               robot_pos.x + distance_objet * cos(deg_to_rad(robot_pos.r) + angle_objet_rad),
                                                               0},
                                                              distance_objet,
                                                              angle_objet_rad,
                                                              largeur_objet_rad};
            }
        }
        else
        {
            taille_objet = 0;
        }
    }
    return number_of_detected_obj;
}

bool Lidar::set_number_of_beacons(int number_of_beacons)
{
    if (number_of_beacons <= NUMBER_OF_BEACONS_MAX)
    {
        _number_of_beacons = number_of_beacons;
        return true;
    }
    return false;
}

void Lidar::init_balise_pos(position_t robot_pos, int number_of_attempts_max)
{
    LidarPoint_t raw_data[550];
    int data_size = _ld19p->getFullTour(raw_data, 550);
    ESP_LOGI(TAG, "<init_balise_pos> Fulltour received (size = %d)", data_size);
    detected_object_t objects_detected[550];

    int number_of_detected_obj;
    int number_of_attempts = 0;
    do
    {
        number_of_detected_obj = detect_object(raw_data, data_size, objects_detected, robot_pos, BALISE_WIDTH_MIN, BALISE_WIDTH_MAX, 1, 2, MARGIN_BEACON_POS);
        ESP_LOGI(TAG, "<init_balise_pos> Detection d'objet passée, %d objets detectés", number_of_detected_obj);
        if (number_of_detected_obj == _number_of_beacons)
        {
            continue;
        }
        // delay de 0.1s
        number_of_attempts++;
    } while (number_of_attempts <= number_of_attempts_max);
    if (number_of_detected_obj == _number_of_beacons)
    {
        ESP_LOGI(TAG, "Les balises %d ont été détectées avec succès", _number_of_beacons);
    }
    else
    {
        ESP_LOGE(TAG, "Initialisation des positions des balises : %d balises détectées au lieu de %d", number_of_detected_obj, _number_of_beacons);
    }
    for (int i = 0; i < min(number_of_detected_obj, _number_of_beacons); i++)
    {
        _balise_position[i] = objects_detected[i].pos;
    }
}

void Lidar::start_detection()
{
    xTaskNotifyGive(_detect_task_handle);
    ESP_LOGI(TAG, "Detection started");
}

void Lidar::stop_detection()
{
    xTaskNotifyStateClear(_detect_task_handle);
    _motion->resume();
    ESP_LOGI(TAG, "Detection stopped");
}

#define MIN_CLUSTER_SIZE 2 // taille du cluster de poin minimal pour décréter que c'est un obstacle
bool Lidar::detect()
{
    ESP_LOGI(TAG, "Detecting...");

    LidarPoint_t points[550];

    position_t delta = _motion->relative_pos_futur();

    LidarPoint_t futur_loc;

    futur_loc.distance = delta.x;
    futur_loc.angle = (int)(delta.r * 100 + 9000) % 36000;
    ESP_LOGI(TAG, "Current_pos_point distance: %d, angle: %d", futur_loc.distance, futur_loc.angle);
    if (futur_loc.distance == 0)
        return false;
    int nb_pt = _ld19p->getFullTour(points, 550);

    int current_cluster_size = 0;
    bool cluster_found = false;
    for (int i = 0; i < nb_pt + MIN_CLUSTER_SIZE; ++i)
    {
        if (point_is_obstacle(points[i % nb_pt], futur_loc))
            ++current_cluster_size;
        else
            current_cluster_size = 0;
        if (current_cluster_size >= MIN_CLUSTER_SIZE)
        {
            cluster_found = true;
            break;
        }
    }
    if (cluster_found)
    {
        ESP_LOGI(TAG, "Detected obstacle");
        return true;
    }
    else
    {
        ESP_LOGI(TAG, "Did not detect obstacle");
        return false;
    }
}

bool Lidar::point_is_obstacle(LidarPoint_t point, LidarPoint_t futur_pos) // à optimiser avec un rectangle
{
    uint16_t angle_diff = (point.angle + 36000 - futur_pos.angle) % 36000;
    //(angle_diff < 4000 || angle_diff > 36000-4000) &&
    return (angle_diff < 4000 || angle_diff > 36000 - 4000) && point.distance > 150 && point.distance < 2 * futur_pos.distance + 350;
}

int distance(position_t p1, position_t p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

position_t sum_pos(position_t p1, position_t p2)
{
    return {p1.x + p2.x, p1.y + p2.y, p1.r + p2.r};
}

position_t sub_pos(position_t p1, position_t p2)
{
    return {p1.x - p2.x, p1.y - p2.y, p1.r - p2.r};
}

position_t Lidar::update_robot_pos(position_t robot_pos)
{
    LidarPoint_t raw_data[550];
    int data_size = _ld19p->getFullTour(raw_data, 550);

    detected_object_t objects_detected[550];
    int number_of_detection = detect_object(raw_data, data_size, objects_detected, robot_pos, BALISE_WIDTH_MIN, BALISE_WIDTH_MAX, 1, 2, MARGIN_BEACON_POS);

    position_t sum_position_error = {0, 0, 0};
    int number_of_beacons_reconized = 0;
    for (int i = 0; i < _number_of_beacons; i++)
    {
        position_t real_position = _balise_position[i];
        int j_min = 0;
        double dist_min = 3000;
        for (int j = 0; j < number_of_detection; j++)
        {
            position_t measured_position = objects_detected[j].pos;
            if (distance(real_position, measured_position) < dist_min)
            {
                j_min = j;
                dist_min = distance(real_position, measured_position);
            }
        }
        if (dist_min < MARGIN_BEACON_POS)
        {
            number_of_beacons_reconized++;
            sum_position_error = sum_pos(sum_position_error, sub_pos(real_position, objects_detected[j_min].pos));
        }
    }

    return {sum_position_error.x / number_of_beacons_reconized, sum_position_error.y / number_of_beacons_reconized, sum_position_error.r / number_of_beacons_reconized};
}
