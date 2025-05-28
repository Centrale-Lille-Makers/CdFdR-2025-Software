#pragma once

//#define M_R_SENSE 0.11f

#define M_DRIVE_EN_PIN 15      // Enable is common to all four drive motors
//#define M_DRIVE_RX 18               // For setting TMC2209 parameters
//#define M_DRIVE_TX 17               // For setting TMC2209 parameters
//#define M_DRIVE_SERIAL Serial1 // For setting TMC2209 parameters
//#define M_DRIVE_CURRENT_MA 2000
#define M_DRIVE_MICROSTEP 32 // default motor microstep

//#define M1_DRIVER_ADDRESS 0b00
#define M1_DIR_PIN 4
#define M1_STP_PIN 5
#define M1_MICROSTEP M_DRIVE_MICROSTEP //8 // 8->32 16->16 64->128 32->64 wut ???

//#define M2_DRIVER_ADDRESS 0b01
#define M2_DIR_PIN 6
#define M2_STP_PIN 7
#define M2_MICROSTEP M_DRIVE_MICROSTEP //32

//#define M3_DRIVER_ADDRESS 0b10
#define M3_DIR_PIN 8
#define M3_STP_PIN 3
#define M3_MICROSTEP M_DRIVE_MICROSTEP //64

//#define M4_DRIVER_ADDRESS 0b11
#define M4_DIR_PIN 9
#define M4_STP_PIN 10
#define M4_MICROSTEP M_DRIVE_MICROSTEP //16


#define M_CHARIOT_EN_PIN 16         // Enable is common to all two lift motors
//#define M_CHARIOT_RX 2              // For setting TMC2209 parameters
//#define M_CHARIOT_TX 42             // For setting TMC2209 parameters
//#define M_CHARIOT_SERIAL Serial2    // For setting TMC2209 parameters
//#define M_CHARIOT_CURRENT_MA 1000
//#define M_CHARIOT_CAL_CURRENT_MA 50
//#define M_CHARIOT_MICROSTEP 16
#define M_CHARIOT_STEPS_PER_TURN 200 // good
#define M_CHARIOT_TRAVEL_PER_TURN 8 // good
#define M_CHARIOT_SPEED_MM 10
#define M_CHARIOT_CAL_SPEED 10

//#define M5_DRIVER_ADDRESS 0b00
#define M5_DIR_PIN 11
#define M5_STP_PIN 12
#define M5_MICROSTEP 32 // good

//#define M6_DRIVER_ADDRESS 0b10
#define M6_DIR_PIN 13
#define M6_STP_PIN 14
#define M6_MICROSTEP 128


#define LIDAR_SERIAL UART_NUM_0
#define LIDAR_RX_PIN 44


#define DIO_IHM_PIN 41
#define CLK_IHM_PIN 40
#define STB_IHM_PIN 37


#define ELECTRO1_PIN 48
#define ELECTRO2_PIN 47

#define PUMP_VALVE1_PIN 39
#define PUMP_VALVE2_PIN 38


#define TIRETTE_PIN 21


#define V_BATT_PIN 1


#define PRIORITY_LIFT_STEPPERS tskIDLE_PRIORITY + 5
#define PRIORITY_CAL_LIFT tskIDLE_PRIORITY + 6
#define PRIORITY_LIFT_STOP_ISR tskIDLE_PRIORITY + 7