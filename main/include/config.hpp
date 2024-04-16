#pragma once

#include "driver/i2c.h"

#define OLED_TASK_STACK_SIZE 32768

#define MAIN_TASK_STACK_SIZE 32768

#define I2C_SENSOR_PORT I2C_NUM_0
#define SENSOR_SDA 13
#define SENSOR_SCL 12

#define I2C_MENU_PORT I2C_NUM_1
#define MENU_SDA 11
#define MENU_SCL 10

#define I2C_MASTER_FREQ_HZ 100000 


// specified multiplexer ports where sensors are connected
const unsigned char SensorList[]={};

// specified an angel of each sensor
const float SensorAngleOffset[]={};

// angel between each sensor, in radians
#define SENSOR_SPACING 0.3490658504
// about 20 degress

#define NUM_OF_SENSORS 1

// ADC unit used for KTIR sampling
#define KTIR_ADC 0

// pins from wich KTIR will be sampled
const unsigned char KTIRChannel[]={0};

#define NUM_OF_KTIRS 1


#define SAMPLE_FREQ 250 // In Hz

#define SAMPLE_TIME 1.0/SAMPLE_FREQ

#define KTIR_THRESHOLD 2048


#define MPU6050_ADDRESS 0x68


#define PCNT_CH1 (gpio_num_t)1
#define PCNT_CH2 (gpio_num_t)41

// a error between measurmed yaws above which there will be only IMU reading taken to account
#define YAW_TOLERANCE 0.25

// a error between measurmed distance above which there will be only IMU reading taken to account
#define DISTANCE_TOLERANCE 0.5

//constant use to convert pulse into distance in cm
#define PULSE_TO_DISTANCE 200.f

//distance between wheel and it's mass center in cm
#define D_WHEELS 4.f

//a weight in delta degree and delta distance change, a weight for IMU is (1-PCNT_CONFIDENCE_F)
#define PCNT_CONFIDENCE_F 0.6f

// Access Point configuration

#define AP_SSID "GaraiPanel"

#define AP_PSK "konarpany_down"

// motor gpio config

#define MOTOR_FREQ 50

const gpio_num_t MotorsPin[]={(gpio_num_t)40,(gpio_num_t)39,(gpio_num_t)2,(gpio_num_t)42};


#define BUZZER_PIN (gpio_num_t)14


#define POS_X_CFG_FILE "posxfilter.json"
#define POS_Y_CFG_FILE "posyfilter.json"


#define FUN_BUTTON (gpio_num_t)17
#define MODE_BUTTON (gpio_num_t)18