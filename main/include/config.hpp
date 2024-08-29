#pragma once

#include <driver/i2c.h>

#include <hal/adc_types.h>

#define OLED_TASK_STACK_SIZE 32768

#define MAIN_TASK_STACK_SIZE 32768

#define MIN_TASK_STACK_SIZE 8196

#define I2C_SENSOR_PORT I2C_NUM_0
#define SENSOR_SDA GPIO_NUM_13
#define SENSOR_SCL GPIO_NUM_12

#define I2C_MENU_PORT I2C_NUM_1
#define MENU_SDA GPIO_NUM_11
#define MENU_SCL GPIO_NUM_10

#define I2C_MASTER_FREQ_HZ 400000 

// in seconds
#define OTA_WS_TIMEOUT 10

#define BATTERY_ADC_CHANNEL ADC_CHANNEL_8

// specified multiplexer ports where sensors are connected
const unsigned char SensorList[]={0,1,2,3,4};

// specified an angel of each sensor
const float SensorAngleOffset[]={0.3490658504,0.3490658504*2,0.3490658504*3,0.3490658504*4};

// angel between each sensor, in radians
#define SENSOR_SPACING 0.3490658504
// about 20 degress

#define NUM_OF_SENSORS 1

// ADC unit used for KTIR sampling
#define KTIR_ADC 0

// pins from wich KTIR will be sampled
const adc_channel_t KTIRChannel[]={ADC_CHANNEL_3,ADC_CHANNEL_4,ADC_CHANNEL_5};

#define NUM_OF_KTIRS 3

// 50 Hz
#define VL_SAMPLE_TIME_MS 20

#define SAMPLE_FREQ 100 // In Hz

// 100 Hz
#define SAMPLE_TIME_MS 10

#define SAMPLE_TIME (float)SAMPLE_TIME_MS/1000.f

#define MOTOR_UPDATE_TIME_MS 50

#define MOTOR_UPDATE_TIME (float)MOTOR_UPDATE_TIME_MS/1000.f

// 1 kHz
#define ENCODER_UPDATE_TIME_MS 5

#define ENCODER_UPDATE_TIME (float)ENCODER_UPDATE_TIME_MS/1000.f;

#define MAX_ENGINE_POWER 4095.f

#define WATCHDOG_TIME_MS 100

#define KTIR_THRESHOLD 2048

#define MPU6050_ADDRESS 0x68

#define MPU6050_INT_PIN GPIO_NUM_21

#define PCNT_CH1 GPIO_NUM_1
#define PCNT_CH2 GPIO_NUM_41

#define START_PIN GPIO_NUM_16

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


#define FUN_BUTTON GPIO_NUM_17
#define MODE_BUTTON GPIO_NUM_18