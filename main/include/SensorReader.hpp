#pragma once

#include <driver/i2c.h>

#include <driver/gpio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include <esp_adc/adc_oneshot.h>
#include <esp_log.h>

#include "config.hpp"

#include "Vec.hpp"
#include "MultiVL.hpp"
#include "MPU6050.hpp"
#include "SingleCounter.hpp"

#include "MeanBuffer.hpp"

#include "ConfigLoader.hpp"
#include "PositionFilter.hpp"

#include "RotationFilter.hpp"

#include "Magnetrometer.hpp"

#include "HMC5883.hpp"
#include "QMC5883.hpp"

extern "C"
{
    #include "MadgwickAHRS.h"
}

#define TOF_CONTINOUS

namespace sensors
{

struct Readings
    {
        //  magnetrometer readings
        Vec3Df magReading;

        //  a voltage from battery
        float battery_voltage = 0.f;

        // distance for each direction
        uint16_t distances[18] = {0};
        // current degree in yaw axis
        float yaw = 0.f;

        // current degree in yaw axis, from MPU-6050
        float gyaw = 0.f;

        // current degree in yaw axis, from encoders
        float eyaw = 0.f;

        // calculated from accelerometer
        float shock_value;
        float shock_angel;

        // is ktir touching black or white surface
        bool floor_sensors[NUM_OF_KTIRS]={0};

        bool stoped=false;

        bool IMUOnlyReading=false;

        float motorSpeed[2] = {0};

        Vec2Df velocity;

        // position in 2D space
        Vec2Df position = Vec2Df(0);

        // position in 2D space, from MPU-6050
        Vec2Df gposition = Vec2Df(0);

        // position in 2D space, from encoders
        Vec2Df epostion = Vec2Df(0);

    };


class SensorReader
{

    protected:

    
    StackType_t* encoder_stack;
    StackType_t* tof_stack;
    StackType_t* imu_stack;
    StackType_t* fusion_stack;

    StaticTask_t encoder_task;
    StaticTask_t tof_task;
    StaticTask_t imu_task;
    StaticTask_t fusion_task;

    SemaphoreHandle_t semp;

    PositionFilter posfilter_x;
    PositionFilter posfilter_y;

    RotationFilter rotor;

    size_t CalbirationSteps;

    MultiVL *vl;

    MPU6050 mpu;

    Readings reads;

    SingleCounter ch1;
    SingleCounter ch2;

    MeanBuffer<Vec3Df,10> gyroMean;

    MeanBuffer<Vec3Df,10> accelMean;

    MeanBuffer<Vec3Di,100> gyroCalibrMean;

    MeanBuffer<Vec3Di,100> accelCalibrMean;


    adc_oneshot_unit_handle_t hmd;

    uint8_t adc_result[NUM_OF_KTIRS*SOC_ADC_DIGI_RESULT_BYTES];

    uint16_t KtirThreshold;

    Magnetrometer* mag;

    EventGroupHandle_t imuEvent;

    // initialize magnetrometer
    void init_mag();

    void read_mag();

    void init_peripherials();

    void init_sensors();

    void init_tasks();

    bool SensorsFaulty;

    bool CalibrateIMU;

    size_t CalibrationCounter;

    void install_adc();

    uint8_t from_angel_to_sensor_index(const float& angel) const; 

    float yaw_error_tolerance;
    float distance_error_tolerance;

    public:

    SensorReader();

    // init peripherials and sensors
    void init(const config::SensorConfig& _config);

    void update_cfg(const config::SensorConfig& _config)
    {
        this->mpu.setCalibrationData(_config.calibr);

        this->Lock();

        this->rotor.reset();
        this->reads.yaw = 0.0;

        this->Unlock();
    }

    void updatePositionFilterYCFG(const config::PositionFilterCFG& _config)
    {
        this->posfilter_y.setFromCFG(_config);
    }

    config::PositionFilterCFG getPositionFilterYCFG()
    {
        return this->posfilter_y.getCFG();
    }

    void updatePositionFilterXCFG(const config::PositionFilterCFG& _config)
    {
        this->posfilter_x.setFromCFG(_config);
    }

    config::PositionFilterCFG getPositionFilterXCFG()
    {
        return this->posfilter_x.getCFG();
    }

    void updateRoationFilterCFG(const config::RotationFilterCFG& _config)
    {
        this->rotor.setFromCFG(_config);

        this->Lock();

        this->rotor.reset();
        this->reads.yaw = 0.0;

        this->Unlock();
    }

    config::RotationFilterCFG getRoationFilterCFG()
    {
        return this->rotor.getCFG();
    }

    config::SensorConfig dump_cfg()
    {
        config::SensorConfig cfg;
        cfg.calibr=this->mpu.getCalibrationData();
        cfg.beta = MadgwickGetBeta();

        return cfg;
    }

    config::MagConfig dump_mag_cfg()
    {
        if( this->mag == NULL)
        {
            return config::ConfigLoader::mag_default();
        }

        return this->mag->getCFG();
    }

    void update_mag_cfg(const config::MagConfig& cfg)
    {
        if( this->mag == NULL)
        {
            return;
        }

        this->mag->updateCFG(cfg);
    }

    const MeanBuffer<Vec3Df,10>& getGyroscopeBuffer()
    {
        return this->gyroMean;
    }

    const MeanBuffer<Vec3Df,10>& getAccelerometerBuffer()
    {
        return this->accelMean;
    }

    BaseType_t imu_gpio_interupt();

    void read_imu();

    void read_encoders();

    void read_adc();

    void fusion();

    void tofs_read();

    const Readings& read() const
    {
        return this->reads;
    }

    operator bool() const
    {
        return this->SensorsFaulty;
    }

    void ClearError()
    {
        SensorsFaulty=false;
    }

    bool IMUCalibrationDone()
    {
        return this->CalibrateIMU;
    }

    void DoIMUCalibration(size_t N=10)
    {
        ESP_LOGI("MAIN","IMU calibration begin, with steps: %u",N);
        this->CalbirationSteps=N;
        this->CalibrateIMU=true;   
        this->CalibrationCounter=0;
    }

    size_t StepsLeftUntilCalibration()
    {
        if(! this->CalibrateIMU )
        {
            return 0;
        }

        return this->CalbirationSteps - this->CalibrationCounter;
    }

    // I don't think it is convinient
    void Lock()
    {
        xSemaphoreTake(this->semp,portMAX_DELAY);
    }

    void Unlock()
    {
        xSemaphoreGive(this->semp);
    }

    ~SensorReader();

};

};