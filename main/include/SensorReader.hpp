#pragma once

#include "driver/timer.h"
#include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

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

#define TOF_CONTINOUS

namespace sensors
{

struct Readings
    {
        // distance for each direction
        uint16_t distances[18]={0};
        // current degree in yaw axis
        float yaw=0.f;

        // current degree in yaw axis, from MPU-6050
        float gyaw=0.f;
        // is ktir touching black or white surface
        bool floor_sensors[NUM_OF_KTIRS]={0};

        bool stoped=false;

        bool IMUOnlyReading=false;

        float motorSpeed[2];

        Vec2Df velocity;

        // position in 2D space
        Vec2Df position=Vec2Df(0);

        // position in 2D space, from MPU-6050
        //Vec2Df gposition;
    };


class SensorReader
{

    protected:

    PositionFilter posfilter_x;
    PositionFilter posfilter_y;

    RotationFilter rotor;

    MultiVL *vl;

    MPU6050 mpu;

    Readings reads;

    SingleCounter ch1;
    SingleCounter ch2;

    MeanBuffer<Vec3Df,10> gyroMean;

    MeanBuffer<Vec3Df,10> accelMean;

    Vec3Df magReading;

    adc_oneshot_unit_handle_t hmd;

    uint8_t adc_result[NUM_OF_KTIRS*SOC_ADC_DIGI_RESULT_BYTES];

    uint16_t KtirThreshold;

    bool xCycleTask;

    Magnetrometer* mag;

    // initialize magnetrometer
    void init_mag();

    void read_mag();

    void init_peripherials();

    void init_sensors();

    bool SensorsFaulty;

    bool CalibrateIMU;

    void install_timer(uint32_t _dt);

    void install_adc();

    void read_adc();

    void start_timer()
    {
        timer_start(TIMER_GROUP_0,TIMER_0);
    }

    void stop_timer()
    {
        timer_pause(TIMER_GROUP_0,TIMER_0);
    }

    uint8_t from_angel_to_sensor_index(const float& angel) const; 

    static IRAM_ATTR bool _timer_callback_wrapper(void* arg)
    {
        SensorReader* reader=(SensorReader*)arg;

        reader->timer_callback();

        return false;
    }

    float yaw_error_tolerance;
    float distance_error_tolerance;

    public:

     // timer that will force step function to execute in each T time step
    void timer_callback()
    {
        this->xCycleTask=true;
    }

    SensorReader();

    // init peripherials and sensors
    void init(const config::SensorConfig& _config);

    void update_cfg(const config::SensorConfig& _config)
    {
        this->mpu.setCalibrationData(_config.calibr);
        this->yaw_error_tolerance=_config.yaw_error_tolerance;
        this->distance_error_tolerance=_config.distance_error_tolerance;
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
    }

    config::RotationFilterCFG getRoationFilterCFG()
    {
        return this->rotor.getCFG();
    }

    config::SensorConfig dump_cfg()
    {
        config::SensorConfig cfg;
        cfg.calibr=this->mpu.getCalibrationData();

        return cfg;
    }

    // perform reading
    void step();

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

    const bool& IMUCalibrationDone()
    {
        return this->CalibrateIMU;
    }

    void DoIMUCalibration()
    {
        this->CalibrateIMU=true;   
    }

    ~SensorReader();

};

};