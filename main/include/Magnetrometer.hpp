#pragma once

#include <cinttypes>

#include <esp_dsp.h>

#include "ConfigLoader.hpp"

#include "1DKalman.hpp"

using dspm::Mat;

class Magnetrometer
{
    // transformation matrix for magentrometer calibration
    Mat c_matrix;
    // 3 elements vector for magnetrometer readings
    Mat readings;
    // 3 elements vector with magnetrometer offsets
    Mat offsets;

    KalmanFilter1D x_kalman;
    KalmanFilter1D y_kalman;
    KalmanFilter1D z_kalman;

    public:

    Magnetrometer()
    {
        c_matrix = Mat::eye(3);

        offsets = Mat(3,1);

        readings = Mat(3,1);

        this->x_kalman = KalmanFilter1D(0.0000089,0.00006);
        this->y_kalman = KalmanFilter1D(0.0000089,0.00006);
        this->z_kalman = KalmanFilter1D(0.000011,0.00006);
    }

    Vec3Df read()
    {   
        this->readings(0,0) = this->x_kalman.step(this->readX());
        this->readings(1,0) = this->y_kalman.step(this->readY());
        this->readings(2,0) = this->z_kalman.step(this->readZ());

        this->readings = this->readings - this->offsets;

        Mat output = this->c_matrix * readings;

        return Vec3Df(output(0,0),output(1,0),output(2,0));
    }

    void updateCFG(const config::MagConfig& cfg)
    {
        this->offsets(0,0) = cfg.offsets.x;
        this->offsets(1,0) = cfg.offsets.y;
        this->offsets(2,0) = cfg.offsets.z;

        for(uint8_t y=0;y<3;++y)
        {
            for(uint8_t x=0;x<3;++x)
            {
                this->c_matrix(y,x) = cfg.c_matrix[ y*3 + x ];
            }
        }
    }

    config::MagConfig getCFG()
    {
        config::MagConfig cfg;

        cfg.offsets.x = this->offsets(0,0);
        cfg.offsets.y = this->offsets(1,0);
        cfg.offsets.z = this->offsets(2,0);

        for(uint8_t y=0;y<3;++y)
        {
            for(uint8_t x=0;x<3;++x)
            {
                cfg.c_matrix[ y*3 + x] = this->c_matrix(y,x);
            }
        }

        return cfg;
    }

    virtual bool checkDataReady()=0;

    virtual int16_t readXRaw()=0;

    virtual int16_t readYRaw()=0;

    virtual int16_t readZRaw()=0;

    virtual float readX()=0;

    virtual float readY()=0;

    virtual float readZ()=0;

    virtual ~Magnetrometer()
    {}
};