#pragma once

#include <cinttypes>

#include <esp_dsp.h>

#include "ConfigLoader.hpp"

#include "1DKalman.hpp"

#include "MeanBuffer.hpp"

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

    MeanBuffer<float,10> x_buffer;
    MeanBuffer<float,10> y_buffer;
    MeanBuffer<float,10> z_buffer;

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
        // x_buffer.push(this->readX());
        // y_buffer.push(this->readY());
        // z_buffer.push(this->readZ());

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

        this->x_kalman = KalmanFilter1D(cfg.xW,cfg.xV);
        this->y_kalman = KalmanFilter1D(cfg.yW,cfg.yV);
        this->z_kalman = KalmanFilter1D(cfg.zW,cfg.zV);

        this->x_buffer.reset();
        this->y_buffer.reset();
        this->z_buffer.reset();
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

        config::Kalman1DCFG k_cfg = this->x_kalman.getCFG();

        cfg.xV = k_cfg.V;
        cfg.xW = k_cfg.W;

        k_cfg = this->y_kalman.getCFG();

        cfg.yV = k_cfg.V;
        cfg.yW = k_cfg.W;

        k_cfg = this->z_kalman.getCFG();

        cfg.zV = k_cfg.V;
        cfg.zW = k_cfg.W;

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