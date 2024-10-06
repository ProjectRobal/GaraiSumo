/*

    A filter that filters incoming data from gyroscope.

*/

#pragma once

#include <dsp_platform.h>

#include <esp_dsp.h>

#include "ConfigLoader.hpp"

using dspm::Mat;

class RotationFilter
{
    // covariance
    float V;
    float W;



    // current value
    float x;    

    // status variable

    Mat Q;
    // measure noise
    Mat Ex;
    // process noise
    float R;

    Mat H;
    // H but transponsed
    Mat Ht;

    Mat P;

    Mat A;
    Mat B;
    
    public:

    RotationFilter()
    : Q(2,1),
    Ex(2,2),
    H(1,2),
    Ht(2,1),
    P(Mat::eye(2)),
    A(2,2),
    B(2,1)
    {
        this->H(0,0) = 1.f;

        /*
        
            A matrix:
            
            --------------
            | 1.0 | - dt |
            --------------
            | 0.0 | 1.0 |
            -------------
        
        */
        this->A(0,0) = 1.f;
        this->A(0,1) = -SAMPLE_TIME;
        this->A(1,0) = 0.f;
        this->A(1,1) = 1.f;

        /*
            B matrix:

            ------
            | dt |
            ------
            | 0 |
            -----

        */
        this->B(0,0) = SAMPLE_TIME;
        this->B(1,0) = 0.f;

        this->Ht = this->H.t();
    }

    void reset()
    {
        this->Q.clear();
        this->P = Mat::eye(2);
    }

    void setFromCFG(const config::RotationFilterCFG &cfg)
    {
        // Params we need:
        // Ex1 , Ex2
        // R
        // Q0_1,Q0_2

        this->Ex(0,0) = cfg.Ex1;
        this->Ex(1,1) = cfg.Ex2;

        this->R = cfg.R;
    }

    config::RotationFilterCFG  getCFG()
    {
        // Params we need:
        // Ex1 , Ex2
        // R
        // Q0_1,Q0_2
        
        config::RotationFilterCFG filter;

        filter.Ex1 = this->Ex(0,0);
        filter.Ex2 = this->Ex(1,1);

        filter.R = this->R;

        return filter;
    }

    // eyaw - yaw from encoders
    // w - angular velocity from IMU
    float step(float eyaw,float w)
    {
        
        Mat Q_curr = this->A*this->Q + this->B*w;

        this->P = this->A*this->P*this->A.t() + this->Ex;

        float error = eyaw - (this->H*Q_curr)(0,0);

        Mat S = this->H*this->P*this->Ht + this->R;

        Mat K = this->P*this->Ht*S.inverse();

        this->Q = Q_curr + K*error;

        this->P = this->P - K*S*K.t();
        
        return (this->H*this->Q)(0,0);   
    }

};