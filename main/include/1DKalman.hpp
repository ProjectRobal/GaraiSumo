/*

    A filter that filters incoming data from gyroscope.

*/

#pragma once

#include <dsp_platform.h>

#include <esp_dsp.h>

#include "ConfigLoader.hpp"

class KalmanFilter1D
{
    // covariance
    float V;
    float W;

    // current value
    float x;    

    // status variable

    float S;
    float P;

    public:

    KalmanFilter1D(float W,float V)
    {
        this->V = V;
        this->W = W;
    }

    KalmanFilter1D()
    {
        this->S = 0;
        this->P = 0;
        this->x = 0;
        this->V = 0;
        this->W = 0;
    }

    void reset()
    {
        this->S = 0; 
        this->P = 0;
    }

    void setFromCFG(const config::Kalman1DCFG &cfg)
    {
        // Params we need:
        // Ex1 , Ex2
        // R
        // Q0_1,Q0_2

        this->V=cfg.V;
        this->W=cfg.W;

        this->x=cfg.Q0;

        this->reset();

    }

    config::Kalman1DCFG  getCFG()
    {
        // Params we need:
        // Ex1 , Ex2
        // R
        // Q0_1,Q0_2
        
        config::Kalman1DCFG filter;

        filter.W=this->V;
        filter.V=this->W;

        filter.Q0=this->x;

        return filter;
    }

    float step(float y)
    {
        float Pp = this->P + this->V;

        float E = y - this->x;

        float S = Pp + this->W;

        float K = Pp/S;

        this->x = this->x + K*E;

        this->P = Pp - S*K*K;
        
        return this->x;   
    }

};