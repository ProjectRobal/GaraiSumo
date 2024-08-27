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

    float S;
    float P;

    public:

    RotationFilter()
    {

    }

    void reset()
    {
        this->S = 0; 
        this->P = 0;
    }

    void setFromCFG(const config::RotationFilterCFG &cfg)
    {
        // Params we need:
        // Ex1 , Ex2
        // R
        // Q0_1,Q0_2

        this->V=cfg.Ex1;
        this->W=cfg.R;

        this->x=cfg.Q0_1;

    }

    config::RotationFilterCFG  getCFG()
    {
        // Params we need:
        // Ex1 , Ex2
        // R
        // Q0_1,Q0_2
        
        config::RotationFilterCFG filter;

        filter.Ex1=this->V;
        filter.R=this->W;

        filter.Q0_1=this->x;

        return filter;
    }

    // eyaw - yaw from encoders
    // yaw - yaw from IMU
    float step(float eyaw,float yaw)
    {
        float Pp = this->P + this->V;

        float E = yaw - eyaw;

        float S = Pp + this->W;

        float K = Pp/S;

        this->x = eyaw + K*E;

        this->P = Pp - S*K*K;
        
        return this->x;   
    }

};