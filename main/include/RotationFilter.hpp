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

    // x - d0 from encoders
    // u - rotation velocity from IMU
    float step(const float& d0,const float& yaw)
    {
        float _x = this->x + d0;

        float Pp = this->P + this->V;

        float E = yaw - _x;

        float S = Pp + this->W;

        float K = Pp/S;

        this->x = _x + K*E;

        this->P = Pp - S*K*K;
        
        return this->x;   
    }

};