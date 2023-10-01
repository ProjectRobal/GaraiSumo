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
    private:

    Mat A;

    Mat B;

    Mat H;

    // H but transposed
    Mat Ht;

    // process noise
    float R;
    // measure noise
    Mat Ex;    

    Mat P;

    Mat Q;

    //float _x;

    //float driftTolerance;
    //float dAngleWindowThreshold;

    public:

    RotationFilter()
    : 
    A(2,2),
    B(2,1),
    H(2,1),
    Ex(2,2),
    P(Mat::eye(2)),
    Q(2,1)
    {
        B(0,0)=-1.f/SAMPLE_FREQ;
        B(1,0)=0.f;

        A(0,0)=1.f;
        A(0,1)=-1.f/SAMPLE_FREQ;    
        A(1,0)=0.f;
        A(1,1)=1.f; 

        H(0,0)=1.f;
        H(1,0)=0.f;

        Ht=H.t();
    }

    void setFromCFG(const config::RotationFilterCFG &cfg)
    {
        // Params we need:
        // Ex1 , Ex2
        // R
        // Q0_1,Q0_2

        this->Ex(0,0)=cfg.Ex1;
        this->Ex(1,1)=cfg.Ex2;

        this->R=cfg.R;

        this->Q(0,0)=cfg.Q0_1;
        this->Q(1,0)=cfg.Q0_2;
    }

    config::RotationFilterCFG  getCFG()
    {
        // Params we need:
        // Ex1 , Ex2
        // R
        // Q0_1,Q0_2
        
        config::RotationFilterCFG filter;

        filter.Ex1=this->Ex(0,0);
        filter.Ex2=this->Ex(1,1);

        filter.R=this->R;

        filter.Q0_1=this->Q(0,0);
        filter.Q0_2=this->Q(1,0);

        return filter;
    }

    // x - d0 from encoders
    // u - rotation velocity from IMU
    float step(const float& x,const float& u)
    {
        Mat Q_curr=this->A*this->Q+this->B*u;

        this->P=this->A*this->P*this->A.t()+this->Ex;

        float error= ( this->Q(0,0) + x ) - Q_curr(0,0);

        Mat S=this->H*this->P*this->Ht + this->R;

        Mat K=this->P*this->Ht*S.inverse();
        
        this->Q=this->Q + K*error;

        this->P=this->P - K*S*K.t();

        while(this->Q(0,0)>2*M_PI)
        {
            this->Q(0,0)-=2*M_PI;
        }

        while(this->Q(0,0)<0.f)
        {
            this->Q(0,0)+=2*M_PI;
        }

        float deg=this->Q(0,0);

        return deg;   
    }

};