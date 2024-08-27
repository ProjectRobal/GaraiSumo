/*
    A class that utilize Kalman Filter, for position tracking in 2D ( x and y axis )   
*/

#pragma once

#include <dsp_platform.h>

#include <esp_dsp.h>

#include <cmath>
#include "config.h"

#include "Vec.hpp"

namespace sensors
{

using dspm::Mat;

class PositionFilter
{
    //float driftTolerance;
    //float VelocityWindowThreshold;

    Mat *Q;
    // measure noise
    Mat *Ex;
    // process noise
    float R;

    Mat *H;
    // H but transponse
    Mat *Ht;

    Mat *P;

    Mat *A;
    Mat *B;

    public:

    PositionFilter()
    {

        this->Q=new Mat(2,1);
        this->Ex=new Mat(2,2);
        this->H=new Mat(2,1);
        this->P=new Mat(Mat::eye(2));
        this->A=new Mat(2,2);
        this->B=new Mat(2,1);

        (*this->H)(2,0)=1.f;

        (*this->A)(0,0)=1.f;
        (*this->A)(0,1)=SAMPLE_TIME;
        (*this->A)(1,0)=0.f;
        (*this->A)(1,1)=1.f;

        (*this->B)(0,0)=(SAMPLE_TIME*SAMPLE_TIME)/2;
        (*this->B)(1,0)=SAMPLE_TIME;

        this->Ht= new Mat(this->H->t());
    }

    void reset()
    {
        this->Q->clear();
    }

    void setFromCFG(const config::PositionFilterCFG& cfg)
    {
        (*this->Ex)(0,0)=cfg.Ex1;
        (*this->Ex)(1,1)=cfg.Ex2;

        this->R=cfg.R;
    }

    config::PositionFilterCFG getCFG()
    {
        return {
            .Ex1=(*this->Ex)(0,0),
            .Ex2=(*this->Ex)(1,1),
            .R=this->R
        };
    }

    // x - an current positions from encoders
    // u - an current accelerations from IMU
    float step(float x,float u)
    {

        Mat Q_curr=(*this->A)*(*this->Q)+(*this->B)*u;

        (*this->P)=(*this->A)*(*this->P)*this->A->t()+(*this->Ex);
        
        float error = x - Q_curr(0,0);

        Mat S=(*this->H)*(*this->P)*(*this->Ht) + this->R;

        Mat K=(*this->P)*(*this->Ht)*S.inverse();

        (*this->Q)=(*this->Q) + K*error;

        (*this->P)=(*this->P) - K*S*K.t();

        return (*this->Q)(0,0);
    }


};

}