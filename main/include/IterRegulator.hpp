#pragma once

#include <cmath>

class IterRegulator
{
    float max_step;

    float P;

    float power;

    float last_error;

    float last_step;

    float max;
    float min;

    public:

    IterRegulator(float P,float max_step)
    {
        this->max_step = max_step;

        this->P = P;

        this->power = 0.f;

        this->last_step = 0.f;

        this->last_error = 0.f;
    }

    void setMax(float max)
    {
        this->max = max;
    }

    void setMin(float min)
    {
        this->min = min;
    }

    void filter()
    {
        if(this->power > this->max)
        {
            this->power = this->max;
        }

        if(this->power < this->min)
        {
            this->power = this->min;
        }
    }

    float step(float targetSpeed,float speed)
    {
        // if( this->last_error != 0 )
        // {
        //     this->P = this->last_step / ( error - this->last_error ) ;
        // }

        if( this->power > 0 && speed > 0 )
        {

            this->P = speed / this->power;

            this->power = this->P*targetSpeed;

        }
        else
        {
            this->power += this->max_step;
        }

        // this->last_step = error*this->P;

        // if( abs(this->last_step) > this->max_step )
        // {
        //      this->last_step = this->max_step*(this->last_step/abs(this->last_step));
        // }

        this->filter();

        return this->power;
    }

};