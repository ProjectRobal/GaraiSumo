#pragma once

#include <cmath>

/*
    An implementation of simple Beta filter.

*/

class BetaFilter
{
    // main parameter
    float Beta;

    // last smoothed data
    float last_value;

    // cut off value , if filtered value is lower than this it is set to 0.f
    float cut_off;

    public:

    BetaFilter()
    {
        this->Beta = 0.f;
        this->last_value = 0.f;
        this->cut_off = 0.f;
    }

    BetaFilter(float Beta)
    : BetaFilter()
    {
        this->Beta = Beta;
    }

    void setBeta(float Beta)
    {
        this->Beta = Beta;
    }

    float getBeta()
    {
        return this->Beta;
    }

    void setCutOff(float cut)
    {
        this->cut_off = cut;
    }

    float getCutOff()
    {
        return this->cut_off;
    }

    float next(float x)
    {
        this->last_value = this->last_value - (this->Beta * (this->last_value - x));

        if( abs(this->last_value) < this->cut_off )
        {
            this->last_value = 0.f;
        }

        return this->last_value;
    }

};