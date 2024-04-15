#pragma once

#include <cinttypes>

class Magnetrometer
{
    public:

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